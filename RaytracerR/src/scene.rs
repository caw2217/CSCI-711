use std::f32::consts::{E, PI};
use std::{fmt, mem};
use std::rc::Rc;
use std::thread::current;
use image::Rgb;
use na::{distance, Point3, UnitVector3, Vector3};
use rand::{random, random_range, rng};
use rand::distr::Distribution;
use rand_distr::{UnitSphere, Exp};
use rand_distr::num_traits::abs;
use crate::{Camera, HitRecord, Ray};
use crate::lighting::{IntersectData, Light, Phong};
use crate::primitives::{Object, Sphere, AABB};

pub const AXIS_X: UnitVector3<f32> = UnitVector3::new_unchecked(Vector3::new(1.0, 0.0, 0.0));
pub const AXIS_Y: UnitVector3<f32> = UnitVector3::new_unchecked(Vector3::new(0.0, 1.0, 0.0));
pub const AXIS_Z: UnitVector3<f32> = UnitVector3::new_unchecked(Vector3::new(0.0, 0.0, 1.0));

pub fn sample_sphere_uniform() -> UnitVector3<f32>
{
    let a: [f32; 3] = UnitSphere.sample(&mut rand::rng());

    let dir = UnitVector3::new_normalize(Vector3::from_column_slice(&a));

    return dir;
}

#[derive(PartialEq, Copy, Clone)]
#[derive(Debug)]
pub enum Axes {
    X,
    Y,
    Z
}

impl Axes {
    fn next(&self) -> Axes {
        match self {
            Axes::X => Axes::Y,
            Axes::Y => Axes::Z,
            Axes::Z => Axes::X,
        }
    }

    fn get(&self) -> UnitVector3<f32> {
        match self {
            Axes::X => AXIS_X,
            Axes::Y => AXIS_Y,
            Axes::Z => AXIS_Z,
        }
    }
}

pub struct KDNode {
    axis: Axes,
    value: f32,
    front: Option<Box<KDNode>>,
    back: Option<Box<KDNode>>,
    object_indices : Vec<usize>,
    voxel: AABB
}

impl fmt::Debug for KDNode {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.debug_struct("KDNode").field("axis", &self.axis).field("value", &self.value).finish()
    }
}

impl KDNode{
    pub fn new_leaf(indices: Vec<usize>, voxel: AABB) -> KDNode {
        KDNode {axis: Axes::X, value: 0.0, front: None, back: None, object_indices: indices, voxel }
    }

    pub fn new_interior(axis: Axes, value: f32, front: KDNode, back: KDNode, voxel: AABB) -> KDNode {
        KDNode {axis, value, front: Some(Box::new(front)), back: Some(Box::new(back)), object_indices: vec![], voxel}
    }

    pub fn get_node(world: &World, indices: Vec<usize>, voxel: AABB, curr_axis: Axes, depth: usize) -> KDNode {

        if (indices.len() <= 2 || depth > 32) {
            return KDNode::new_leaf(indices, voxel);
        }

        let split = 0.5;
        let value = match curr_axis {
            Axes::X => {((voxel.max.x - voxel.min.x) * split) + voxel.min.x},
            Axes::Y => {((voxel.max.y - voxel.min.y) * split) + voxel.min.y},
            Axes::Z => {((voxel.max.z - voxel.min.z) * split) + voxel.min.z},
        };
        let (left, right) = voxel.split(curr_axis, value);

        let mut objs_left: Vec<usize> = vec![];
        let mut objs_right: Vec<usize> = vec![];
        for i in indices  {
            let obj = &world.objects[i];
            if (obj.get_bounding_box().intersect(&left)) {
                objs_left.push(i);
            }
            if (obj.get_bounding_box().intersect(&right)) {
                objs_right.push(i);
            }
        }

        return Self::new_interior(curr_axis, value, Self::get_node(world, objs_left, left, curr_axis.next(), depth+1), Self::get_node(world, objs_right, right, curr_axis.next(), depth+1), voxel)
    }
}

pub struct World {
    pub objects: Vec<Box<dyn Object>>,
    pub kdtree: KDNode,
    pub lights: Vec<Light>,
    pub ambient_light: Vector3<f32>,
}

impl World {
    pub fn new(ambient_light: Vector3<f32>) -> World {
        let kdtree = KDNode::new_leaf(vec![], AABB { min: Point3::origin(), max: Point3::origin() });
        return World { objects: vec![], lights: vec![], ambient_light, kdtree};
    }

    pub fn add(&mut self, object: impl Object + 'static) {
        self.objects.push(Box::new(object));
    }
    pub fn add_light(&mut self, light: Light) {
        self.lights.push(light);
    }

    pub fn convert_all_objects(&mut self, camera: &Camera) {
        for object in &mut self.objects {
            object.convert(&camera.view);
        }

        for light in &mut self.lights {
            light.position = camera.view.transform_point(&light.position);
        }
    }

    pub fn build_kdtree(&mut self) {
        let indices: Vec<usize> = (0..self.objects.len()).collect();

        let mut objs_min: Point3<f32> = self.objects[0].get_bounding_box().min;
        let mut objs_max: Point3<f32> = self.objects[0].get_bounding_box().max;

         for object in &self.objects {
             let min = object.get_bounding_box().min;
             let max = object.get_bounding_box().max;

             objs_min = objs_min.inf(&min);
             objs_max = objs_max.inf(&max);
         }

        self.kdtree = KDNode::get_node(&self, indices, AABB{min: Point3::new(-50.0, -50.0, -50.0), max: Point3::new(50.0,50.0, 50.0)}, Axes::X, 1);
    }

    //Spawn a ray and return irradiance
    pub fn spawn_vol_light_ray(&self, ray: Ray) -> Rgb<f32> {
        let origin = ray.origin;
        let dir = ray.direction;
        let viewing = -ray.direction;
        let first_hit = self.spawn_ray(ray);

        let max_dist: f32 = if let Some(hr) = &first_hit {
            hr.omega
        } else {
            50.0
        };

        //Transmission (homogenous medium for now)
        //rough extinction coeff of air, in m^-1
        //for now assume we start and end in medium
        let scatter_coeff: Vector3<f32> = Vector3::new(0.3, 0.3, 0.3);
        let absorption_coeff: Vector3<f32> = Vector3::new(0.05, 0.05, 0.05);
        let extinction_coeff: Vector3<f32> = scatter_coeff + absorption_coeff;
        let optical_depth: Vector3<f32> = extinction_coeff.scale(max_dist);
        let transmittance: Vector3<f32> = Vector3::new(E.powf(-optical_depth.x), E.powf(-optical_depth.y), E.powf(-optical_depth.z));

        //In scatter
        let step: f32 = 0.1;
        let mut t: f32 = random_range(0.0..step);

        let mut in_transmittance: Vector3<f32> = Vector3::new(1.0, 1.0, 1.0);
        //for now, isotropic phase function
        //let phase = 1.0 / (4.0 * PI);
        let g = 0.6;
        let mut in_scatter= Vector3::zeros();
        let mut emitted: Vector3<f32> = Vector3::zeros();
        let att = (-extinction_coeff * step).map(|x| x.exp());

        while t < max_dist {
            let curr_point = origin + dir.scale(t);
            //calculate transmittance change
            in_transmittance = in_transmittance.component_mul(&att);

            //scatter coeff

            //in-scatter radiance
            //single scatter (no loop for now, doing one pass of monte carlo
            //only using point lights right now, so no need for monte carlo i think
            let mut ls: Vector3<f32> = Vector3::zeros();
            for light in &self.lights {
                let light_power = &light.intensity;
                let optical_depth_between_light: Vector3<f32> = extinction_coeff.scale(distance(&curr_point, &light.position));
                let trans_between_light: Vector3<f32> = Vector3::new(E.powf(-optical_depth_between_light.x), E.powf(-optical_depth_between_light.y), E.powf(-optical_depth_between_light.z));
                let dir_to_light = UnitVector3::new_normalize(light.position - curr_point);

                let cos_theta = dir_to_light.dot(&-dir);
                let denom = 1.0 + g * g - 2.0 * g * cos_theta;
                let phase = (1.0 / (4.0 * PI)) * ((1.0 - g*g) / denom.powf(1.5));


                let shadow_ray = Ray::new(curr_point + dir_to_light.scale(0.01), dir_to_light);

                let mut v: f32 = 1.0;

                if let Some(sh) = self.spawn_ray(shadow_ray) {
                    if sh.omega < distance(&light.position, &curr_point) {
                        v = 0.0;
                    }
                }

                //not sure if i need visibility, phong is calculating shadows
                //let h: f32 = 1.0/(curr_point - light.position).magnitude_squared();
                let h: f32 = 1.0;

                ls += (light_power * phase * h * v).component_mul(&trans_between_light);
            }

            let env_dir = -dir;
            let cos_theta = env_dir.dot(&-dir);
            let denom = 1.0 + g*g - 2.0*g*cos_theta;
            let phase_env = (1.0 / (4.0 * PI)) * ((1.0 - g*g) / denom.powf(1.5));

            ls += self.ambient_light * phase_env * 0.05;

            //multi scattering
            let N = 0;
            let mut lm: Vector3<f32> = Vector3::zeros();
            for i in 0..N {
                let random_w = sample_sphere_uniform();
                let random_r = rand_distr::Exp::new(extinction_coeff.x).unwrap().sample(&mut rng());
                let next_point = curr_point  + random_w.scale(random_r); //x prime
                //same phase
                let opt = extinction_coeff.scale(distance(&curr_point, &next_point));
                let tr = Vector3::new(-opt.x.exp(), -opt.y.exp(), -opt.z.exp());
                //scatter

                //ugh dupe code for now
                //only double scatter for now

                //multi scattering (recurs)
                // let random_w2 = sample_sphere_uniform();
                // let random_r2 = rand_distr::Exp::new(extinction_coeff.x).unwrap().sample(&mut rng());
                // let next_point2 = next_point  + random_w2.scale(random_r2); //x prime
                // //same phase
                // let opt2 = extinction_coeff.scale(distance(&next_point, &next_point2));
                // let tr2 = Vector3::new(-opt2.x.exp(), -opt2.y.exp(), -opt2.z.exp());
                //scatter

                let mut li = Vector3::zeros();
                let mut mul_phase: f32 = 0.0;
                for light in &self.lights {
                    let light_power = &light.intensity;
                    let optical_depth_between_light: Vector3<f32> = extinction_coeff.scale(distance(&next_point, &light.position));
                    let trans_between_light: Vector3<f32> = Vector3::new(E.powf(-optical_depth_between_light.x), E.powf(-optical_depth_between_light.y), E.powf(-optical_depth_between_light.z));
                    let dir_to_light = UnitVector3::new_normalize(light.position - curr_point);
                    //not sure if i need visibility, phong is calculating shadows
                    let h: f32 = 1.0/(next_point - light.position).magnitude_squared();
                    let shadow_ray = Ray::new(curr_point + dir_to_light.scale(0.01), dir_to_light);

                    let mut v: f32 = 1.0;

                    let cos_theta = dir_to_light.dot(&-dir);
                    let denom = 1.0 + g * g - 2.0 * g * cos_theta;
                    mul_phase = (1.0 / (4.0 * PI)) * ((1.0 - g*g) / denom.powf(1.5));


                    if let Some(sh) = self.spawn_ray(shadow_ray) {
                        if sh.omega < distance(&light.position, &curr_point) {
                            v = 0.0;
                        } else {
                            v = 5.0;
                        }
                    } else {
                        v = 5.0;
                    }

                    li += (light_power * mul_phase * h * v).component_mul(&trans_between_light);
                }

                //for now skip v
                let prob_r = extinction_coeff[0] * (-extinction_coeff[0] * random_r).exp();
                let prob_w = 1.0/(4.0 * PI);
                lm += (tr * mul_phase).component_mul(&scatter_coeff).component_mul(&li).scale(1.0/(prob_w * prob_r));
            }

            emitted += in_transmittance.component_mul(&absorption_coeff).component_mul(&Vector3::zeros()) * step;

            in_scatter += in_transmittance.component_mul(&scatter_coeff).component_mul(&(ls + lm)) * step * 5.0;

            t += step;
        }

        let mut total = emitted + in_scatter;

        if let Some(hr) = &first_hit {
            let material = hr.object.get_material();
            let id = IntersectData::new(&hr, viewing, &self.lights);
            // id_cpy = IntersectData::new(&hr, viewing, &self.lights);

            //Surface radiance
            let rad_vec = material.illuminate(id, &self, 1);
            let reduced_surface_radiance = transmittance.component_mul(&rad_vec);
            total += reduced_surface_radiance;
        }

        let rad_color = Rgb([total.x, total.y, total.z]);

        return rad_color;
    }

    pub fn spawn_light_ray(&self, ray: Ray) -> Rgb<f32> {
        let viewing = -ray.direction;
        let first_hit = self.spawn_ray(ray);
        //Is there a first hit record?
        if let Some(hr) = first_hit {
            let material = hr.object.get_material();
            let id = IntersectData::new(&hr, viewing, &self.lights);

            let rad_vec = material.illuminate(id, &self, 1);
            let rad_color = Rgb([rad_vec.x, rad_vec.y, rad_vec.z]);

            return rad_color;
        } else {
            return Rgb([
                self.ambient_light.x,
                self.ambient_light.y,
                self.ambient_light.z]);
        }
    }

    pub fn traverse_tree<'a>(&'a self, node: &'a KDNode, ray: &Ray, tmin: f32, tmax: f32) -> Option<HitRecord<'a>> {
        if node.front.is_none() && node.back.is_none() {
            let mut best: Option<HitRecord> = None;

            for i in &node.object_indices {
                let obj = &self.objects[*i];

                if let Some(hit) = obj.intersect(ray) {
                    if hit.omega >= tmin && hit.omega <= tmax {
                        if best.as_ref().map_or(true, |b| hit.omega < b.omega) {
                            best = Some(hit);
                        }
                    }
                }
            }

            return best;
        }

        let axis_val = match node.axis {
            Axes::X => ray.origin.x,
            Axes::Y => ray.origin.y,
            Axes::Z => ray.origin.z,
        };

        let dir_val = match node.axis {
            Axes::X => ray.direction.x,
            Axes::Y => ray.direction.y,
            Axes::Z => ray.direction.z,
        };

        if dir_val.abs() < 1e-8 {
            // ray parallel → choose one side
            let child = if axis_val <= node.value {
                node.front.as_ref().unwrap()
            } else {
                node.back.as_ref().unwrap()
            };

            return self.traverse_tree(child, ray, tmin, tmax);
        }

        let t_split = (node.value - axis_val) / dir_val;

        let (near, far) = if dir_val >= 0.0 {
            (&node.front, &node.back)
        } else {
            (&node.back, &node.front)
        };

        let near = near.as_ref().unwrap();
        let far = far.as_ref().unwrap();

        if t_split >= tmax {
            return self.traverse_tree(near, ray, tmin, tmax);
        }

        if t_split <= tmin {
            return self.traverse_tree(far, ray, tmin, tmax);
        }

        if let Some(hit) = self.traverse_tree(near, ray, tmin, t_split) {
            return Some(hit);
        }

        self.traverse_tree(far, ray, t_split, tmax)
    }

    //Spawn a ray and return a hitrecord for the first intersection, if it exists
    pub fn spawn_ray(&self, ray: Ray) -> Option<HitRecord> {
        return self.traverse_tree(&self.kdtree, &ray, 0.0, f32::INFINITY);
    }
}