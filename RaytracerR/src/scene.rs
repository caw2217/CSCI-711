use std::f32::consts::{E, PI};
use std::mem;
use std::rc::Rc;
use std::thread::current;
use image::Rgb;
use na::{distance, Point3, UnitVector3, Vector3};
use rand::{random, random_range, rng};
use rand::distr::Distribution;
use rand_distr::{UnitSphere, Exp};
use rand_distr::num_traits::abs;
use crate::{Camera, HitRecord, Ray, MAX_IRRADIANCE};
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
    objects: Vec<Box<dyn Object>>,
}

impl KDNode {
    pub fn new_leaf(objs: Vec<Box<dyn Object>>) -> KDNode {
        KDNode {axis: Axes::X, value: 0.0, front: None, back: None, objects: objs}
    }

    pub fn new_interior(axis: Axes, value: f32, front: KDNode, back: KDNode) -> KDNode {
        KDNode {axis, value, front: Some(Box::new(front)), back: Some(Box::new(back)), objects: vec![]}
    }

    pub fn get_node(objs: Vec<Box<dyn Object>>, voxel: AABB) -> KDNode {
        if (objs.len() <= 2) {
            return KDNode::new_leaf(objs);
        }


    }
}

pub struct World {
    pub objects: Vec<Box<dyn Object>>,
    pub kdtree: KDNode,
    lights: Vec<Light>,
    pub ambient_light: Vector3<f32>,
}

impl World {
    pub fn new(ambient_light: Vector3<f32>) -> World {
        let kdtree = KDNode::new_leaf(vec![]);
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
        let objects = mem::take(&mut self.objects);
        self.kdtree = KDNode::get_node(objects, AABB{min: Point3::new(-100.0, -100.0, -100.0), max: Point3::new(100.0,100.0, 100.0)})
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
        let scatter_coeff: Vector3<f32> = Vector3::new(0.5, 0.5, 0.5);
        let absorption_coeff: Vector3<f32> = Vector3::new(0.001, 0.001, 0.001);
        let extinction_coeff: Vector3<f32> = scatter_coeff + absorption_coeff;
        let optical_depth: Vector3<f32> = extinction_coeff.scale(max_dist);
        let transmittance: Vector3<f32> = Vector3::new(E.powf(-optical_depth.x), E.powf(-optical_depth.y), E.powf(-optical_depth.z));

        //In scatter
        let step: f32 = 0.5;
        let mut t: f32 = random_range(0.0..step);

        let mut in_transmittance: Vector3<f32> = Vector3::new(1.0, 1.0, 1.0);
        //for now, isotropic phase function
        let phase = 1.0 / (4.0 * PI);
        let mut in_scatter= Vector3::zeros();
        let mut emitted: Vector3<f32> = Vector3::zeros();

        while t < max_dist {
            let curr_point = origin + dir.scale(t);
            //calculate transmittance change
            let att = (-extinction_coeff * step).map(|x| x.exp());
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

                let shadow_ray = Ray::new(curr_point + dir_to_light.scale(0.01), dir_to_light);

                let mut v: f32 = 1.0;

                if let Some(sh) = self.spawn_ray(shadow_ray) {
                    if sh.omega < distance(&light.position, &curr_point) {
                        v = 0.0;
                    }
                }
                //not sure if i need visibility, phong is calculating shadows
                let h: f32 = 1.0/(curr_point - light.position).magnitude_squared();

                ls += (light_power * phase * h * v).component_mul(&trans_between_light);
            }

            ls += self.ambient_light;

            //multi scattering
            let N = 8;
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
                for light in &self.lights {
                    let light_power = &light.intensity;
                    let optical_depth_between_light: Vector3<f32> = extinction_coeff.scale(distance(&next_point, &light.position));
                    let trans_between_light: Vector3<f32> = Vector3::new(E.powf(-optical_depth_between_light.x), E.powf(-optical_depth_between_light.y), E.powf(-optical_depth_between_light.z));
                    let dir_to_light = UnitVector3::new_normalize(light.position - curr_point);
                    //not sure if i need visibility, phong is calculating shadows
                    let h: f32 = 1.0/(next_point - light.position).magnitude_squared();
                    let shadow_ray = Ray::new(curr_point + dir_to_light.scale(0.01), dir_to_light);

                    let mut v: f32 = 1.0;

                    if let Some(sh) = self.spawn_ray(shadow_ray) {
                        if sh.omega < distance(&light.position, &curr_point) {
                            v = 0.0;
                        }
                    }

                    li += (light_power * phase * h * v).component_mul(&trans_between_light);
                }

                //for now skip v
                let prob_r = extinction_coeff[0] * (-extinction_coeff[0] * random_r).exp();
                let prob_w = 1.0/(4.0 * PI);
                lm += (tr * phase).component_mul(&scatter_coeff).component_mul(&li).scale(1.0/(prob_w * prob_r));
            }

            emitted += in_transmittance.component_mul(&absorption_coeff).component_mul(&Vector3::new(0.1, 0.1, 0.1)) * step;

            in_scatter += in_transmittance.component_mul(&scatter_coeff).component_mul(&(ls + lm)) * step;

            t += step;
        }

        let mut total = emitted + in_scatter;

        if let Some(hr) = &first_hit {
            let material = hr.object.get_material();
            let id = IntersectData::new(&hr, viewing, &self.lights);
            let id_cpy = IntersectData::new(&hr, viewing, &self.lights);

            //Surface radiance
            let rad_vec = material.illuminate(id, &self, 1);
            let reduced_surface_radiance = transmittance.component_mul(&rad_vec);
            total += reduced_surface_radiance;
        }

        let rad_color = Rgb([total.x.min(MAX_IRRADIANCE), total.y.min(MAX_IRRADIANCE), total.z.min(MAX_IRRADIANCE)]);

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
            let rad_color = Rgb([rad_vec.x.min(MAX_IRRADIANCE), rad_vec.y.min(MAX_IRRADIANCE), rad_vec.z.min(MAX_IRRADIANCE)]);

            return rad_color;
        } else {
            return Rgb([
                self.ambient_light.x.min(MAX_IRRADIANCE),
                self.ambient_light.y.min(MAX_IRRADIANCE),
                self.ambient_light.z.min(MAX_IRRADIANCE)]);
        }
    }

    pub fn traverse_tree<'a>(curr_node: &'a KDNode, ray: &Ray) -> Option<HitRecord<'a>> {
        if (curr_node.back.is_none() && curr_node.front.is_none()) {
            let mut first_hit: Option<HitRecord> = None;
            //Check all objects for intersection, return first hit
            for object in &curr_node.objects {
                if let Some(hr) = object.intersect(&ray) {
                    if let Some(ref first_hit_record) = first_hit {
                        if hr.omega < first_hit_record.omega {
                            first_hit = Some(hr);
                        }
                    } else {
                        first_hit = Some(hr);
                    }
                }
            }

            return first_hit;
        } else {
            let plane = curr_node.plane.as_ref().unwrap();
            let split = plane.value;
             let (dir, origin) = match plane.axis {
                Axes::X => {(ray.direction.x, ray.origin.x)}
                Axes::Y => {(ray.direction.y, ray.origin.y)}
                Axes::Z => {(ray.direction.z, ray.origin.z)}
            };

            let t_split: f32 = (split - origin)/dir;

            let (near, far) = if dir >= 0.0 {
                (curr_node.front.as_ref().unwrap(), curr_node.back.as_ref().unwrap())
            } else {
                (curr_node.back.as_ref().unwrap(), curr_node.front.as_ref().unwrap())
            };

            // Case 1: split plane is beyond current interval
            if t_split > f32::INFINITY || t_split <= 0.0 {
                return Self::traverse_tree(&*near, ray);
            }

            // Case 2: split plane is before interval
            if t_split < 0.0 {
                return Self::traverse_tree(&*far, ray,);
            }

            // Case 3: we must check both
            if let Some(hit) = Self::traverse_tree(&*near, ray) {
                return Some(hit); // early exit (TA-B optimization)
            }

            return Self::traverse_tree(&*far, ray);
        }
    }

    //Spawn a ray and return a hitrecord for the first intersection, if it exists
    pub fn spawn_ray(&self, ray: Ray) -> Option<HitRecord> {
        return Self::traverse_tree(&self.kdtree, &ray);
    }
}