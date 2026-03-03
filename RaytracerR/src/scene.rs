use std::rc::Rc;
use image::Rgb;
use na::{UnitVector3, Vector3};
use crate::{Camera, HitRecord, Ray, MAX_IRRADIANCE};
use crate::lighting::{IntersectData, Light};
use crate::primitives::{Object, AABB};

pub const AXIS_X: UnitVector3<f32> = UnitVector3::new_unchecked(Vector3::new(1.0, 0.0, 0.0));
pub const AXIS_Y: UnitVector3<f32> = UnitVector3::new_unchecked(Vector3::new(0.0, 1.0, 0.0));
pub const AXIS_Z: UnitVector3<f32> = UnitVector3::new_unchecked(Vector3::new(0.0, 0.0, 1.0));

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

pub struct SubdivPlane {
    pub normal: UnitVector3<f32>,
    pub value: f32,
}

pub struct KDNode {
    plane: Option<SubdivPlane>,
    front: Option<Box<KDNode>>,
    back: Option<Box<KDNode>>,
    objects: Vec<Rc<dyn Object>>,
}

impl KDNode {
    pub fn new_leaf(objs: Vec<Rc<dyn Object>>) -> KDNode {
        KDNode {plane: None, front: None, back: None, objects: objs}
    }

    pub fn new_interior(plane: SubdivPlane, front: KDNode, back: KDNode) -> KDNode {
        KDNode {plane: Some(plane), front: Some(Box::new(front)), back: Some(Box::new(back)), objects: vec![]}
    }

    pub fn get_node(objs: Vec<Box<dyn Object>>, voxel: AABB) -> KDNode {
        let shared: Vec<Rc<dyn Object>> = objs.into_iter().map(
            |o| o.into()).collect();

        return Self::get_node_inner(shared, voxel, Axes::X);
    }

    fn get_node_inner(objs: Vec<Rc<dyn Object>>, voxel: AABB, axis: Axes) -> KDNode {
        //for now, terminate when 1 object
        if (objs.len() <= 1) {
            return Self::new_leaf(objs);
        }
        let d = &voxel.max.coords;
        let total = (axis.get().dot(&voxel.max.coords) - axis.get().dot(&voxel.min.coords)).abs();
        //let med = ;
        let plane = SubdivPlane {normal: axis.get(), value: 0.5};
        let (vfront, vback) = voxel.split(&plane);

        let mut ofront: Vec<Rc<dyn Object>> = vec![];
        let mut oback: Vec<Rc<dyn Object>> = vec![];

        for obj in objs {
            let shared:Rc<dyn Object> = obj.into();
            if (vfront.intersect(shared.get_bounding_box())) {
                ofront.push(Rc::clone(&shared));
            }

            if (vback.intersect(shared.get_bounding_box())) {
                oback.push(shared);
            }
        }
        return Self::new_interior(plane, Self::get_node_inner(ofront, vfront, axis.next()),
                                  Self::get_node_inner(oback, vback, axis.next()));
    }
}

pub struct World {
    pub objects: Vec<Box<dyn Object>>,
    lights: Vec<Light>,
    pub ambient_light: Vector3<f32>,
}

impl World {
    pub fn new(ambient_light: Vector3<f32>) -> World {
        return World { objects: vec![], lights: vec![], ambient_light};
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

    //Spawn a ray and return irradiance
    pub fn spawn_light_ray(&self, ray: Ray) -> Rgb<f32> {
        let viewing = -ray.direction;
        let first_hit = self.spawn_ray(ray);
        //Is there a first hit record?
        if let Some(hr) = first_hit {
            let material = hr.object.get_material();
            let id = IntersectData::new(&hr, viewing, &self.lights);

            let rad_vec = material.illuminate(id, &self);
            let rad_color = Rgb([rad_vec.x.min(MAX_IRRADIANCE), rad_vec.y.min(MAX_IRRADIANCE), rad_vec.z.min(MAX_IRRADIANCE)]);

            return rad_color;
        } else {
            return Rgb([
                self.ambient_light.x.min(MAX_IRRADIANCE),
                self.ambient_light.y.min(MAX_IRRADIANCE),
                self.ambient_light.z.min(MAX_IRRADIANCE)]);
        }
    }

    //Spawn a ray and return a hitrecord for the first intersection, if it exists
    pub fn spawn_ray(&self, ray: Ray) -> Option<HitRecord> {
        let mut first_hit: Option<HitRecord> = None;

        //Check all objects for intersection, return first hit
        for object in &self.objects {
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
    }
}