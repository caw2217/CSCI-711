use na::{Point3, Similarity3, Translation3, UnitQuaternion, UnitVector3};
use crate::lighting::{IntersectData, Material};
use crate::{HitRecord, Ray, World};
use crate::scene::SubdivPlane;

#[derive(Clone)]
pub struct AABB {
    pub min: Point3<f32>,
    pub max: Point3<f32>,
}

impl AABB {
    pub fn split(&self, plane: &SubdivPlane) -> (Self, Self) {
        let back_offset = *plane.normal * (1. - plane.value);
        let front_offset = *plane.normal * plane.value;
        let back_max = self.max - back_offset;
        let front_min = self.min + front_offset;

        let back = AABB {min: self.min, max: back_max};
        let front = AABB {min: front_min, max: self.max};

        return (back, front);
    }

    pub fn intersect(&self, other: &AABB) -> bool {
        if self.min.x > other.max.x || other.min.x > self.max.x {
            return false;
        }
        if self.min.y > other.max.y || other.min.y > self.max.y {
            return false;
        }
        if self.min.z > other.max.z || other.min.z > self.max.z {
            return false;
        }

        return true;
    }
}

pub trait Object {
    fn convert(&mut self, view: &Similarity3<f32>);

    fn transform(&self) -> &Similarity3<f32>;
    fn transform_mut(&mut self) -> &mut Similarity3<f32>;

    fn get_material(&self) -> &dyn Material;

    fn get_bounding_box(&self) -> &AABB;

    fn translate(&mut self, x: f32, y: f32, z: f32) {
        let trans = Translation3::new(x, y, z);
        *self.transform_mut() = trans * *self.transform();
        self.apply_model();
    }
    fn rotate(&mut self, x: f32, y: f32, z: f32) {
        let rot = UnitQuaternion::from_euler_angles(x, y, z);
        *self.transform_mut() = rot * *self.transform();
        self.apply_model();
    }

    fn scale(&mut self, s: f32) {
        self.transform_mut().set_scaling(s);
        self.apply_model();
    }
    fn apply_model(&mut self);
    fn intersect(&self, ray: &Ray) -> Option<HitRecord>;
}

#[derive(Clone)]
pub struct Sphere {
    center: Point3<f32>,
    radius: f32,
    transform: Similarity3<f32>,
    material: Box<dyn Material>,
    bbox: AABB
}

impl Sphere {
    pub fn new(center: Point3<f32>, radius: f32, material: Box<dyn Material>, transform: Similarity3<f32>) -> Self {
        let min = center - Point3::new(-radius, -radius, -radius);
        let max = center - Point3::new(radius, radius, radius);
        let bbox = AABB {min: <Point3<f32>>::from(min), max: <Point3<f32>>::from(max) };
        return Sphere { center, radius, material, transform, bbox};
    }

    pub fn new_in_world(center: Point3<f32>, radius: f32, material: Box<dyn Material>, world: &mut World) -> Self {
        let s = Self::new(center, radius, material, Similarity3::identity());
        world.add(s.clone());
        return s;
    }

    pub fn new_transformed(center: Point3<f32>, radius: f32, rotation: UnitQuaternion<f32>, scale: f32, material: Box<dyn Material>) -> Self {
        let mut s = Self::new(center, radius, material,Similarity3::from_parts(
            Translation3::new(center.x, center.y, center.z),
            rotation,
            scale));
        s.apply_model();
        return s;
    }
}


impl Object for Sphere {
    fn convert(&mut self, view: &Similarity3<f32>) {
        //Modify internal transform
        self.transform = view * self.transform;

        //Apply internal transform
        self.apply_model();
    }

    fn transform(&self) -> &Similarity3<f32> {
        return &self.transform;
    }

    fn transform_mut(&mut self) -> &mut Similarity3<f32> {
        return &mut self.transform;
    }

    fn get_material(&self) -> &dyn Material {
        return self.material.as_ref();
    }

    fn get_bounding_box(&self) -> &AABB {
        return &self.bbox;
    }

    fn apply_model(&mut self) {
        self.radius = self.radius * self.transform.scaling();
        self.center = self.transform.isometry.translation.transform_point(&self.center);

        self.transform = Similarity3::identity();
    }

    fn intersect(&self, ray: &Ray) -> Option<HitRecord> {
        let b = 2.0 * (ray.direction.dot( &(ray.origin - self.center)));
        let c = (ray.origin - self.center).magnitude_squared() - self.radius * self.radius;

        let determinant = b * b - 4.0 * c;
        let mut omega = 0.0;
        if determinant < 0.0 {
            return None;
        } else if determinant == 0.0 {
            omega = (-b)/(2.0);
        } else {
            let root1 = (-b + determinant.sqrt()) / 2.0;
            let root2 = (-b - determinant.sqrt()) / 2.0;

            //root1 is least and positive
            if root1 < root2 && root1 > 0.0 {
                omega = root1;
            } else if root2 < root1 && root2 > 0.0 {
                omega = root2;
            } else {
                return None;
            }
        }

        let point = ray.origin + ray.direction.scale(omega);
        let normal = UnitVector3::new_normalize(point - self.center);
        return Some(HitRecord::new(self, ray, omega, normal, point));
    }
}

#[derive(Clone)]
pub struct Triangle {
    vertices: Vec<Point3<f32>>,
    normal: UnitVector3<f32>,
    transform: Similarity3<f32>,
    material: Box<dyn Material>,
    bbox: AABB,
}

impl Triangle {
    pub fn new(p1: Point3<f32>, p2:Point3<f32>, p3:Point3<f32>, material: Box<dyn Material>, transform: Similarity3<f32>) -> Self {
        let min = Point3::new(
            p1.x.min(p2.x).min(p3.x),
            p1.y.min(p2.y).min(p3.y),
            p1.z.min(p2.z).min(p3.z));
        let max = Point3::new(
            p1.x.max(p2.x).max(p3.x),
            p1.y.max(p2.y).max(p3.y),
            p1.z.max(p2.z).max(p3.z));

        //counterclockwise
        let n = (p2 - p1).cross(&(p3-p1));
        return Triangle {
            vertices: vec![p1, p2, p3],
            normal: UnitVector3::new_normalize(n),
            material,
            transform,
            bbox: AABB {min, max}
        };
    }

    pub fn new_in_world(p1: Point3<f32>, p2:Point3<f32>, p3:Point3<f32>, material: Box<dyn Material>, world: &mut World) -> Self {
        let n = (p2 - p1).cross(&(p3-p1));
        let t = Self::new(p1, p2, p3, material, Similarity3::identity());
        world.add(t.clone());
        return t;
    }

    pub fn new_transformed(p1: Point3<f32>, p2:Point3<f32>, p3:Point3<f32>, position: Point3<f32>, rotation: UnitQuaternion<f32>, scale: f32, material: Box<dyn Material>) -> Self {
        //counterclockwise
        let n = (p2 - p1).cross(&(p3-p1));
        let mut t = Self::new(p1, p2, p3, material, Similarity3::from_parts(
            Translation3::new(position.x, position.y, position.z),
            rotation,
            scale));
        t.apply_model();
        return t;
    }
}

impl Object for Triangle {
    fn convert(&mut self, view: &Similarity3<f32>) {
        //Modify internal transform
        self.transform = view * self.transform;

        //Apply internal transform
        self.apply_model();
    }

    fn transform(&self) -> &Similarity3<f32> {
        return &self.transform;
    }

    fn transform_mut(&mut self) -> &mut Similarity3<f32> {
        return &mut self.transform;
    }

    fn apply_model(&mut self) {
        self.vertices = self.vertices.iter()
            .map(|v| self.transform.transform_point(v)).collect();

        self.normal = UnitVector3::new_normalize(self.transform.transform_vector(&*self.normal));

        self.transform = Similarity3::identity();
    }

    fn get_material(&self) -> &dyn Material {
        return self.material.as_ref();
    }
    fn intersect(&self, ray: &Ray) -> Option<HitRecord> {
        let e1 = self.vertices[1] - self.vertices[0];
        let e2 = self.vertices[2] - self.vertices[0];
        let t = ray.origin - self.vertices[0];
        let p = ray.direction.cross(&e2);
        let q = t.cross(&e1);

        //let temp: Vector3<f32> = Vector3::new(q.dot(&e2), p.dot(&t), q.dot(&ray.direction));

        let denom = p.dot(&e1);

        //Prevent division by zero (no intersection)
        if denom.abs() < 0.0001 {
            return None;
        }

        let omega = q.dot(&e2) / denom;
        let u = p.dot(&t) / denom;
        let v = q.dot(&ray.direction) / denom;

        if omega < 0.0 || u < 0.0 || v < 0.0 || u + v > 1.0 {
            return None;
        }

        //println!("{}, {}, {}", omega, u, v);

        let normal = UnitVector3::new_normalize(e1.cross(&e2));
        let point = ray.origin + ray.direction.scale(omega);

        return Some(HitRecord::new(self, ray, omega, normal, point));
    }

    fn get_bounding_box(&self) -> &AABB {
        return &self.bbox;
    }
}