#![allow(warnings)]
extern crate image;
extern crate nalgebra as na;

use std::collections::hash_set::Intersection;
use std::fmt::Debug;
use image::{ImageBuffer, RgbImage, Rgb};
use na::{Transform3, Point3, UnitVector3, Vector3, Scale3, Scale, Similarity3, Matrix4, UnitQuaternion, Translation3, Unit, Rotation3, Quaternion};

pub mod colors {
    use image::Rgb;
    pub const RED : Rgb<u8> = Rgb([255, 0, 0]);
    pub const GREEN: Rgb<u8> = Rgb([0, 255, 0]);
    pub const BLUE: Rgb<u8> = Rgb([0, 0, 255]);
    pub const WHITE : Rgb<u8> = Rgb([255, 255, 255]);
    pub const BLACK: Rgb<u8> = Rgb([0, 0, 0]);

    pub const YELLOW: Rgb<u8> = Rgb([255, 255, 0]);
}

pub struct Ray {
    pub origin: Point3<f32>,
    pub direction: UnitVector3<f32>
}

pub struct HitRecord<'a> {
    pub object: &'a dyn Object,
    pub omega: f32,
    pub normal: UnitVector3<f32>,
}

impl<'a> HitRecord<'a> {
    pub fn new(object: &'a dyn Object, omega: f32, normal: UnitVector3<f32>) -> Self {
        return HitRecord{object, omega, normal};
    }
}

#[derive(Debug)]
pub enum Shape {
    Sphere(Sphere),
    Triangle(Triangle),
}

impl Ray {
    pub fn new(origin: Point3<f32>, direction: Vector3<f32>) -> Ray {
        return Ray { origin, direction: UnitVector3::new_normalize(direction) };
    }
}

pub struct Camera {
    position: Point3<f32>,
    view: Similarity3<f32>,
    lookat: UnitVector3<f32>,
    up: UnitVector3<f32>,
    focal_length: f32,
    img_height: u32,
    img_width: u32,
    fp_height: f32,
    fp_width: f32,
}

impl Camera {
    pub fn new(pos: Point3<f32>, lookat: UnitVector3<f32>, up: UnitVector3<f32>,focal_length: f32, vfov: f32, img_height:u32, img_width: u32) -> Self {
        let rot = UnitQuaternion::face_towards(&*lookat, &*up);
        let trans = Translation3::new(pos.x, pos.y, pos.z);
        let cam_trans = Similarity3::from_parts(trans, rot, 1.0);

        let ar: f32 = (img_width as f32 / img_height as f32);
        let h = 3.3 * focal_length * (vfov.to_radians() / 2.0).tan();
        let w = h * ar;

        return Camera {position: pos, lookat, up, view: cam_trans.inverse(), focal_length, img_height, img_width, fp_height: h,fp_width: w};
    }

    fn recalculate_view(&mut self) {
        let rot = UnitQuaternion::face_towards(&*self.lookat, &*self.up);
        let trans = Translation3::new(self.position.x, self.position.y, self.position.z);
        let cam_trans = Similarity3::from_parts(trans, rot, 1.0);

        self.view = cam_trans.inverse();
    }

    pub fn set_pos(&mut self, x: f32, y: f32, z: f32) {
        self.position = Point3::new(x, y, z);
        self.recalculate_view();
    }

    pub fn set_rotation(&mut self, x: f32, y: f32, z: f32) {
        let rot = Rotation3::from_euler_angles(45.0f32.to_radians(), 0.0, 0.0);
        let la = UnitVector3::new_normalize(rot.transform_vector(&*Vector3::z_axis()));
        let up = UnitVector3::new_normalize(rot.transform_vector(&*Vector3::y_axis()));

        self.lookat = la;
        self.up = up;

        self.recalculate_view();
    }

    pub fn snapshot(&self, world: &mut World, filename: &str) {
        world.convert_all_objects(&self);
        let h = self.fp_height;
        let w = self.fp_width;
        let pw = w/self.img_width as f32;
        let ph = h/self.img_height as f32;
        let hph = ph /2.0;
        let hpw = pw /2.0;

        let mut img: RgbImage = ImageBuffer::new(self.img_width, self.img_height);

        let mut x: f32 = -(w/2.0) + hpw;
        let mut y: f32 = (h/2.0) - hph;
        let z = self.focal_length;
        for i in 0..self.img_height {
            for j in 0..self.img_width {
                let origin = Point3::origin();
                //println!("{}", origin);
                let dir = Vector3::new(x, y, z).normalize();
                let r = Ray::new(origin, dir);
                img.put_pixel(j, i, world.spawn_ray(r));
                x += pw;
            }

            y -= ph;
            x = -(w/2.0) + hpw;
        }

        img.save(format!("output/{}", filename)).unwrap();
    }
}

trait Object {
    fn convert(&mut self, view: &Similarity3<f32>);

    fn transform(&self) -> &Similarity3<f32>;
    fn transform_mut(&mut self) -> &mut Similarity3<f32>;

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
    fn get_color(&self) -> Rgb<u8>;
    fn intersect(&self, ray: &Ray) -> Option<HitRecord>;
}

#[derive(Debug, Copy, Clone)]
pub struct Sphere {
    center: Point3<f32>,
    radius: f32,
    transform: Similarity3<f32>,
    color: Rgb<u8>
}

impl Sphere {
    pub fn new(center: Point3<f32>, radius: f32, color: Rgb<u8>) -> Self {
        return Sphere { center, radius, color, transform: Similarity3::identity() };
    }

    pub fn new_in_world(center: Point3<f32>, radius: f32, color: Rgb<u8>, world: &mut World) -> Self {
        let s = Sphere { center, radius, color, transform: Similarity3::identity() };
        world.add(s);
        return s;
    }

    pub fn new_transformed(center: Point3<f32>, radius: f32, rotation: UnitQuaternion<f32>, scale: f32, color: Rgb<u8>) -> Self {
        let mut s = Sphere { center, radius, color, transform: Similarity3::from_parts(
            Translation3::new(center.x, center.y, center.z),
            rotation,
            scale
        ) };
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

    fn apply_model(&mut self) {
        self.radius = self.radius * self.transform.scaling();
        self.center = self.transform.isometry.translation.transform_point(&self.center);

        self.transform = Similarity3::identity();
    }

    fn get_color(&self) -> Rgb<u8> {
        return self.color;
    }

    fn intersect(&self, ray: &Ray) -> Option<HitRecord> {
        let x = ray.origin.x;
        let y = ray.origin.y;
        let z = ray.origin.z;
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
            } else {
                omega = root2;
            }
        }

        let point = ray.origin + ray.direction.scale(omega);
        let normal = UnitVector3::new_normalize(point - self.center);
        return Some(HitRecord::new(self, omega, normal));
    }
}

#[derive(Debug, Clone)]
pub struct Triangle {
    vertices: Vec<Point3<f32>>,
    normal: UnitVector3<f32>,
    transform: Similarity3<f32>,
    color: Rgb<u8>
}

impl Triangle {
    pub fn new(p1: Point3<f32>, p2:Point3<f32>, p3:Point3<f32>, color: Rgb<u8>) -> Self {
        //counterclockwise
        let n = (p2 - p1).cross(&(p3-p1));
        return Triangle {
            vertices: vec![p1, p2, p3],
            normal: UnitVector3::new_normalize(n),
            color,
            transform: Similarity3::identity()
        };
    }

    pub fn new_in_world(p1: Point3<f32>, p2:Point3<f32>, p3:Point3<f32>, color: Rgb<u8>, world: &mut World) -> Self {
        let n = (p2 - p1).cross(&(p3-p1));
        let t = Triangle {
            vertices: vec![p1, p2, p3],
            normal: UnitVector3::new_normalize(n),
            color,
            transform: Similarity3::identity()
        };
        world.add(t.clone());
        return t;
    }

    pub fn new_transformed(p1: Point3<f32>, p2:Point3<f32>, p3:Point3<f32>, position: Point3<f32>, rotation: UnitQuaternion<f32>, scale: f32, color: Rgb<u8>) -> Self {
        //counterclockwise
        let n = (p2 - p1).cross(&(p3-p1));
        let mut t = Triangle {
            vertices: vec![p1, p2, p3],
            normal: UnitVector3::new_normalize(n),
            color,
            transform: Similarity3::from_parts(
                Translation3::new(position.x, position.y, position.z),
                rotation,
                scale
            )
        };
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

    fn get_color(&self) -> Rgb<u8> {
        return self.color;
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

        return Some(HitRecord::new(self, omega, normal));
    }
}

struct World {
    objects: Vec<Box<dyn Object>>,
    bg_color: Rgb<u8>,
}

impl World {
    pub fn new(bg_color: Rgb<u8>) -> World {
        return World { objects: vec![], bg_color };
    }

    pub fn add(&mut self, object: impl Object + 'static) {
        self.objects.push(Box::new(object));
    }

    pub fn convert_all_objects(&mut self, camera: &Camera) {
        for object in &mut self.objects {
            object.convert(&camera.view);
        }
    }

    pub fn spawn_ray(&self, ray: Ray) -> Rgb<u8> {
        let mut first_hit: Option<HitRecord> = None;

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

        if let Some(first_hit_record) = first_hit {
            return first_hit_record.object.get_color()
        } else {
            return self.bg_color;
        }
    }
}

//Objects/Camera must be transformed before adding to the world
//The world will convert all its objects to camera space
fn main() {
    let mut c: Camera = Camera::new(Point3::new(-4.5, 1.6, -10.0), Vector3::z_axis(), Vector3::y_axis(), 5.0, 45.0, 480, 640);
    c.set_rotation(45.0f32.to_radians(), 0.0, 0.0);
    c.set_pos(-3.5, 10.0, -10.0);
    let mut w: World = World::new(colors::BLUE);

    let s1 = Sphere::new_in_world(Point3::new(-3.35, 1.4, -7.0), 0.8, colors::RED, &mut w);

    let s2 = Sphere::new_in_world(Point3::new(-4.6, 2.0, -7.5), 1.0, colors::GREEN, &mut w);

    let mut t1 = Triangle::new(
        Point3::new(-7.0, 0.0, 20.0),
        Point3::new(-7.0, 0.0, -20.0),
        Point3::new(7.0, 0.0, 20.0),
        colors::YELLOW);

    t1.translate(0.0, 0.0, 0.0);
    t1.rotate(-1.0f32.to_radians(), 0.0, 0.0);
    w.add(t1);

    let mut t2 = Triangle::new(
        Point3::new(7.0, 0.0, -20.0),
        Point3::new(-7.0, 0.0, -20.0),
        Point3::new(7.0, 0.0, 20.0),
        colors::YELLOW);

    t2.translate(0.0, 0.0, 0.0);
    t2.rotate(-1.0f32.to_radians(), 0.0, 0.0);
    w.add(t2);

    c.snapshot(&mut w, "assign2render-moved.png");
}
