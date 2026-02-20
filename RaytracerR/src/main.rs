#![allow(warnings)]

mod lighting;

extern crate image;
extern crate nalgebra as na;

use std::collections::hash_set::Intersection;
use std::fmt::Debug;
use image::{ImageBuffer, RgbImage, Rgb, DynamicImage, Rgb32FImage};
use image::buffer::ConvertBuffer;
use na::{Transform3, Point3, UnitVector3, Vector3, Scale3, Scale, Similarity3, Matrix4, UnitQuaternion, Translation3, Unit, Rotation3, Quaternion, Point2};
use crate::lighting::{IntersectData, Light, Material, Phong};

const MAX_IRRADIANCE: f32 = 1.0;

const TONE_SLOPE: f32 = (1.0 - 0.0)/(MAX_IRRADIANCE - 0.0);

pub fn reflect(direction: Vector3<f32>, normal: UnitVector3<f32>) -> UnitVector3<f32> {
    return UnitVector3::new_normalize(direction - 2.0 * *normal * (direction.dot(&normal)));
}

pub mod colors {
    use na::Vector3;
    pub const RED : Vector3<f32> = Vector3::new(1.0, 0.0, 0.0);
    pub const GREEN : Vector3<f32> = Vector3::new(0.0, 1.0, 0.0);
    pub const BLUE : Vector3<f32> = Vector3::new(0.0, 0.0, 1.0);
    pub const SKY_BLUE: Vector3<f32> = Vector3::new(0.53, 0.81, 0.92);
    pub const YELLOW : Vector3<f32> = Vector3::new(1.0, 1.0, 0.0);
    pub const WHITE : Vector3<f32> = Vector3::new(1.0, 1.0, 1.0);
    pub const BLACK : Vector3<f32> = Vector3::new(0.0, 0.0, 0.0);
}

pub struct Ray {
    pub origin: Point3<f32>,
    pub direction: UnitVector3<f32>
}

pub struct HitRecord<'a> {
    pub object: &'a dyn Object,
    pub omega: f32,
    pub normal: UnitVector3<f32>,
    pub point: Point3<f32>
}

impl<'a> HitRecord<'a> {
    pub fn new(object: &'a dyn Object, ray: &Ray, omega: f32, normal: UnitVector3<f32>,
               point: Point3<f32>) -> Self {
        return HitRecord { object, omega, normal, point };
    }
}

impl Ray {
    pub fn new(origin: Point3<f32>, direction: UnitVector3<f32>) -> Ray {
        return Ray { origin, direction };
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

        //use DynamicImage to convert
        let mut fp_buffer: Rgb32FImage = Rgb32FImage::new(self.img_width, self.img_height);
        let mut img: RgbImage = ImageBuffer::new(self.img_width, self.img_height);

        let mut x: f32 = -(w/2.0) + hpw;
        let mut y: f32 = (h/2.0) - hph;
        let z = self.focal_length;
        for i in 0..self.img_height {
            for j in 0..self.img_width {
                let origin = Point3::origin();
                //println!("{}", origin);
                let dir = Vector3::new(x, y, z).normalize();
                let r = Ray::new(origin, UnitVector3::new_normalize(dir));
                fp_buffer.put_pixel(j, i, world.spawn_light_ray(r));
                x += pw;
            }

            y -= ph;
            x = -(w/2.0) + hpw;
        }
        //Tone Reproduction
        for (x, y, pixel) in fp_buffer.enumerate_pixels_mut() {
            let mut pixel_new = Rgb([0, 0, 0]);
            pixel_new[0] = (pixel[0].clamp(0.0, 1.0) * 255.0) as u8;
            pixel_new[1] = (pixel[1].clamp(0.0, 1.0) * 255.0) as u8;
            pixel_new[2] = (pixel[2].clamp(0.0, 1.0) * 255.0) as u8;
            // pixel_new[0] = ((pixel[0] * TONE_SLOPE).clamp(0.0, 1.0) * 255.0) as u8;
            // pixel_new[1] = ((pixel[1] * TONE_SLOPE).clamp(0.0, 1.0) * 255.0) as u8;
            // pixel_new[2] = ((pixel[2] * TONE_SLOPE).clamp(0.0, 1.0) * 255.0) as u8;

            img.put_pixel(x, y, pixel_new);
        }

        img.save(format!("output/{}", filename)).unwrap();
    }
}

trait Object {
    fn convert(&mut self, view: &Similarity3<f32>);

    fn transform(&self) -> &Similarity3<f32>;
    fn transform_mut(&mut self) -> &mut Similarity3<f32>;

    fn get_material(&self) -> &dyn Material;

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
}

impl Sphere {
    pub fn new(center: Point3<f32>, radius: f32, material: Box<dyn Material>) -> Self {
        return Sphere { center, radius, material, transform: Similarity3::identity() };
    }

    pub fn new_in_world(center: Point3<f32>, radius: f32, material: Box<dyn Material>, world: &mut World) -> Self {
        let s = Sphere { center, radius, material, transform: Similarity3::identity() };
        world.add(s.clone());
        return s;
    }

    pub fn new_transformed(center: Point3<f32>, radius: f32, rotation: UnitQuaternion<f32>, scale: f32, material: Box<dyn Material>) -> Self {
        let mut s = Sphere { center, radius, material, transform: Similarity3::from_parts(
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

    fn get_material(&self) -> &dyn Material {
        return self.material.as_ref();
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
}

impl Triangle {
    pub fn new(p1: Point3<f32>, p2:Point3<f32>, p3:Point3<f32>, material: Box<dyn Material>) -> Self {
        //counterclockwise
        let n = (p2 - p1).cross(&(p3-p1));
        return Triangle {
            vertices: vec![p1, p2, p3],
            normal: UnitVector3::new_normalize(n),
            material,
            transform: Similarity3::identity()
        };
    }

    pub fn new_in_world(p1: Point3<f32>, p2:Point3<f32>, p3:Point3<f32>, material: Box<dyn Material>, world: &mut World) -> Self {
        let n = (p2 - p1).cross(&(p3-p1));
        let t = Triangle {
            vertices: vec![p1, p2, p3],
            normal: UnitVector3::new_normalize(n),
            material,
            transform: Similarity3::identity()
        };
        world.add(t.clone());
        return t;
    }

    pub fn new_transformed(p1: Point3<f32>, p2:Point3<f32>, p3:Point3<f32>, position: Point3<f32>, rotation: UnitQuaternion<f32>, scale: f32, material: Box<dyn Material>) -> Self {
        //counterclockwise
        let n = (p2 - p1).cross(&(p3-p1));
        let mut t = Triangle {
            vertices: vec![p1, p2, p3],
            normal: UnitVector3::new_normalize(n),
            material,
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
}

struct World {
    objects: Vec<Box<dyn Object>>,
    lights: Vec<Light>,
    ambient_light: Vector3<f32>,
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

//Objects/Camera must be transformed before adding to the world
//The world will convert all its objects to camera space
fn main() {
    let mut c: Camera = Camera::new(Point3::new(-4.5, 1.6, -10.0), Vector3::z_axis(), Vector3::y_axis(), 5.0, 45.0, 480, 640);
    //c.set_rotation(45.0f32.to_radians(), 0.0, 0.0);
    //c.set_pos(-3.5, 10.0, -10.0);
    let light1: Light = Light::new(Point3::new(-10.0, 40.0, -40.0), Vector3::new(5.0, 0.0, 0.0));
    let light2: Light = Light::new(Point3::new(10.0, 40.0, -40.0), Vector3::new(0.0, 0.0, 5.0));
    let light3: Light = Light::new(Point3::new(-10.0, 30.0, -60.0), Vector3::new(0.0, 5.0, 0.0));
    let mut w: World = World::new(colors::SKY_BLUE);

    w.add_light(light1);
    w.add_light(light2);
    w.add_light(light3);

    let mat1 = Phong::new(colors::RED, colors::WHITE, 0.1, 0.1, 0.1, 6.0);
    let mat2 = Phong::new(colors::GREEN, colors::WHITE, 0.1, 0.1, 0.1, 6.0);
    let mat3 = Phong::new(colors::YELLOW, colors::WHITE, 0.1, 0.1, 0.1, 6.0);

    let s1 = Sphere::new_in_world(Point3::new(-3.35, 1.4, -7.0), 0.8, Box::new(mat1), &mut w);

    let s2 = Sphere::new_in_world(Point3::new(-4.6, 2.0, -7.5), 1.0, Box::new(mat2), &mut w);

    let mut t1 = Triangle::new(
        Point3::new(-7.0, 0.0, 20.0),
        Point3::new(7.0, 0.0, 20.0),
        Point3::new(-7.0, 0.0, -20.0),
        Box::new(mat3));

    t1.translate(0.0, 0.0, 0.0);
    t1.rotate(-1.0f32.to_radians(), 0.0, 0.0);
    w.add(t1);

    let mut t2 = Triangle::new(
        Point3::new(7.0, 0.0, -20.0),
        Point3::new(-7.0, 0.0, -20.0),
        Point3::new(7.0, 0.0, 20.0),
        Box::new(mat3));

    t2.translate(0.0, 0.0, 0.0);
    t2.rotate(-1.0f32.to_radians(), 0.0, 0.0);
    w.add(t2);

    c.snapshot(&mut w, "assign3render-3lights.png");
}
