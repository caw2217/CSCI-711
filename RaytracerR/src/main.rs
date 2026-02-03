#![allow(warnings)]
extern crate image;
extern crate nalgebra as na;

use std::collections::hash_set::Intersection;
use std::fmt::Debug;
use image::{ImageBuffer, RgbImage, Rgb};
use na::{Transform3, Point3, UnitVector3, Vector3, Scale3, Scale, Similarity3, Matrix4, UnitQuaternion, Translation3};

pub struct Ray {
    pub origin: Point3<f32>,
    pub direction: UnitVector3<f32>
}

pub struct HitRecord {
    pub point: Point3<f32>,
    pub normal: UnitVector3<f32>,
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
    lookat: UnitVector3<f32>,
    up: UnitVector3<f32>,
    view: Similarity3<f32>,
}

impl Camera {
    pub fn new(pos: Point3<f32>, lookat: UnitVector3<f32>, up: UnitVector3<f32>) -> Self {
        let rot = UnitQuaternion::face_towards(&*lookat, &*up);
        let trans = Translation3::new(pos.x, pos.y, pos.z);
        let cam_trans = Similarity3::from_parts(trans, rot, 1.0);
        return Camera {position: pos, lookat, up, view: cam_trans.inverse()}
    }
}

pub struct Object {
    //TODO material
    color: Vector3<f32>,
    shape: Shape,
    transform: Similarity3<f32>,
}

impl Object {
    pub fn new(color: Vector3<f32>, shape: Shape, transform: Similarity3<f32>) -> Object {
        return Object {color, shape, transform };
    }

    ///Convert object into camera coords
    pub fn convert(&mut self, view: &Similarity3<f32>) {
        //Modify internal transform
        self.transform = view * self.transform;

        //Apply internal transform
        self.apply_model();
    }

    fn apply_model(&mut self) {
        match &mut self.shape {
            Shape::Sphere(sphere) => {
                sphere.radius = sphere.radius * self.transform.scaling();
                sphere.center = self.transform.isometry.translation.transform_point(&sphere.center);
            },
            Shape::Triangle(triangle) => {
                triangle.vertices = triangle.vertices.iter()
                    .map(|v| self.transform.transform_point(v)).collect();
            }
        }
    }
}

impl Debug for Object {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "({}, {:?}, {})", self.color, self.shape, self.transform)
    }
}

trait Intersects {
    fn intersect(&self, ray: &Ray) -> Option<HitRecord>;
    //fn transform(&mut self, transform: &Similarity3<f32>);
    //fn apply_mvp(&mut self, model_matrix: Matrix4<f32>, view_matrix: Matrix4<f32>, projection_matrix: Matrix4<f32>);
}

#[derive(Debug, Copy, Clone)]
pub struct Sphere {
    center: Point3<f32>,
    radius: f32,
}

impl Sphere {
    pub fn new(center: Point3<f32>, radius: f32) -> Self {
        return Sphere { center, radius };
    }
}


impl Intersects for Sphere {
    fn intersect(&self, ray: &Ray) -> Option<HitRecord> {
        return Option::None;
    }

    // fn transform(&mut self, transform: &Similarity3<f32>) {
    //     self.center = transform.isometry.transform_point(&self.center);
    //
    //     self.radius = self.radius * transform.scaling();
    // }

    //fn apply_mvp(&mut self, model_matrix: Matrix4<f32>, view_matrix: Matrix4<f32>, projection_matrix: Matrix4<f32>) {
    //    let mvp = projection_matrix * view_matrix * model_matrix;

    //    self.center = mvp.transform_point(&self.center);

    //}
}

#[derive(Debug)]
pub struct Triangle {
    vertices: Vec<Point3<f32>>,
    normal: UnitVector3<f32>,
}

impl Intersects for Triangle {
    fn intersect(&self, ray: &Ray) -> Option<HitRecord> {
        return Option::None;
    }

    // fn transform(&mut self, transform: &Similarity3<f32>) {
    //     for vertex in self.vertices.iter_mut() {
    //         *vertex = transform.transform_point(vertex);
    //     }
    // }
}

struct World {
    objects: Vec<Object>,
    camera: Camera,
    //TODO attributes
}

impl World {
    pub fn new(camera: Camera) -> World {
        return World { objects: vec![], camera};
    }

    pub fn add(&mut self, object: Object) {
        self.objects.push(object);
    }

    pub fn convert_all_objects(&mut self) {
        for object in &mut self.objects {
            object.convert(&self.camera.view);
        }
    }
}

fn main() {
    let camera = Camera::new(
        Point3::new(0.0, 0.0, -10.0),
        UnitVector3::new_normalize(Vector3::new(0.0, 0.0, 1.0)),
        UnitVector3::new_normalize(Vector3::y())
    );

    let mut world = World::new(camera);

    let sphere1 = Sphere::new(Point3::new(0.0, 0.0, 0.0), 1.0);

    //here obj1 copies the sphere for ownership (want this)
    let mut obj1 = Object::new(Vector3::identity(), Shape::Sphere(sphere1), Similarity3::identity());

    println!("obj1: {:?}", obj1);
    println!("sphere1: {:?}", sphere1);

    obj1.transform.append_translation_mut(&Translation3::new(1.0, 0.0, 0.0));

    obj1.apply_model();

    println!("obj1: {:?}", obj1);
    println!("sphere1: {:?}", sphere1);

    //const WIDTH: u32 = 100;
    //const HEIGHT: u32 = 100;

    //let mut img: RgbImage = ImageBuffer::new(WIDTH, HEIGHT);

    //img.put_pixel(50, 50, Rgb([255, 0, 0]));

    //img.save("output/test.png").unwrap();
}
