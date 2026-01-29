
extern crate image;
extern crate nalgebra as na;

use std::collections::hash_set::Intersection;
use image::{ImageBuffer, RgbImage, Rgb};
use na::{Transform3, Point3, UnitVector3, Vector3, Scale3, Scale, Similarity3};

pub struct Ray {
    pub origin: Point3<f32>,
    pub direction: UnitVector3<f32>
}

pub struct HitRecord {
    pub point: Point3<f32>,
    pub normal: UnitVector3<f32>,
}

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
    up: UnitVector3<f32>
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
}

trait Intersects {
    fn intersect(&self, ray: &Ray) -> Option<HitRecord>;
    //fn transform(&mut self, transform: &Similarity3<f32>);
    //fn apply_mvp(&mut self, model_matrix: Matrix4<f32>, view_matrix: Matrix4<f32>, projection_matrix: Matrix4<f32>);
}

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
    //TODO attributes
}

impl World {
    pub fn new() -> World {
        return World { objects: vec![] };
    }

    pub fn add(&mut self, object: Object) {
        self.objects.push(object);
    }

    pub fn transform(&self, object: &mut Object) {
        match &mut object.shape {
            Shape::Sphere(sphere) => {
                sphere.radius = sphere.radius * object.transform.scaling();
                sphere.center = object.transform.isometry.translation.transform_point(&sphere.center);
            },
            Shape::Triangle(triangle) => {
                triangle.vertices = triangle.vertices.iter()
                    .map(|v| object.transform.transform_point(v)).collect();
            }
        }
    }
    pub fn transform_all_objects(&mut self, transform: &Similarity3<f32>) {
        for object in &mut self.objects {
            object.transform(transform);
        }
    }
}

fn main() {
    let mut world = World::new();

    let sphere1 = Sphere::new(Point3::new(0.0, 0.0, 0.0), 1.0);

    sphere1.transform(&Similarity3::identity());

    world.add(Box::new(sphere1));
    &mut world.transform(&Transform3::identity());
    //const WIDTH: u32 = 100;
    //const HEIGHT: u32 = 100;

    //let mut img: RgbImage = ImageBuffer::new(WIDTH, HEIGHT);

    //img.put_pixel(50, 50, Rgb([255, 0, 0]));

    //img.save("output/test.png").unwrap();
}
