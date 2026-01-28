
extern crate image;
extern crate nalgebra as na;

use std::collections::hash_set::Intersection;
use image::{ImageBuffer, RgbImage, Rgb};
use na::{Transform3, Point3, UnitVector3, Vector3};

pub struct Ray {
    pub origin: Point3<f32>,
    pub direction: UnitVector3<f32>
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
    color: UnitVector3<f32>,
    position: Point3<f32>,
}

pub trait Intersects {
    fn intersect(&self, ray: &Ray) -> Option<Object>;
}

pub struct Sphere {
    pub center: Point3<f32>,
    pub radius: f32
}

impl Intersects for Sphere {
    fn intersect(&self, ray: &Ray) -> Option<Object> {
        return Option::None;
    }
}

pub struct Triangle {
    pub vertices: Vec<Point3<f32>>,
    pub normal: UnitVector3<f32>,
}

impl Intersects for Triangle {
    fn intersect(&self, ray: &Ray) -> Option<Object> {
        return Option::None;
    }
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

    pub fn transform(object: &mut Object, transform3: Transform3<f32>) {
        object.position = transform3 * object.position;
    }

    pub fn transform_all_objects(&mut self, transform3: Transform3<f32>) {
        for object in self.objects.iter_mut() {
            Self::transform(object, transform3);
        }
    }
}

fn main() {
    const WIDTH: u32 = 100;
    const HEIGHT: u32 = 100;

    let mut img: RgbImage = ImageBuffer::new(WIDTH, HEIGHT);

    img.put_pixel(50, 50, Rgb([255, 0, 0]));

    img.save("output/test.png").unwrap();
}
