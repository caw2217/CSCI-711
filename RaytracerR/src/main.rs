#![allow(warnings)]

extern crate image;
extern crate nalgebra as na;
mod lighting;
mod scene;
mod primitives;
mod models;

use crate::lighting::{Checkerboard, IntersectData, Light, Material, Phong};
use crate::primitives::{Object, Sphere, Triangle, AABB};
use crate::models::load_model;
use crate::scene::{KDNode, World};
use image::buffer::ConvertBuffer;
use image::{ImageBuffer, Rgb, Rgb32FImage, RgbImage};
use na::{Point3, Rotation3, Similarity3, Translation3, UnitQuaternion, UnitVector3, Vector2, Vector3};
use std::fmt::Debug;

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
    pub const GREY : Vector3<f32> = Vector3::new(0.5, 0.5, 0.5);
}

pub struct Ray {
    pub origin: Point3<f32>,
    pub direction: UnitVector3<f32>
}

pub struct HitRecord<'a> {
    pub object: &'a dyn Object,
    pub omega: f32,
    pub is_vol: bool,
    pub normal: UnitVector3<f32>,
    pub point: Point3<f32>
}

impl<'a> HitRecord<'a> {
    pub fn new(object: &'a dyn Object, ray: &Ray, omega: f32, is_vol: bool, normal: UnitVector3<f32>,
               point: Point3<f32>) -> Self {
        return HitRecord { object, omega, is_vol, normal, point };
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

        for i in 0..world.objects.len() {
            println!("{}: {:?}", i, world.objects[i]);
        }

        world.build_kdtree();
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
                fp_buffer.put_pixel(j, i, world.spawn_vol_light_ray(r));
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

//Objects/Camera must be transformed before adding to the world
//The world will convert all its objects to camera space
fn main() {
    let mut c: Camera = Camera::new(Point3::new(-4.5, 1.6, -10.0), Vector3::z_axis(), Vector3::y_axis(), 5.0, 45.0, 480, 640);
    //c.set_rotation(45.0f32.to_radians(), 0.0, 0.0);
    //c.set_pos(-3.5, 10.0, -10.0);
    //let light1: Light = Light::new(Point3::new(-3.05, 1.4, -5.5), Vector3::new(300.0, 300.0, 300.0));
    let light1: Light = Light::new(Point3::new(0.0, 2.0, -5.5), Vector3::new(20.0, 20.0, 20.0));
    //let light2: Light = Light::new(Point3::new(10.0, 40.0, -40.0), Vector3::new(0.0, 0.0, 5.0));
    //let light3: Light = Light::new(Point3::new(-10.0, 30.0, -60.0), Vector3::new(0.0, 5.0, 0.0));
    let mut w: World = World::new(colors::SKY_BLUE);

    w.add_light(light1);
    //w.add_light(light2);
    //w.add_light(light3);

    let mat1 = Phong::new(colors::WHITE, colors::WHITE, 0.1, 0.1, 0.1, 6.0, 1.0, 0.0);
    let mat2 = Phong::new(colors::WHITE, colors::WHITE, 0.1, 0.1, 0.1, 6.0, 0.0, 0.8);
    //let mat3 = Phong::new(colors::YELLOW, colors::WHITE, 0.1, 0.1, 0.1, 6.0);
    let mat3 = Checkerboard::new(colors::RED, colors::YELLOW, 1.0, Vector2::new(0.5, 0.0));
    let mat4 = Phong::new(colors::WHITE, colors::GREY, 0.1, 0.1, 0.1, 6.0, 0.0, 0.0);

    //let s1 = Sphere::new_in_world(Point3::new(-3.05, 1.4, -6.5), 0.8, Box::new(mat1), &mut w);

    //let s2 = Sphere::new_in_world(Point3::new(-4.6, 2.0, -7.5), 1.0, Box::new(mat2), &mut w);

    let mut t1 = Triangle::new(
        Point3::new(-7.0, 0.0, 20.0),
        Point3::new(7.0, 0.0, 20.0),
        Point3::new(-7.0, 0.0, -20.0),
        Box::new(mat3),
        Similarity3::identity());
    w.add(t1);

    let mut t2 = Triangle::new(
        Point3::new(7.0, 0.0, -20.0),
        Point3::new(-7.0, 0.0, -20.0),
        Point3::new(7.0, 0.0, 20.0),
        Box::new(mat3),
        Similarity3::identity());
    w.add(t2);

    let mut t1 = Triangle::new(
        Point3::new(-7.0, 5.0, 20.0),
        Point3::new(-7.0, 5.0, -20.0),
        Point3::new(7.0, 5.0, 20.0),
        Box::new(mat4),
        Similarity3::identity());
    w.add(t1);

    let mut t2 = Triangle::new(
        Point3::new(7.0, 5.0, -20.0),
        Point3::new(7.0, 5.0, 20.0),
        Point3::new(-7.0,5.0, -20.0),
        Box::new(mat4),
        Similarity3::identity());
    w.add(t2);


    let mut t3 = Triangle::new(
        Point3::new(-7.0, 0.0, 20.0),
        Point3::new(-7.0, 0.0, -20.0),
        Point3::new(-7.0, 5.0, 20.0),
        Box::new(mat4),
        Similarity3::identity());
    w.add(t3);

    let mut t4 = Triangle::new(
        Point3::new(-7.0, 5.0, -20.0),
        Point3::new(-7.0, 5.0, 20.0),
        Point3::new(-7.0, 0.0, -20.0),
        Box::new(mat4),
        Similarity3::identity());
    w.add(t4);

    let mut t5 = Triangle::new(
        Point3::new(-2.0, 0.0, 20.0),
        Point3::new(-2.0, 0.9, 20.0),
        Point3::new(-2.0, 0.0, -20.0),
        Box::new(mat4),
        Similarity3::identity());
    w.add(t5);

    let mut t6 = Triangle::new(
        Point3::new(-2.0, 0.9, -20.0),
        Point3::new(-2.0, 0.0, -20.0),
        Point3::new(-2.0, 0.9, 20.0),
        Box::new(mat4),
        Similarity3::identity());
    w.add(t6);


    let mut t6 = Triangle::new(
        Point3::new(-2.0, 1.9, 20.0),
        Point3::new(-2.0, 5.0, 20.0),
        Point3::new(-2.0, 1.9, -20.0),
        Box::new(mat4),
        Similarity3::identity());
    w.add(t6);

    let mut t7 = Triangle::new(
        Point3::new(-2.0, 5.0, -20.0),
        Point3::new(-2.0, 1.9, -20.0),
        Point3::new(-2.0, 5.0, 20.0),
        Box::new(mat4),
        Similarity3::identity());
    w.add(t7);

    let mut t8 = Triangle::new(
        Point3::new(-2.0, 0.9, 20.0),
        Point3::new(-2.0, 1.9, 20.0),
        Point3::new(-2.0, 0.9, -4.5),
        Box::new(mat4),
        Similarity3::identity());
    w.add(t8);

    let mut t9 = Triangle::new(
        Point3::new(-2.0, 1.9, -4.5),
        Point3::new(-2.0, 0.9, -4.5),
        Point3::new(-2.0, 1.9, 20.0),
        Box::new(mat4),
        Similarity3::identity());
    w.add(t9);

    let mut t10 = Triangle::new(
        Point3::new(-2.0, 0.9, -6.5),
        Point3::new(-2.0, 1.9, -6.5),
        Point3::new(-2.0, 0.9, -20.0),
        Box::new(mat4),
        Similarity3::identity());
    w.add(t10);

    let mut t11 = Triangle::new(
        Point3::new(-2.0, 1.9, -20.0),
        Point3::new(-2.0, 0.9, -20.0),
        Point3::new(-2.0, 1.9, -6.5),
        Box::new(mat4),
        Similarity3::identity());
    w.add(t11);

    // let loaded = load_model("bun_zipper.ply");
    //
    // //let mut triangles: Vec<Triangle> = vec![];
    // let mut min = loaded[0].0;
    // let mut max = loaded[0].0;
    // for (p1, p2, p3) in loaded.iter() {
    //     min = min.inf(&p1.inf(&p2.inf(&p3)));
    //     max = max.sup(&p1.sup(&p2.sup(&p3)));
    //     let mut tri = Triangle::new(*p1, *p2, *p3, Box::new(mat4), Similarity3::identity());
    //     tri.scale(100.0);
    //     tri.translate(5.0, -10.0, -5.0);
    //     tri.rotate(0.0, 180f32.to_radians(), 0.0);
    //
    //     w.add(tri);
    //     //triangles.push(tri);
    // }





    c.snapshot(&mut w, "voltest.png");

    //let root = KDNode::get_node(w.objects,
                               // AABB{min: Point3::new(-100.0, -100.0, -100.0), max: Point3::new(100.0,100.0, 100.0)});


}
