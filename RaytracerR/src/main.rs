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
use rand_distr::num_traits::Pow;

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
    pub max_illuminance: f32,
    init_world: bool,
}

pub fn abs_ilum(pixel: &Rgb<f32>) -> f32 {
    return 0.27 * pixel[0] + 0.67 * pixel[1]+ 0.06 * pixel[2];
}

pub fn log_avg_lum(fp_buffer: &Rgb32FImage) -> f32 {
    let mut sum = 0.0f32;
    let mut n = 0;
    for (x, y, pixel) in fp_buffer.enumerate_pixels() {
        sum += (1e-4 + abs_ilum(pixel)).ln();
        n += 1;
    }
    return (sum / n as f32).exp()
}

pub fn overall_lum(fp_buffer: &Rgb32FImage) -> f32 {
    let mut sum = 0.0f32;
    let mut n = 0;
    for (x, y, pixel) in fp_buffer.enumerate_pixels() {
        sum += (1e-4 + abs_ilum(pixel)).ln();
        n += 1;
    }
    return (sum / n as f32).exp()
}

pub fn max_scene_lum(fp_buffer: &Rgb32FImage) -> f32 {
    let mut max = 0.0f32;
    for (x, y, pixel) in fp_buffer.enumerate_pixels() {
        let lum = abs_ilum(pixel);
        if lum > max {
            max = lum;
        }
    }
    return max;
}

pub fn ward_sf(l_dmax: f32, l_wa: f32) -> f32 {
    return ((1.219 + (l_dmax/2.0).powf(0.4))/(1.219+l_wa.powf(0.4))).powf(2.5);
}

pub fn ward_tr(pixel: &Rgb<f32>, sf: f32) -> Rgb<f32> {
    return Rgb([sf * pixel[0], sf * pixel[1], sf * pixel[2]]);
}

pub fn rein_tr(pixel: &Rgb<f32>, key: f32, a: f32, max_lum: f32) -> Rgb<f32> {
    let coeff = a/key;
    let rs = coeff * pixel[0];
    let gs = coeff * pixel[1];
    let bs = coeff * pixel[2];

    let rr = rs/(1.0 + rs);
    let gr = gs/(1.0 + gs);
    let br = bs/(1.0 + bs);

    return Rgb([rr * max_lum, gr * max_lum, br * max_lum]);
}

pub fn adaptive_log_tr(pixel: &Rgb<f32>, lwmax: f32, lwa: f32, bias: f32, max_lum: f32) -> Rgb<f32> {
    let lw_scaled = abs_ilum(pixel)/lwa;
    let lwmax_scaled = lwmax/lwa;

    let ld = 1.0/(lwmax_scaled + 1.0).log10() + (lw_scaled + 1.0).ln()/(2.0+((lw_scaled/lwmax_scaled).powf(bias.ln()/0.5f32.ln())) * 8.0).ln();

    let rd = ld * pixel[0];
    let gd = ld * pixel[1];
    let bd = ld * pixel[2];

    return Rgb([rd, gd, bd]);
}

impl Camera {
    pub fn new(pos: Point3<f32>, lookat: UnitVector3<f32>, up: UnitVector3<f32>,focal_length: f32, vfov: f32, img_height:u32, img_width: u32, max_illuminance: f32) -> Self {
        let rot = UnitQuaternion::face_towards(&*lookat, &*up);
        let trans = Translation3::new(pos.x, pos.y, pos.z);
        let cam_trans = Similarity3::from_parts(trans, rot, 1.0);

        let ar: f32 = (img_width as f32 / img_height as f32);
        let h = 3.3 * focal_length * (vfov.to_radians() / 2.0).tan();
        let w = h * ar;

        return Camera {position: pos, lookat, up, view: cam_trans.inverse(), focal_length, img_height, img_width, fp_height: h,fp_width: w, max_illuminance, init_world: false};
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

    pub fn snapshot(&mut self, world: &mut World, filename: &str, vol: bool) {
        if (!self.init_world) {
            //world.convert_all_objects(&self);

            world.build_kdtree();
            self.init_world = true;
        }
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
                //let origin = Point3::origin();
                //println!("{}", origin);
                let dir = self.view * Vector3::new(x, y, z).normalize();
                let r = Ray::new(self.position, UnitVector3::new_normalize(dir));
                if (vol) {
                    fp_buffer.put_pixel(j, i, world.spawn_vol_light_ray(r));
                } else {
                    fp_buffer.put_pixel(j, i, world.spawn_light_ray(r));
                }
                x += pw;
            }

            y -= ph;
            x = -(w/2.0) + hpw;
        }

        let log_avg = log_avg_lum(&fp_buffer);

        let sf = ward_sf(self.max_illuminance, log_avg);
        let lwmax = max_scene_lum(&fp_buffer);

        //Tone Reproduction
        for (x, y, pixel) in fp_buffer.enumerate_pixels_mut() {
            let mut pixel_new = Rgb([0, 0, 0]);
            let mut pixel_target = Rgb([pixel[0], pixel[1], pixel[2]]);
            //Step 2: overall illuminance
            //let abs_il = abs_ilum(pixel);

            //Step 3: Compression
            pixel_target = ward_tr(pixel, sf); //Perceptual: Ward's

            //pixel_target = rein_tr(pixel, log_avg, 0.18, self.max_illuminance); //Photographic: Reinhard

            //pixel_target = adaptive_log_tr(pixel, lwmax, log_avg, 0.85, self.max_illuminance);

            //Step 4
            pixel_new[0] = ((pixel_target[0]/self.max_illuminance) * 255.0) as u8;
            pixel_new[1] = ((pixel_target[1]/self.max_illuminance) * 255.0) as u8;
            pixel_new[2] = ((pixel_target[2]/self.max_illuminance) * 255.0) as u8;
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
    let cpos = Point3::new(-6.5, 1.6, -7.5);
    let holepos = Point3::new(7.0, 1.6, -9.5);

    let dir = UnitVector3::new_normalize(cpos - holepos);

    let mut c: Camera = Camera::new(Point3::new(-6.5, 1.6, -4.5), dir, Vector3::y_axis(), 5.0, 45.0, 480, 640, 100.0);
    //c.set_rotation(45.0f32.to_radians(), 0.0, 0.0);
    //c.set_pos(-3.5, 10.0, -10.0);
    let mut light1: Light = Light::new(Point3::new(0.0, 2.0, -5.5), Vector3::new(10.0, 10.0, 10.0));
    //let light2: Light = Light::new(Point3::new(10.0, 40.0, -40.0), Vector3::new(0.0, 0.0, 5.0));
    //let light3: Light = Light::new(Point3::new(-10.0, 30.0, -60.0), Vector3::new(0.0, 5.0, 0.0));
    let mut w: World = World::new(colors::SKY_BLUE.scale(50.0));

    w.add_light(light1);
    //w.add_light(light2);
    //w.add_light(light3);

    let mat1 = Phong::new(colors::WHITE, colors::GREY, 0.1, 0.1, 0.1, 6.0, 1.0, 0.0);
    let mat2 = Phong::new(colors::WHITE, colors::WHITE, 0.1, 0.1, 0.1, 6.0, 0.0, 0.8);
    //let mat3 = Phong::new(colors::YELLOW, colors::WHITE, 0.1, 0.1, 0.1, 6.0);
    let mat3 = Checkerboard::new(colors::RED, colors::YELLOW, 1.0, Vector2::new(0.5, 0.0));
    let mat4 = Phong::new(colors::WHITE, colors::GREY, 0.1, 0.1, 0.1, 6.0, 0.0, 0.0);

    //let s1 = Sphere::new_in_world(Point3::new(-3.05, 1.4, -6.5), 0.8, Box::new(mat1), &mut w);

    //let s2 = Sphere::new_in_world(Point3::new(-4.6, 2.0, -7.5), 1.0, Box::new(mat4), &mut w);

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





    w.lights[0].intensity = Vector3::new(100.0, 100.0, 100.0);
    c.snapshot(&mut w, "wardvolmidangle3.png", false);
}
