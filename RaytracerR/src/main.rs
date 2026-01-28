
extern crate image;
extern crate nalgebra as na;

use image::{ImageBuffer, RgbImage, Rgb};

pub struct Camera {
    position: na::Point3<f32>,
    lookat: na::Vector3<f32>,
    up: na::Vector3<f32>
} 

fn main() {
    const WIDTH: u32 = 100;
    const HEIGHT: u32 = 100;

    let mut img: RgbImage = ImageBuffer::new(WIDTH, HEIGHT);

    img.put_pixel(50, 50, Rgb([255, 0, 0]));

    img.save("output/test.png").unwrap();
}
