use image::Rgb;
use na::{Point3, UnitVector3, Vector3};
use crate::{reflect, HitRecord, Ray, World};

pub trait Material {
    fn illuminate(&self, id: IntersectData, world: &World) -> Rgb<f32>;
    fn get_color(&self) -> Rgb<u8>;
}

pub struct Light {
    pub position: Point3<f32>,
}

pub struct IntersectData {
    pub point: Point3<f32>,
    pub normal: UnitVector3<f32>,
    pub incoming: UnitVector3<f32>,
    pub reflective: UnitVector3<f32>,
    pub light: Light,
}

impl IntersectData {
    pub fn new(hit_record: &HitRecord, incoming: UnitVector3<f32>, light: Light) -> Self {
        let point = hit_record.point;
        let normal = hit_record.normal;
        let reflective = reflect(*incoming, normal);
        
        IntersectData{point, normal, incoming, reflective, light}
    }
}

pub struct Phong {
    base_color: Rgb<u8>,
    specular_color: Rgb<u8>,
    ambient_intensity: f32,
    diffuse_intensity: f32,
    specular_intensity: f32,
    specular_exponent: f32
}

impl Phong {
    pub fn new(base_color: Rgb<u8>,
               specular_color: Rgb<u8>,
               ambient_intensity: f32,
               diffuse_intensity: f32,
               specular_intensity: f32,
               specular_exponent: f32) -> Self {
        Phong{base_color, specular_color, ambient_intensity, diffuse_intensity, specular_intensity, specular_exponent}
    }
}

impl Material for Phong {
    fn illuminate(&self, id: IntersectData, world: &World) -> Rgb<f32> {
        let shadow_ray = Ray::new(id.point, id.incoming);
        let sh_fh = world.spawn_ray(shadow_ray);

        let light_omega = (id.light.position - id.point).magnitude();



        //Does the shadow hit something?
        if let Some(sh_hr) = sh_fh {
            //if yes, we need to see if it is beyond the light source or before it
        
            if sh_hr.omega < light_omega {
        
            }
        }
    }
    
    fn get_color(&self) -> Rgb<u8> {
        return self.base_color;
    }
}