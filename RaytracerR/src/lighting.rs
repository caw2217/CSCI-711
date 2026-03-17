use image::Rgb;
use na::{Point3, UnitVector3, Vector2, Vector3};
use crate::{reflect, HitRecord, Ray, World};
use dyn_clone::DynClone;

pub trait Material: DynClone {
    fn illuminate(&self, id: IntersectData, world: &World) -> Vector3<f32>;
    fn is_vol(&self) -> bool;
}

dyn_clone::clone_trait_object!(Material);

pub struct Light {
    pub position: Point3<f32>,
    pub intensity: Vector3<f32>,
}

impl Light {
    pub fn new(position: Point3<f32>, intensity: Vector3<f32>) -> Self {
        Light{ position, intensity }
    }
}

pub struct IntersectData<'a> {
    pub point: Point3<f32>,
    pub viewing: UnitVector3<f32>,
    pub normal: UnitVector3<f32>,
    pub lights: &'a Vec<Light>,
}

impl<'a> IntersectData<'a> {
    pub fn new(hit_record: &HitRecord, viewing: UnitVector3<f32>, lights: &'a Vec<Light>) -> Self {
        let point = hit_record.point;
        let normal = hit_record.normal;
        
        IntersectData{point, viewing, normal, lights}
    }
}

#[derive(Clone, Copy)]
pub struct Checkerboard {
    color1: Vector3<f32>,
    color2: Vector3<f32>,
    check_size: f32,
    offset: Vector2<f32>
}

impl Checkerboard {
    pub fn new(color1: Vector3<f32>, color2: Vector3<f32>, check_size: f32, offset: Vector2<f32>) -> Self {
        Checkerboard{ color1, color2, check_size, offset }
    }
}

impl Material for Checkerboard {
    fn illuminate(&self, id: IntersectData, world: &World) -> Vector3<f32> {
        let u = (id.point.x * self.check_size + self.offset.x).floor() as i32;
        let v = (id.point.z * self.check_size + self.offset.y).floor() as i32;

        if (u + v) % 2 == 0 {
            self.color1
        } else {
            self.color2
        }
    }

    fn is_vol(&self) -> bool {
        false
    }
}

#[derive(Clone, Copy)]
pub struct Phong {
    base_color: Vector3<f32>,
    specular_color: Vector3<f32>,
    ambient_intensity: f32,
    diffuse_intensity: f32,
    specular_intensity: f32,
    specular_exponent: f32
}

impl Phong {
    pub fn new(base_color: Vector3<f32>,
               specular_color: Vector3<f32>,
               ambient_intensity: f32,
               diffuse_intensity: f32,
               specular_intensity: f32,
               specular_exponent: f32) -> Self {
        Phong { base_color, specular_color, ambient_intensity, diffuse_intensity, specular_intensity, specular_exponent }
    }
}

impl Material for Phong {

    //Returns the radiance
    fn illuminate(&self, id: IntersectData, world: &World) -> Vector3<f32> {

        let mut ambient = self.ambient_intensity * self.base_color.component_mul(&world.ambient_light);

        let mut diffuse: Vector3<f32> = Vector3::zeros();
        let mut specular: Vector3<f32> = Vector3::zeros();

        for light in id.lights {
            let incoming = UnitVector3::new_normalize(light.position-id.point);
            let reflective = reflect(-*incoming, id.normal);
            let shadow_ray = Ray::new(id.point + id.normal.scale(0.01), incoming);
            let sh_fh = world.spawn_ray(shadow_ray);
            let light_omega = (light.position - id.point).magnitude();

            //Does the shadow hit something?
            if let Some(sh_hr) = sh_fh {
                //if yes, we need to see if it is beyond the light source or before it
                if sh_hr.omega < light_omega {
                    continue;
                }
            }

            diffuse += light.intensity.component_mul(&self.base_color) * incoming.dot(&*id.normal).max(0.0);
            specular += light.intensity.component_mul(&self.specular_color) *
                reflective.dot(&*id.viewing).max(0.0).powf(self.specular_exponent);
        }

        diffuse *= self.diffuse_intensity;
        specular *= self.specular_intensity;

        return (ambient + diffuse + specular);
    }

    fn is_vol(&self) -> bool {
        false
    }
}