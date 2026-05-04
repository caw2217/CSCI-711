use std::fs::File;
use na::{Point3, Similarity3};
use ply_rs::ply;
use ply_rs::parser::Parser;
use ply_rs::ply::Property;
use crate::primitives::Triangle;

pub fn load_model(filename: &str) -> Vec<(Point3<f32>, Point3<f32>, Point3<f32>)> {
    let mut f = File::open(format!("models/{}", filename)).unwrap();
    let parser = Parser::<ply::DefaultElement>::new();
    let bunny_model = parser.read_ply(&mut f).unwrap();

    // Access vertices
    if let Some(vertex_element) = bunny_model.payload.get("vertex") {
        println!("Loaded {} vertices", vertex_element.len());
    }
    // Access faces
    if let Some(face_element) = bunny_model.payload.get("face") {
        println!("Loaded {} faces", face_element.len());
    }

    let vertex_data = &bunny_model.payload["vertex"];

    let vertices: Vec<Point3<f32>> = bunny_model.payload["vertex"]
        .iter()
        .map(|v| {
            let x = if let Property::Float(f) = v["x"] { f } else { 0.0 };
            let y = if let Property::Float(f) = v["y"] { f } else { 0.0 };
            let z = if let Property::Float(f) = v["z"] { f } else { 0.0 };
            Point3::new(x, y, z)
        }).collect();

    let mut triangles: Vec<(Point3<f32>, Point3<f32>, Point3<f32>)> = Vec::new();
    let face_data = &bunny_model.payload["face"];

    for face in face_data {
        if let Property::ListInt(ref indices) = face["vertex_indices"] {
            if indices.len() == 3 {
                triangles.push((
                    vertices[indices[0] as usize],
                    vertices[indices[1] as usize],
                    vertices[indices[2] as usize],
                    ));
            }
        }
    }

    return triangles;
}
