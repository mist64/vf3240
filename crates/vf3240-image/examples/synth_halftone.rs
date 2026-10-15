//! Synthetic rotated dot screen with known chip-seam gaps, to validate
//! `halftone_gap`: `synth_halftone <out.tif> [--angle DEG] [--pitch PX]
//! [--seams x:gap:dy,...]`. Right of each seam the lattice is shifted left
//! by `gap` pixels (missing pixels) and down by `dy` rows.

use std::fs::File;
use std::io::BufWriter;
use vf3240_image::tiff::{Compression, Writer};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    let mut take = |flag: &str| -> Option<String> {
        let i = args.iter().position(|a| a == flag)?;
        let v = args.get(i + 1).cloned();
        args.drain(i..i + 2);
        v
    };
    let angle: f64 = take("--angle").and_then(|v| v.parse().ok()).unwrap_or(37.0);
    let pitch: f64 = take("--pitch").and_then(|v| v.parse().ok()).unwrap_or(33.0);
    let seams: Vec<(usize, f64, f64)> = take("--seams")
        .unwrap_or_else(|| "1080:3.0:0,2808:4.0:0,4536:2.0:1.0".into())
        .split(',')
        .map(|s| {
            let p: Vec<&str> = s.split(':').collect();
            (
                p[0].parse().unwrap(),
                p[1].parse().unwrap(),
                p[2].parse().unwrap(),
            )
        })
        .collect();
    let out = args.first().ok_or("usage: synth_halftone <out.tif>")?;
    let (w, h) = (5951usize, 4000usize);
    let mut wr = Writer::new(
        BufWriter::new(File::create(out)?),
        w,
        h,
        2400,
        Compression::None,
        3,
        8,
    )?;
    let (c, s) = (angle.to_radians().cos(), angle.to_radians().sin());
    let mut row = vec![0u8; w * 3];
    let mut seed = 12345u32;
    for y in 0..h {
        for x in 0..w {
            // Physical position: add the gaps of the seams left of x, and dy.
            let (mut px, mut py) = (x as f64, y as f64);
            for &(sx, g, d) in &seams {
                if x >= sx {
                    px += g;
                    py += d;
                }
            }
            let u = (px * c + py * s) / pitch;
            let v = (-px * s + py * c) / pitch;
            // Round dots: distance to the nearest lattice point.
            let (du, dv) = (u - u.round(), v - v.round());
            let r = (du * du + dv * dv).sqrt() * pitch;
            let dot = (1.0 - ((r - 9.0) / 3.0).clamp(-1.0, 1.0)) / 2.0; // soft edge, radius 9
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            let noise = ((seed >> 24) as f64 - 128.0) * 0.05;
            let val = (235.0 - 150.0 * dot + noise).clamp(0.0, 255.0) as u8;
            row[3 * x] = val;
            row[3 * x + 1] = val;
            row[3 * x + 2] = val;
        }
        wr.write_row(&row)?;
    }
    wr.finish()?;
    println!("wrote {out}: {w}x{h}, angle {angle}, pitch {pitch}, seams {seams:?}");
    Ok(())
}
