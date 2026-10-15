//! Lateral and vertical colour registration measurement:
//! `fringe <scan.tif> [--bins N] [--step N] [--out offsets.txt] [--dpi N]`.
//!
//! Reads an 8-bit RGB TIFF written by `vf3240`, finds sharp edges on
//! sampled rows (vertical edges → X offsets) and sampled columns (horizontal
//! edges → Y offsets), and for each edge measures the sub-pixel 50 % crossing
//! of R, G and B. Prints the mean R−G and B−G offsets in pixels (positive =
//! the channel's edge lies right of / below G's) per column bin and per CIS
//! segment (boundaries from the dpi in the file). Only edges with strong
//! contrast in all three channels count, so a black-and-white page is the
//! ideal input. `--out` writes one line per bin: `X|Y x0 x1 n dr db`.

use std::fs::File;
use std::io::{BufReader, Write};
use vf3240_core::unpack::Geometry;
use vf3240_image::tiff::Reader;

/// lo/hi are taken over ±WIDE, the crossing is searched within ±NEAR of the
/// G gradient peak; edges closer than WIDE to each other are skipped.
const WIDE: usize = 10;
const NEAR: usize = 6;
const MIN_CONTRAST: i32 = 90;
const MIN_GRAD: i32 = 40;

/// Sub-pixel position where `v` (sampled at 0..len) crosses the midpoint of
/// its min and max over the whole window, searched within the central ±NEAR.
fn crossing(v: &[i32]) -> Option<f64> {
    let (lo, hi) = (*v.iter().min().unwrap(), *v.iter().max().unwrap());
    if hi - lo < MIN_CONTRAST {
        return None;
    }
    let mid = f64::from(lo + hi) / 2.0;
    let c = v.len() / 2;
    let mut best: Option<f64> = None;
    for i in c - NEAR..c + NEAR {
        let (a, b) = (f64::from(v[i]), f64::from(v[i + 1]));
        if (a - mid) * (b - mid) <= 0.0 && a != b {
            let pos = i as f64 + (mid - a) / (b - a);
            if best.map_or(true, |p| (pos - c as f64).abs() < (p - c as f64).abs()) {
                best = Some(pos);
            }
        }
    }
    best
}

#[derive(Default, Clone, Copy)]
struct Acc {
    n: u32,
    r: f64,
    r2: f64,
    b: f64,
    b2: f64,
}
impl Acc {
    fn add(&mut self, dr: f64, db: f64) {
        self.n += 1;
        self.r += dr;
        self.r2 += dr * dr;
        self.b += db;
        self.b2 += db * db;
    }
    /// (mean R−G, sd, mean B−G, sd)
    fn stats(&self) -> (f64, f64, f64, f64) {
        let n = f64::from(self.n.max(1));
        let (mr, mb) = (self.r / n, self.b / n);
        (
            mr,
            (self.r2 / n - mr * mr).max(0.0).sqrt(),
            mb,
            (self.b2 / n - mb * mb).max(0.0).sqrt(),
        )
    }
}

/// Measure one 1-D RGB profile (interleaved RGB samples); calls `emit(pos, dr, db)`
/// for every usable edge, `pos` being the G crossing position.
fn profile_edges(rgb: &[u8], mut emit: impl FnMut(f64, f64, f64)) {
    let n = rgb.len() / 3;
    let g = |i: usize| i32::from(rgb[3 * i + 1]);
    let mut i = WIDE;
    while i + WIDE + 1 < n {
        let grad = (g(i + 1) - g(i - 1)).abs();
        if grad >= MIN_GRAD && grad >= (g(i + 2) - g(i)).abs() && grad > (g(i) - g(i - 2)).abs() {
            let win = |c: usize| {
                (i - WIDE..=i + WIDE)
                    .map(|k| i32::from(rgb[3 * k + c]))
                    .collect::<Vec<_>>()
            };
            if let (Some(pr), Some(pg), Some(pb)) =
                (crossing(&win(0)), crossing(&win(1)), crossing(&win(2)))
            {
                let (dr, db) = (pr - pg, pb - pg);
                if dr.abs() < 4.0 && db.abs() < 4.0 {
                    emit(i as f64 - WIDE as f64 + pg, dr, db);
                }
            }
            i += WIDE;
        }
        i += 1;
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    let mut take = |flag: &str| -> Option<String> {
        let i = args.iter().position(|a| a == flag)?;
        let v = args.get(i + 1).cloned();
        args.drain(i..i + 2);
        v
    };
    let bins: usize = take("--bins").and_then(|v| v.parse().ok()).unwrap_or(48);
    let step: usize = take("--step").and_then(|v| v.parse().ok()).unwrap_or(8);
    let out = take("--out");
    let dpi_override: Option<u32> = take("--dpi").and_then(|v| v.parse().ok());
    let path = args
        .first()
        .cloned()
        .ok_or("usage: fringe <scan.tif> [--bins N] [--step N] [--out offsets.txt]")?;
    let mut t = Reader::new(BufReader::with_capacity(8 << 20, File::open(&path)?))?;
    let (w, h) = (t.width, t.height);
    let dpi = dpi_override
        .or(t.dpi)
        .ok_or("TIFF has no resolution tag; pass --dpi")?;
    let geom = Geometry::for_dpi(dpi).ok_or("unsupported dpi")?;
    let seg_of = |x: usize| -> usize {
        let rx = x + geom.start_pixel_offset;
        if rx < geom.seg_pixels {
            1
        } else if rx < 2 * geom.seg_pixels {
            2
        } else {
            0
        }
    };
    println!(
        "{path}: {w}x{h} at {dpi} dpi, {} strips; CIS segment boundaries at image x = {} and {}",
        t.strips(),
        geom.seg_pixels - geom.start_pixel_offset,
        2 * geom.seg_pixels - geom.start_pixel_offset
    );

    let mut x_bins = vec![Acc::default(); bins];
    let mut y_bins = vec![Acc::default(); bins];
    let mut x_seg = [Acc::default(); 3];
    let mut y_seg = [Acc::default(); 3];
    let mut x_third = [Acc::default(); 3];
    let mut col_profiles: Vec<Vec<u8>> = (0..w / step).map(|_| Vec::with_capacity(3 * h)).collect();
    for s in 0..t.strips() {
        let data = t.strip(s)?;
        let rows = t.rows_in_strip(s);
        for r in (0..rows).step_by(step) {
            let y = s * t.rows_per_strip + r;
            let row = &data[r * w * 3..(r + 1) * w * 3];
            profile_edges(row, |pos, dr, db| {
                let x = (pos.max(0.0) as usize).min(w - 1);
                x_bins[x * bins / w].add(dr, db);
                x_seg[seg_of(x)].add(dr, db);
                x_third[(y * 3 / h).min(2)].add(dr, db);
            });
        }
        // Column profiles for the Y measurement: every `step`th column, all rows.
        for r in 0..rows {
            let row = &data[r * w * 3..(r + 1) * w * 3];
            for (k, prof) in col_profiles.iter_mut().enumerate() {
                let x = k * step;
                prof.extend_from_slice(&row[3 * x..3 * x + 3]);
            }
        }
    }
    for (k, prof) in col_profiles.iter().enumerate() {
        let x = k * step;
        profile_edges(prof, |_, dr, db| {
            y_bins[x * bins / w].add(dr, db);
            y_seg[seg_of(x)].add(dr, db);
        });
    }

    let total: u32 = x_bins.iter().map(|a| a.n).sum();
    println!("\nX offsets ({total} vertical edges on every {step}th row; positive = channel edge right of G):");
    println!(
        "{:>4} {:>7} {:>7} {:>3} {:>7} {:>7} {:>5} {:>7} {:>5}",
        "bin", "x0", "x1", "seg", "n", "R-G", "sd", "B-G", "sd"
    );
    let mut file = out.map(File::create).transpose()?;
    for (i, a) in x_bins.iter().enumerate() {
        let (x0, x1) = (i * w / bins, (i + 1) * w / bins - 1);
        let (mr, sr, mb, sb) = a.stats();
        println!(
            "{i:>4} {x0:>7} {x1:>7} {:>3} {:>7} {mr:>7.3} {sr:>5.2} {mb:>7.3} {sb:>5.2}",
            seg_of((x0 + x1) / 2),
            a.n
        );
        if let Some(f) = file.as_mut() {
            writeln!(f, "X {x0} {x1} {} {mr:.4} {mb:.4}", a.n)?;
        }
    }
    let ytotal: u32 = y_bins.iter().map(|a| a.n).sum();
    println!("\nY offsets ({ytotal} horizontal edges on every {step}th column; positive = channel edge below G):");
    for (i, a) in y_bins.iter().enumerate() {
        let (x0, x1) = (i * w / bins, (i + 1) * w / bins - 1);
        let (mr, sr, mb, sb) = a.stats();
        if a.n > 0 {
            println!(
                "{i:>4} {x0:>7} {x1:>7} {:>3} {:>7} {mr:>7.3} {sr:>5.2} {mb:>7.3} {sb:>5.2}",
                seg_of((x0 + x1) / 2),
                a.n
            );
        }
        if let Some(f) = file.as_mut() {
            writeln!(f, "Y {x0} {x1} {} {mr:.4} {mb:.4}", a.n)?;
        }
    }
    println!("\nper CIS segment (image order: 1, 2, 0):");
    for seg in [1usize, 2, 0] {
        let (mr, sr, mb, sb) = x_seg[seg].stats();
        let (yr, ysr, yb, ysb) = y_seg[seg].stats();
        println!(
            "  seg {seg}: X n {:>7}  R-G {mr:>7.3} ± {sr:.2}  B-G {mb:>7.3} ± {sb:.2}   |  Y n {:>6}  R-G {yr:>7.3} ± {ysr:.2}  B-G {yb:>7.3} ± {ysb:.2}",
            x_seg[seg].n, y_seg[seg].n
        );
    }
    println!("per page third (top/middle/bottom), X:");
    for (i, a) in x_third.iter().enumerate() {
        let (mr, sr, mb, sb) = a.stats();
        println!(
            "  third {i}: n {:>7}  R-G {mr:>7.3} ± {sr:.2}   B-G {mb:>7.3} ± {sb:.2}",
            a.n
        );
    }
    Ok(())
}
