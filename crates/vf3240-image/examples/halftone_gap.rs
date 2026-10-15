//! Chip-seam gaps from halftone lattice phase:
//! `halftone_gap <scan.tif> [--seams x,x,...] [--band N] [--step N] [--mode y|2d]`.
//!
//! Where a printed dot screen spans a seam, the lattice phase along the row
//! direction is linear in x on both sides and jumps at the seam by the number
//! of missing pixels times the phase slope. Per seam and row band: find the
//! vertical dot period from column autocorrelation, compute the phase of that
//! frequency component per column, fit the unwrapped phase against x on each
//! side, and divide the jump by the slope. Reported per seam as the median
//! over bands, with the count and spread. Positive = pixels missing.
//!
//! `--mode 2d` works for any screen angle: the band's 2D spectrum (both
//! sides together) gives the lattice vectors; the one with the largest
//! horizontal frequency is taken, its complex amplitude is measured in
//! sliding 32-column sub-windows on each side with one (x, y) reference,
//! and straight lines fitted to the unwrapped phase on each side meet at the
//! seam with a jump of 2π·fx·gap. Mode y (the default, used for the first
//! thirteen seams) only works where the vertical component's phase advances
//! along x at a usable rate.

use std::f64::consts::PI;
use std::fs::File;
use std::io::BufReader;
use vf3240_core::unpack::Geometry;
use vf3240_image::seamcal;
use vf3240_image::tiff::Reader;

const REACH: usize = seamcal::REACH;

fn linfit(xs: &[f64], ys: &[f64]) -> (f64, f64) {
    let n = xs.len() as f64;
    let (mx, my) = (xs.iter().sum::<f64>() / n, ys.iter().sum::<f64>() / n);
    let sxx: f64 = xs.iter().map(|x| (x - mx) * (x - mx)).sum();
    let sxy: f64 = xs.iter().zip(ys).map(|(x, y)| (x - mx) * (y - my)).sum();
    let a = sxy / sxx;
    (a, my - a * mx)
}

fn unwrap(ph: &[f64]) -> Vec<f64> {
    let mut out = vec![ph[0]];
    for &p in &ph[1..] {
        let mut d = p - out.last().unwrap();
        while d > PI {
            d -= 2.0 * PI;
        }
        while d < -PI {
            d += 2.0 * PI;
        }
        out.push(out.last().unwrap() + d);
    }
    out
}

/// Phase and amplitude of the component with period `period` in one column.
fn component(col: &[f64], period: f64) -> (f64, f64) {
    let mean = col.iter().sum::<f64>() / col.len() as f64;
    let f = 2.0 * PI / period;
    let (mut re, mut im) = (0.0, 0.0);
    for (y, v) in col.iter().enumerate() {
        let a = f * y as f64;
        re += (v - mean) * a.cos();
        im -= (v - mean) * a.sin();
    }
    (im.atan2(re), (re * re + im * im).sqrt())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    let mut take = |flag: &str| -> Option<String> {
        let i = args.iter().position(|a| a == flag)?;
        let v = args.get(i + 1).cloned();
        args.drain(i..i + 2);
        v
    };
    let seams_arg = take("--seams");
    let band: usize = take("--band").and_then(|v| v.parse().ok()).unwrap_or(320);
    let step: usize = take("--step").and_then(|v| v.parse().ok()).unwrap_or(400);
    let use_2d = take("--mode").map(|m| m == "2d").unwrap_or(false);
    let verbose = args.iter().any(|a| a == "--verbose");
    args.retain(|a| a != "--verbose");
    let path = args
        .first()
        .cloned()
        .ok_or("usage: halftone_gap <scan.tif> [--seams x,x,...] [--band N] [--step N]")?;
    let mut t = Reader::new(BufReader::with_capacity(8 << 20, File::open(&path)?))?;
    let (w, h) = (t.width, t.height);
    let dpi = t.dpi.unwrap_or(2400);
    let geom = Geometry::for_dpi(dpi).ok_or("dpi")?;
    let seams: Vec<usize> = match seams_arg {
        Some(list) => list
            .split(',')
            .map(|v| v.parse())
            .collect::<Result<_, _>>()?,
        None => {
            let chip = geom.seg_pixels / 6;
            (1..18)
                .map(|k| k * chip)
                .filter(|&x| x > geom.start_pixel_offset)
                .map(|x| x - geom.start_pixel_offset)
                .filter(|&x| x + REACH < w)
                .collect()
        }
    };
    println!("{path}: {w}x{h} at {dpi} dpi; seams {seams:?}; bands of {band} rows every {step}");

    // Gather, per seam, the G columns xs-REACH..xs+REACH for every band.
    let bands: Vec<usize> = (1200..h.saturating_sub(band + 1200))
        .step_by(step)
        .collect();
    let mut gaps: Vec<Vec<(f64, f64, f64)>> = seams.iter().map(|_| Vec::new()).collect(); // (gap, period, xperiod)
    let mut cols: Vec<Vec<Vec<f64>>> = seams
        .iter()
        .map(|_| (0..2 * REACH).map(|_| Vec::with_capacity(band)).collect())
        .collect();
    for &y0 in &bands {
        for c in cols.iter_mut() {
            for col in c.iter_mut() {
                col.clear();
            }
        }
        for s in y0 / t.rows_per_strip..=(y0 + band - 1) / t.rows_per_strip {
            let data = t.strip(s)?;
            for r in 0..t.rows_in_strip(s) {
                let y = s * t.rows_per_strip + r;
                if y < y0 || y >= y0 + band {
                    continue;
                }
                let row = &data[r * w * 3..(r + 1) * w * 3];
                for (si, &xs) in seams.iter().enumerate() {
                    for (k, col) in cols[si].iter_mut().enumerate() {
                        col.push(f64::from(row[3 * (xs - REACH + k) + 1]));
                    }
                }
            }
        }
        for (si, c) in cols.iter().enumerate() {
            if use_2d {
                let mut lg = |m: &str| println!("  seam {} rows {y0}: {m}", seams[si]);
                let mut quiet = |_: &str| {};
                let r = if verbose {
                    seamcal::fit_band(c, band, &mut lg)
                } else {
                    seamcal::fit_band(c, band, &mut quiet)
                };
                if let Some(g) = r {
                    gaps[si].push(g);
                }
                continue;
            }
            // Dot period from averaged column autocorrelation (lags 12..90).
            let mut best = (0usize, 0.0f64);
            for lag in 12..90 {
                let (mut acc, mut var) = (0.0, 0.0);
                for col in c.iter().step_by(8) {
                    let mean = col.iter().sum::<f64>() / col.len() as f64;
                    for y in 0..band - lag {
                        acc += (col[y] - mean) * (col[y + lag] - mean);
                        var += (col[y] - mean) * (col[y] - mean);
                    }
                }
                let r = if var > 0.0 { acc / var } else { 0.0 };
                if r > best.1 {
                    best = (lag, r);
                }
            }
            if best.1 < 0.4 {
                continue;
            }
            // Refine the period by spectral power.
            let mut period = best.0 as f64;
            let mut power_best = 0.0;
            for k in -20..=20 {
                let p = best.0 as f64 + k as f64 * 0.05;
                let power: f64 = c.iter().step_by(4).map(|col| component(col, p).1).sum();
                if power > power_best {
                    power_best = power;
                    period = p;
                }
            }
            let comps: Vec<(f64, f64)> = c.iter().map(|col| component(col, period)).collect();
            let amax = comps.iter().map(|v| v.1).fold(0.0, f64::max);
            let amin_outside = comps
                .iter()
                .enumerate()
                .filter(|(k, _)| (*k as isize - REACH as isize).abs() > 8)
                .map(|(_, v)| v.1)
                .fold(f64::MAX, f64::min);
            if amin_outside < 0.25 * amax {
                continue;
            }
            let lx: Vec<f64> = (20..REACH - 6).map(|x| x as f64).collect();
            let rx: Vec<f64> = (REACH + 6..2 * REACH - 20).map(|x| x as f64).collect();
            let lph = unwrap(&lx.iter().map(|&x| comps[x as usize].0).collect::<Vec<_>>());
            let rph = unwrap(&rx.iter().map(|&x| comps[x as usize].0).collect::<Vec<_>>());
            let (al, bl) = linfit(&lx, &lph);
            let (ar, br) = linfit(&rx, &rph);
            if al.abs() < 0.02 || (al - ar).abs() > 0.15 * al.abs() {
                continue;
            }
            let a = (al + ar) / 2.0;
            let mut jump = (ar * REACH as f64 + br) - (al * REACH as f64 + bl);
            jump = (jump + PI).rem_euclid(2.0 * PI) - PI;
            gaps[si].push((jump / a, period, 2.0 * PI / a.abs()));
        }
    }
    println!(
        "\n{:>7} {:>5} {:>8} {:>8} {:>7}   dot period y / phase period x",
        "seam", "bands", "median", "mean", "spread"
    );
    if use_2d {
        println!(
            "{:>48}   (median vertical offset dy / shortest x period)",
            ""
        );
    }
    for (si, &xs) in seams.iter().enumerate() {
        let g = &gaps[si];
        if g.is_empty() {
            println!("{xs:>7} {:>5}   (no usable halftone band)", 0);
            continue;
        }
        let mut v: Vec<f64> = g.iter().map(|t| t.0).collect();
        v.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let med = v[v.len() / 2];
        let mean = v.iter().sum::<f64>() / v.len() as f64;
        let sd = (v.iter().map(|x| (x - med) * (x - med)).sum::<f64>() / v.len() as f64).sqrt();
        println!(
            "{xs:>7} {:>5} {med:>8.2} {mean:>8.2} {sd:>7.2}   {:.1} / {:.0}",
            v.len(),
            if use_2d {
                let mut d: Vec<f64> = g.iter().map(|t| t.1).collect();
                d.sort_by(|a, b| a.partial_cmp(b).unwrap());
                d[d.len() / 2]
            } else {
                g[0].1
            },
            g[0].2
        );
    }
    Ok(())
}
