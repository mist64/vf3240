//! Vertical step at chip seams from horizontal edges:
//! `seamdy <scan.tif> [--seams x,x,...] [--reach N] [--chunk N]`.
//!
//! For each seam the G channel of the columns seam ± (reach + 8) is kept
//! for the whole image. In every chunk of rows, each column's edge is the
//! row of strongest light-to-dark gradient (top edge of a dark rule)
//! refined by the gradient centroid over ±6 rows. A straight line is fitted through the edge
//! positions on each side of the seam (so a skewed rule is not mistaken
//! for a step) and evaluated at the seam; the step is right minus left.
//! A chunk counts only if both fits are tight (residual < 0.8 px) and the
//! two slopes agree. Reported as the median over chunks, with the count and
//! spread. Positive = the right-hand chip sees the paper lower (later).

use std::fs::File;
use std::io::BufReader;
use vf3240_core::unpack::Geometry;
use vf3240_image::tiff::Reader;

fn fit(pts: &[(f64, f64)]) -> (f64, f64, f64) {
    let n = pts.len() as f64;
    let mx = pts.iter().map(|p| p.0).sum::<f64>() / n;
    let my = pts.iter().map(|p| p.1).sum::<f64>() / n;
    let sxx: f64 = pts.iter().map(|p| (p.0 - mx) * (p.0 - mx)).sum();
    let sxy: f64 = pts.iter().map(|p| (p.0 - mx) * (p.1 - my)).sum();
    let a = sxy / sxx;
    let b = my - a * mx;
    let resid = (pts
        .iter()
        .map(|p| (p.1 - (a * p.0 + b)).powi(2))
        .sum::<f64>()
        / n)
        .sqrt();
    (a, b, resid)
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
    let reach: usize = take("--reach").and_then(|v| v.parse().ok()).unwrap_or(120);
    let chunk: usize = take("--chunk").and_then(|v| v.parse().ok()).unwrap_or(400);
    let verbose = args.iter().any(|a| a == "--verbose");
    args.retain(|a| a != "--verbose");
    let path = args
        .first()
        .cloned()
        .ok_or("usage: seamdy <scan.tif> [--seams x,x,...] [--reach N] [--chunk N]")?;
    let mut t = Reader::new(BufReader::with_capacity(8 << 20, File::open(&path)?))?;
    let (w, h) = (t.width, t.height);
    let dpi = t.dpi.unwrap_or(2400);
    let geom = Geometry::for_dpi(dpi).ok_or("dpi")?;
    let half = reach + 8;
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
                .collect()
        }
    };
    let seams: Vec<usize> = seams
        .into_iter()
        .filter(|&x| x >= half && x + half < w)
        .collect();
    println!("{path}: {w}x{h}; seams {seams:?}; reach {reach}, chunks of {chunk} rows");
    // G channel of the 2·half columns around each seam, all rows.
    let cols = 2 * half;
    let mut g: Vec<Vec<u8>> = seams.iter().map(|_| vec![0u8; cols * h]).collect();
    for s in 0..t.strips() {
        let data = t.strip(s)?;
        for r in 0..t.rows_in_strip(s) {
            let y = s * t.rows_per_strip + r;
            let row = &data[r * w * 3..(r + 1) * w * 3];
            for (si, &xs) in seams.iter().enumerate() {
                let dst = &mut g[si][y * cols..(y + 1) * cols];
                for (k, d) in dst.iter_mut().enumerate() {
                    *d = row[3 * (xs - half + k) + 1];
                }
            }
        }
    }
    println!(
        "\n{:>7} {:>6} {:>8} {:>7}   (step in rows; edge slope px/px)",
        "seam", "edges", "median", "spread"
    );
    for (si, &xs) in seams.iter().enumerate() {
        let img = &g[si];
        let px = |x: usize, y: usize| f64::from(img[y * cols + x]);
        let mut steps: Vec<(f64, f64)> = Vec::new();
        let mut y0 = 0;
        while y0 + chunk <= h {
            let rows = y0 + 7..y0 + chunk - 7;
            // Per column: strongest |gradient| row in the chunk, centroid-refined.
            let edge = |x: usize| -> Option<f64> {
                // Light-to-dark going down (the top edge of a dark rule), so
                // the two edges of a thick rule are never mixed.
                let grad = |y: usize| (px(x, y - 1) - px(x, y + 1)).max(0.0);
                let (mut by, mut bg) = (0usize, 0.0f64);
                for y in rows.clone() {
                    let gv = grad(y);
                    if gv > bg {
                        bg = gv;
                        by = y;
                    }
                }
                if bg < 30.0 {
                    return None;
                }
                let (mut num, mut den) = (0.0, 0.0);
                for y in by - 6..=by + 6 {
                    let gv = grad(y);
                    num += gv * y as f64;
                    den += gv;
                }
                Some(num / den)
            };
            if verbose {
                let x = half;
                let (mut lo, mut hi, mut gmax) = (255.0f64, 0.0f64, 0.0f64);
                for y in rows.clone() {
                    lo = lo.min(px(x, y));
                    hi = hi.max(px(x, y));
                    gmax = gmax.max(px(x, y - 1) - px(x, y + 1));
                }
                println!("    rows {y0}: centre column range {lo}..{hi}, max top gradient {gmax}");
            }
            let side = |xr: std::ops::Range<usize>| -> Option<(f64, f64)> {
                let pts: Vec<(f64, f64)> = xr
                    .clone()
                    .filter_map(|x| edge(x).map(|e| (x as f64, e)))
                    .collect();
                if pts.len() < xr.len() * 3 / 4 {
                    if verbose && !pts.is_empty() {
                        println!(
                            "    rows {y0}: only {} of {} columns have an edge",
                            pts.len(),
                            xr.len()
                        );
                    }
                    return None;
                }
                let (a, b, resid) = fit(&pts);
                if verbose {
                    println!(
                        "    rows {y0}: {} columns, slope {a:+.4}, resid {resid:.2}",
                        pts.len()
                    );
                }
                (resid < 0.8).then_some((a, b))
            };
            if let (Some((al, bl)), Some((ar, br))) = (
                side(half - 8 - reach..half - 8),
                side(half + 8..half + 8 + reach),
            ) {
                if (al - ar).abs() < 0.01 {
                    let x = half as f64;
                    steps.push(((ar * x + br) - (al * x + bl), (al + ar) / 2.0));
                }
            }
            y0 += chunk / 2;
        }
        if steps.is_empty() {
            println!(
                "{xs:>7} {:>6}   (no straight horizontal edge spanning the seam)",
                0
            );
            continue;
        }
        let mut d: Vec<f64> = steps.iter().map(|t| t.0).collect();
        d.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let med = d[d.len() / 2];
        let sd = (d.iter().map(|x| (x - med) * (x - med)).sum::<f64>() / d.len() as f64).sqrt();
        let slope = steps.iter().map(|t| t.1).sum::<f64>() / steps.len() as f64;
        println!(
            "{xs:>7} {:>6} {med:>+8.2} {sd:>7.2}   slope {slope:+.4}",
            d.len()
        );
    }
    Ok(())
}
