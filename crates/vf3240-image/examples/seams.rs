//! Seam check: `seams <scan.tif> [x,x,...] [--rows y0:y1]`. For each candidate
//! column x, prints the mean |G(x+1) − G(x)| for columns x−6..x+6 over the row
//! band, so a physical gap between sensor chips shows as a spike at one column.
//! Default candidates: every 1728 sensor pixels (a sixth of a segment, one
//! chip) mapped to image columns for the file's dpi; `--all` scans every column.

use std::fs::File;
use std::io::BufReader;
use vf3240_core::unpack::Geometry;
use vf3240_image::tiff::Reader;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    let mut rows = None;
    if let Some(i) = args.iter().position(|a| a == "--rows") {
        rows = args.get(i + 1).and_then(|v| {
            v.split_once(':')
                .and_then(|(a, b)| Some((a.parse::<usize>().ok()?, b.parse::<usize>().ok()?)))
        });
        args.drain(i..i + 2);
    }
    let path = args
        .first()
        .cloned()
        .ok_or("usage: seams <scan.tif> [x,x,...] [--rows y0:y1]")?;
    let mut t = Reader::new(BufReader::with_capacity(8 << 20, File::open(&path)?))?;
    let (w, h) = (t.width, t.height);
    let dpi = t.dpi.unwrap_or(2400);
    let geom = Geometry::for_dpi(dpi).ok_or("dpi")?;
    if args.get(1).map(|a| a == "--all").unwrap_or(false) {
        // Whole-width spike scan: mean |G(x+1)-G(x)| per column over sampled rows,
        // reported where it exceeds 3x the median of the surrounding ±20 columns.
        let (y0, y1) = rows.unwrap_or((h / 4, 3 * h / 4));
        let mut sums = vec![0.0f64; w - 1];
        let mut n = 0u64;
        for s in y0 / t.rows_per_strip..=(y1 - 1) / t.rows_per_strip {
            let data = t.strip(s)?;
            for r in (0..t.rows_in_strip(s)).step_by(8) {
                let y = s * t.rows_per_strip + r;
                if y < y0 || y >= y1 {
                    continue;
                }
                let row = &data[r * w * 3..(r + 1) * w * 3];
                n += 1;
                for (x, slot) in sums.iter_mut().enumerate() {
                    *slot += (f64::from(row[3 * (x + 1) + 1]) - f64::from(row[3 * x + 1])).abs();
                }
            }
        }
        println!("{path}: whole-width spike scan over {n} rows ({y0}..{y1}):");
        let mut found = 0;
        for x in 20..w - 21 {
            let mut neigh: Vec<f64> = (x - 20..x + 21)
                .filter(|&k| (k as isize - x as isize).abs() > 2)
                .map(|k| sums[k])
                .collect();
            neigh.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let med = neigh[neigh.len() / 2];
            if sums[x] > 3.0 * med && med > 0.0 {
                println!(
                    "  spike between x={x} and {}: {:.2} vs local median {:.2} ({:.1}x)",
                    x + 1,
                    sums[x] / n as f64,
                    med / n as f64,
                    sums[x] / med
                );
                found += 1;
            }
        }
        println!("  {found} spikes");
        return Ok(());
    }
    let candidates: Vec<usize> = match args.get(1) {
        Some(list) => list
            .split(',')
            .map(|v| v.parse())
            .collect::<Result<_, _>>()?,
        None => {
            let chip = geom.seg_pixels / 6;
            (1..18)
                .map(|i| i * chip)
                .filter(|&x| x > geom.start_pixel_offset)
                .map(|x| x - geom.start_pixel_offset)
                .filter(|&x| x + 8 < w)
                .collect()
        }
    };
    let (y0, y1) = rows.unwrap_or((h / 4, 3 * h / 4));
    println!("{path}: {w}x{h} at {dpi} dpi; rows {y0}..{y1}; candidates {candidates:?}");
    const HALF: usize = 6;
    let mut sums: Vec<Vec<f64>> = candidates.iter().map(|_| vec![0.0; 2 * HALF + 1]).collect();
    let mut n = 0u64;
    let first = y0 / t.rows_per_strip;
    let last = (y1 - 1) / t.rows_per_strip;
    for s in first..=last {
        let data = t.strip(s)?;
        for r in (0..t.rows_in_strip(s)).step_by(4) {
            let y = s * t.rows_per_strip + r;
            if y < y0 || y >= y1 {
                continue;
            }
            let row = &data[r * w * 3..(r + 1) * w * 3];
            n += 1;
            for (ci, &cx) in candidates.iter().enumerate() {
                for (k, slot) in sums[ci].iter_mut().enumerate() {
                    let x = cx + k - HALF;
                    *slot += (f64::from(row[3 * (x + 1) + 1]) - f64::from(row[3 * x + 1])).abs();
                }
            }
        }
    }
    println!("mean |G(x+1)-G(x)| over {n} rows, for x = candidate-{HALF} .. candidate+{HALF}:");
    for (ci, &cx) in candidates.iter().enumerate() {
        let vals: Vec<String> = sums[ci]
            .iter()
            .map(|s| format!("{:5.2}", s / n as f64))
            .collect();
        println!("  x={cx:>6}: {}", vals.join(" "));
    }

    // Gap estimate from column correlation: the lens blur makes neighbouring
    // columns correlated, falling off with distance. Reference: corr(x, x+d)
    // averaged over columns 60..20 left of the seam; seam: the least
    // correlated adjacent pair within ±2 of the candidate. The distance d at
    // which the reference curve equals the seam value is the apparent pitch,
    // so the gap is d − 1 pixels.
    const DMAX: usize = 10;
    println!("\ngap estimate from adjacent-column correlation (reference curve corr(d) for d = 1..{DMAX}):");
    for &cx in &candidates {
        let x_lo = cx - 60;
        let x_hi = cx + 4;
        let cols = x_hi - x_lo;
        // Gather G columns for the band (every 4th row).
        let mut col: Vec<Vec<f64>> = vec![Vec::new(); cols];
        for s in first..=last {
            let data = t.strip(s)?;
            for r in (0..t.rows_in_strip(s)).step_by(4) {
                let y = s * t.rows_per_strip + r;
                if y < y0 || y >= y1 {
                    continue;
                }
                let row = &data[r * w * 3..(r + 1) * w * 3];
                for (k, c) in col.iter_mut().enumerate() {
                    c.push(f64::from(row[3 * (x_lo + k) + 1]));
                }
            }
        }
        let corr = |a: &[f64], b: &[f64]| -> f64 {
            let n = a.len() as f64;
            let (ma, mb) = (a.iter().sum::<f64>() / n, b.iter().sum::<f64>() / n);
            let (mut sab, mut saa, mut sbb) = (0.0, 0.0, 0.0);
            for (x, y) in a.iter().zip(b) {
                sab += (x - ma) * (y - mb);
                saa += (x - ma) * (x - ma);
                sbb += (y - mb) * (y - mb);
            }
            sab / (saa * sbb).sqrt()
        };
        let reference: Vec<f64> = (1..=DMAX)
            .map(|d| {
                let pairs: Vec<f64> = (0..40 - d).map(|k| corr(&col[k], &col[k + d])).collect();
                pairs.iter().sum::<f64>() / pairs.len() as f64
            })
            .collect();
        let (mut seam_x, mut seam_c) = (cx, 1.0f64);
        for x in cx - 2..=cx + 2 {
            let c = corr(&col[x - x_lo], &col[x + 1 - x_lo]);
            if c < seam_c {
                seam_c = c;
                seam_x = x;
            }
        }
        // Interpolate the apparent distance on the (decreasing) reference curve.
        let mut d_est = DMAX as f64;
        for d in 1..DMAX {
            let (c0, c1) = (reference[d - 1], reference[d]);
            if seam_c <= c0 && seam_c >= c1 && c0 != c1 {
                d_est = d as f64 + (c0 - seam_c) / (c0 - c1);
                break;
            }
        }
        let refs: Vec<String> = reference.iter().map(|c| format!("{c:.3}")).collect();
        println!(
            "  seam between x={seam_x} and {}: corr {seam_c:.3}; reference {}; apparent pitch {d_est:.1} px → gap ≈ {:.1} px",
            seam_x + 1,
            refs.join(" "),
            d_est - 1.0
        );
    }
    Ok(())
}
