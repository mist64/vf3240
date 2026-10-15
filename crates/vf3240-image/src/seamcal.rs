//! Seam calibration: gap and vertical stagger at every chip junction from
//! a scan of a dot screen (the printed `target`, or any halftone page).
//!
//! Per seam and row band, the band's 2D power spectrum (both sides of the
//! seam together) gives the lattice vectors of the screen. For up to four
//! of them the complex amplitude is measured in sliding 32-column
//! sub-windows on each side with one (x, y) reference; straight lines
//! through the unwrapped phases meet at the seam with a jump of
//! 2π·(fx·gap + fy·step), and the jumps of all vectors are solved jointly
//! by least squares (trying one phase wrap either way). The per-band
//! results are reduced to the median. Validated on `examples/synth_halftone`
//! (known gaps 3/4/2 px and one 1-row step: 3.04/3.99/2.00, 0.2/−0.1/1.0)
//! and against the scanner (`docs/hardware/README.md`, 2026-10-14).

use crate::seams::{self, SeamTable, SEAMS};
use crate::tiff::Reader;
use std::f64::consts::PI;
use std::fs::File;
use std::io::BufReader;

/// Columns kept on each side of a seam.
pub const REACH: usize = 240;
/// Default row band and spacing between bands.
pub const BAND: usize = 320;
pub const STEP: usize = 400;

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

/// In-place iterative radix-2 FFT of `re`/`im` (length a power of two).
fn fft(re: &mut [f64], im: &mut [f64]) {
    let n = re.len();
    let mut j = 0;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j |= bit;
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }
    let mut len = 2;
    while len <= n {
        let ang = -2.0 * PI / len as f64;
        for start in (0..n).step_by(len) {
            for k in 0..len / 2 {
                let (wr, wi) = ((ang * k as f64).cos(), (ang * k as f64).sin());
                let (ur, ui) = (re[start + k], im[start + k]);
                let (xr, xi) = (re[start + k + len / 2], im[start + k + len / 2]);
                let (vr, vi) = (xr * wr - xi * wi, xr * wi + xi * wr);
                re[start + k] = ur + vr;
                im[start + k] = ui + vi;
                re[start + k + len / 2] = ur - vr;
                im[start + k + len / 2] = ui - vi;
            }
        }
        len <<= 1;
    }
}

/// 2D power spectrum of a `w`×`h` block (row-major), zero-padded to `nx`×`ny`.
fn power2d(block: &[f64], w: usize, h: usize, nx: usize, ny: usize) -> Vec<f64> {
    let mut re = vec![0.0; nx * ny];
    let mut im = vec![0.0; nx * ny];
    for y in 0..h {
        // Hann window in both directions.
        let wy = 0.5 - 0.5 * (2.0 * PI * y as f64 / h as f64).cos();
        for x in 0..w {
            let wx = 0.5 - 0.5 * (2.0 * PI * x as f64 / w as f64).cos();
            re[y * nx + x] = block[y * w + x] * wx * wy;
        }
    }
    for y in 0..ny {
        fft(&mut re[y * nx..(y + 1) * nx], &mut im[y * nx..(y + 1) * nx]);
    }
    let (mut cr, mut ci) = (vec![0.0; ny], vec![0.0; ny]);
    for x in 0..nx {
        for y in 0..ny {
            cr[y] = re[y * nx + x];
            ci[y] = im[y * nx + x];
        }
        fft(&mut cr, &mut ci);
        for y in 0..ny {
            re[y * nx + x] = cr[y] * cr[y] + ci[y] * ci[y];
        }
    }
    re
}

/// Fit one band: `c` holds the 2·REACH columns (G channel, f64) of the
/// band around a seam. Returns `(gap, vertical step, rms residual in cycles)`.
pub fn fit_band(
    c: &[Vec<f64>],
    band: usize,
    verbose: &mut dyn FnMut(&str),
) -> Option<(f64, f64, f64)> {
    let n = 2 * REACH;
    // High-pass every row with a 61-px running mean.
    let rows: Vec<Vec<f64>> = (0..band)
        .map(|y| {
            let r: Vec<f64> = (0..n).map(|k| c[k][y]).collect();
            let half = 30usize;
            (0..n)
                .map(|x| {
                    let (a, b) = (x.saturating_sub(half), (x + half + 1).min(n));
                    r[x] - r[a..b].iter().sum::<f64>() / (b - a) as f64
                })
                .collect()
        })
        .collect();
    let left = 20..REACH - 6;
    let right = REACH + 6..n - 20;
    let w = left.len();
    let (nx, ny) = (256usize, band.next_power_of_two());
    let block = |xs: &std::ops::Range<usize>| -> Vec<f64> {
        rows.iter()
            .flat_map(|r| r[xs.clone()].iter().copied())
            .collect()
    };
    let pl = power2d(&block(&left), w, band, nx, ny);
    let pr = power2d(&block(&right), w, band, nx, ny);
    // Candidate lattice vectors: local maxima of the summed spectrum in the
    // half-plane fy >= 0 (fx > 0 on the fy = 0 line), periods 8..100 px.
    let fx_of = |x: usize| {
        if x < nx / 2 {
            x as f64 / nx as f64
        } else {
            x as f64 / nx as f64 - 1.0
        }
    };
    let mut peaks: Vec<(f64, f64, f64)> = Vec::new(); // (power, fx, fy)
    for y in 0..ny / 2 {
        let fy = y as f64 / ny as f64;
        for x in 0..nx {
            let fx = fx_of(x);
            if y == 0 && fx <= 0.0 {
                continue;
            }
            let f = (fx * fx + fy * fy).sqrt();
            if !(1.0 / 100.0..=1.0 / 8.0).contains(&f) {
                continue;
            }
            let p = pl[y * nx + x] + pr[y * nx + x];
            let idx = |dx: isize, dy: isize| {
                let xx = (x as isize + dx).rem_euclid(nx as isize) as usize;
                let yy = (y as isize + dy).rem_euclid(ny as isize) as usize;
                pl[yy * nx + xx] + pr[yy * nx + xx]
            };
            if (-1..=1).all(|dy| (-1..=1).all(|dx| (dx == 0 && dy == 0) || idx(dx, dy) <= p)) {
                peaks.push((p, fx, fy));
            }
        }
    }
    peaks.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
    let &(top, _, _) = peaks.first()?;
    // Peaks must stand out from the spectrum mean.
    let mean = (pl.iter().sum::<f64>() + pr.iter().sum::<f64>()) / (nx * ny) as f64;
    let share = top / mean;
    if share < 20.0 {
        verbose(&format!("band: peak share {share:.0} rejected"));
        return None;
    }
    // Phase jump at the seam for one lattice vector: complex amplitude of
    // (fx0, fy0) in 32-column sub-windows every 8 columns on each side with
    // an absolute (x, y) reference, straight lines through the unwrapped
    // phases, evaluated at the seam. Returns (jump, fx, fy, max residual).
    // Sub-windows of 96 columns (2.4 periods of the printed target, 3.4 of a
    // 28 px screen) with a Hann weight: a short rectangular window leaks the
    // dots' harmonics into the fundamental with a phase bias that depends on
    // where the seam cuts the lattice (seen as placement-dependent gaps).
    let sub = 96usize;
    let jump_for = |fx0: f64, fy0: f64| -> Option<(f64, f64, f64, f64)> {
        let amp = |x0: usize| -> (f64, f64) {
            let (mut re, mut im) = (0.0, 0.0);
            for (y, r) in rows.iter().enumerate() {
                for (k, (x, v)) in r.iter().enumerate().skip(x0).take(sub).enumerate() {
                    let wgt = 0.5 - 0.5 * (2.0 * PI * (k as f64 + 0.5) / sub as f64).cos();
                    let a = 2.0 * PI * (fx0 * x as f64 + fy0 * y as f64);
                    re += wgt * v * a.cos();
                    im -= wgt * v * a.sin();
                }
            }
            (im.atan2(re), (re * re + im * im).sqrt())
        };
        let side = |xs: std::ops::Range<usize>| -> (Vec<f64>, Vec<f64>, Vec<f64>) {
            let mut xc = Vec::new();
            let mut ph = Vec::new();
            let mut am = Vec::new();
            let mut x0 = xs.start;
            while x0 + sub <= xs.end {
                let (p, a) = amp(x0);
                xc.push(x0 as f64 + sub as f64 / 2.0);
                ph.push(p);
                am.push(a);
                x0 += 8;
            }
            (xc, unwrap(&ph), am)
        };
        let (lx, lph, lam) = side(left.clone());
        let (rx, rph, ram) = side(right.clone());
        let amax = lam.iter().chain(&ram).copied().fold(0.0, f64::max);
        let amin = lam.iter().chain(&ram).copied().fold(f64::MAX, f64::min);
        if amin < 0.25 * amax {
            return None;
        }
        let (al, bl) = linfit(&lx, &lph);
        let (ar, br) = linfit(&rx, &rph);
        let resid = lx
            .iter()
            .zip(&lph)
            .map(|(x, p)| (p - (al * x + bl)).abs())
            .chain(rx.iter().zip(&rph).map(|(x, p)| (p - (ar * x + br)).abs()))
            .fold(0.0, f64::max);
        let fxl = fx0 + al / (2.0 * PI);
        let fxr = fx0 + ar / (2.0 * PI);
        if resid > 0.6 || (fxl - fxr).abs() > 0.002 {
            return None;
        }
        let seam = REACH as f64;
        let mut jump = (ar * seam + br) - (al * seam + bl);
        jump = (jump + PI).rem_euclid(2.0 * PI) - PI;
        Some((jump, (fxl + fxr) / 2.0, fy0, resid))
    };
    // Up to four strong lattice vectors; solve jump_i = 2π(fx_i·gap + fy_i·dy)
    // by least squares, trying one wrap either way per vector.
    let vecs: Vec<(f64, f64, f64, f64)> = peaks
        .iter()
        .take(6)
        .filter(|p| p.0 >= 0.1 * top)
        .filter_map(|&(_, fx, fy)| jump_for(fx, fy))
        .take(4)
        .collect();
    if vecs.is_empty() {
        verbose("band: no usable lattice vector");
        return None;
    }
    let best_fx = vecs.iter().map(|v| v.1.abs()).fold(0.0, f64::max);
    if vecs.len() < 2 {
        // One vector: gap assuming no vertical offset.
        let v = vecs[0];
        if v.1.abs() < 1.0 / 60.0 {
            return None;
        }
        let gap = v.0 / (2.0 * PI * v.1);
        verbose(&format!(
            "band: share {share:.0} one vector fx {:.4} fy {:.4} gap {gap:.2} (dy assumed 0)",
            v.1, v.2
        ));
        return Some((gap, 0.0, 1.0 / best_fx));
    }
    let nv = vecs.len();
    let mut best: Option<(f64, f64, f64)> = None; // (rms, gap, dy)
    for combo in 0..3usize.pow(nv as u32) {
        let mut c = combo;
        let (mut sxx, mut sxy, mut syy, mut bx, mut by) = (0.0, 0.0, 0.0, 0.0, 0.0);
        let mut js = Vec::with_capacity(nv);
        for v in &vecs {
            let k = (c % 3) as f64 - 1.0;
            c /= 3;
            let j = (v.0 + 2.0 * PI * k) / (2.0 * PI); // cycles
            js.push(j);
            sxx += v.1 * v.1;
            sxy += v.1 * v.2;
            syy += v.2 * v.2;
            bx += v.1 * j;
            by += v.2 * j;
        }
        let det = sxx * syy - sxy * sxy;
        if det.abs() < 1e-12 {
            continue;
        }
        let gap = (syy * bx - sxy * by) / det;
        let dy = (sxx * by - sxy * bx) / det;
        if gap.abs() > 12.0 || dy.abs() > 6.0 {
            continue;
        }
        let rms = (vecs
            .iter()
            .zip(&js)
            .map(|(v, j)| (j - (v.1 * gap + v.2 * dy)).powi(2))
            .sum::<f64>()
            / nv as f64)
            .sqrt();
        if best.map_or(true, |b| rms < b.0) {
            best = Some((rms, gap, dy));
        }
    }
    let (rms, gap, dy) = best?;
    let desc: Vec<String> = vecs
        .iter()
        .map(|v| format!("({:.4},{:.4}) j {:.2}", v.1, v.2, v.0))
        .collect();
    verbose(&format!(
        "band: share {share:.0} {nv} vectors {} -> gap {gap:.2} dy {dy:.2} rms {rms:.3} cyc",
        desc.join(" ")
    ));
    if rms > 0.05 {
        return None;
    }
    Some((gap, dy, 1.0 / best_fx))
}

/// One seam's result over all usable bands.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SeamMeasure {
    /// Median gap in pixels at the scan's dpi.
    pub gap: f64,
    /// Median vertical step in rows at the scan's dpi.
    pub dy: f64,
    pub bands: usize,
    /// Spread (rms about the median) of the per-band gaps.
    pub spread: f64,
}

/// The G channel of the `2·REACH` columns around each seam, every row:
/// all the measurement needs, ~7.7 KB per row per seam.
pub struct SeamColumns {
    pub cols: Vec<usize>,
    pub height: usize,
    /// Per seam, `height × 2·REACH` bytes.
    data: Vec<Vec<u8>>,
}

impl SeamColumns {
    /// Keep the seams that have `REACH` columns on both sides in a row of `width`.
    pub fn new(seam_cols: &[usize], width: usize) -> Self {
        let cols: Vec<usize> = seam_cols
            .iter()
            .copied()
            .filter(|&x| x >= REACH && x + REACH < width)
            .collect();
        let data = cols.iter().map(|_| Vec::new()).collect();
        SeamColumns {
            cols,
            height: 0,
            data,
        }
    }

    /// Add one interleaved 8-bit RGB row.
    pub fn push_row(&mut self, row: &[u8]) {
        for (si, &xs) in self.cols.iter().enumerate() {
            let d = &mut self.data[si];
            d.extend((0..2 * REACH).map(|k| row[3 * (xs - REACH + k) + 1]));
        }
        self.height += 1;
    }

    /// Load from an 8-bit RGB TIFF.
    pub fn from_file(path: &str, seam_cols: &[usize]) -> Result<Self, Box<dyn std::error::Error>> {
        let mut t = Reader::new(BufReader::with_capacity(8 << 20, File::open(path)?))?;
        if t.bits != 8 || t.channels != 3 {
            return Err("seam measurement needs an 8-bit RGB scan".into());
        }
        let w = t.width;
        let mut sc = SeamColumns::new(seam_cols, w);
        for s in 0..t.strips() {
            let data = t.strip(s)?;
            for r in 0..t.rows_in_strip(s) {
                sc.push_row(&data[r * w * 3..(r + 1) * w * 3]);
            }
        }
        Ok(sc)
    }

    /// Measure every seam: median gap and step over the usable bands.
    pub fn measure(
        &self,
        band: usize,
        step: usize,
        verbose: bool,
        log: &mut dyn FnMut(&str),
    ) -> Vec<Option<SeamMeasure>> {
        let n = 2 * REACH;
        let bands: Vec<usize> = (1200..self.height.saturating_sub(band + 1200))
            .step_by(step)
            .collect();
        let mut quiet = |_: &str| {};
        self.cols
            .iter()
            .enumerate()
            .map(|(si, &xs)| {
                let d = &self.data[si];
                let mut results: Vec<(f64, f64)> = Vec::new();
                for &y0 in &bands {
                    let cols: Vec<Vec<f64>> = (0..n)
                        .map(|k| (y0..y0 + band).map(|y| f64::from(d[y * n + k])).collect())
                        .collect();
                    let mut lg = |m: &str| log(&format!("  seam {xs} rows {y0}: {m}"));
                    let r = if verbose {
                        fit_band(&cols, band, &mut lg)
                    } else {
                        fit_band(&cols, band, &mut quiet)
                    };
                    if let Some((gap, dy, _)) = r {
                        results.push((gap, dy));
                    }
                }
                if results.is_empty() {
                    return None;
                }
                let mut g: Vec<f64> = results.iter().map(|t| t.0).collect();
                let mut dy: Vec<f64> = results.iter().map(|t| t.1).collect();
                g.sort_by(|a, b| a.partial_cmp(b).unwrap());
                dy.sort_by(|a, b| a.partial_cmp(b).unwrap());
                let gap = g[g.len() / 2];
                let spread =
                    (g.iter().map(|x| (x - gap) * (x - gap)).sum::<f64>() / g.len() as f64).sqrt();
                Some(SeamMeasure {
                    gap,
                    dy: dy[dy.len() / 2],
                    bands: results.len(),
                    spread,
                })
            })
            .collect()
    }
}

/// Measure the seams at image columns `seam_cols` of an 8-bit RGB TIFF.
pub fn measure_file(
    path: &str,
    seam_cols: &[usize],
    band: usize,
    step: usize,
    verbose: bool,
    log: &mut dyn FnMut(&str),
) -> Result<Vec<Option<SeamMeasure>>, Box<dyn std::error::Error>> {
    let sc = SeamColumns::from_file(path, seam_cols)?;
    let m = sc.measure(band, step, verbose, log);
    // Back onto the caller's list (seams too close to an edge are None).
    Ok(seam_cols
        .iter()
        .map(|c| sc.cols.iter().position(|x| x == c).and_then(|i| m[i]))
        .collect())
}

/// The 16 nominal seam columns of a full-width 2400 dpi scan.
pub fn nominal_cols(width: usize) -> Result<Vec<usize>, Box<dyn std::error::Error>> {
    let cols: Vec<usize> = seams::seam_columns(2400, width, 0)
        .ok_or("geometry")?
        .into_iter()
        .map(|(c, _)| c)
        .collect();
    if cols.len() != SEAMS {
        return Err(format!(
            "expected {SEAMS} seams in a full-width 2400 dpi scan, found {}",
            cols.len()
        )
        .into());
    }
    Ok(cols)
}

/// A measurement worth believing: enough bands agreeing on a gap and step
/// in the range the hardware can have (`MIN_BANDS`, 0–10 px, ±3 rows,
/// spread under 1.5 px).
pub const MIN_BANDS: usize = 3;
fn plausible(r: &SeamMeasure) -> bool {
    r.bands >= MIN_BANDS && (0.0..=10.0).contains(&r.gap) && r.dy.abs() <= 3.0 && r.spread < 1.5
}

/// Merge one or more passes into `base`: per seam, the median over the
/// plausible measurements (seams without one keep their old value), with
/// a report line per seam giving the number of passes, the median gap and
/// step, and their ranges across the passes. Scanning the target in a few
/// placements and taking the median matters: the apparent gap at a junction
/// moves by up to ±0.5 px with how the sheet lies on the glass.
pub fn merge(
    base: &SeamTable,
    cols: &[usize],
    passes: &[Vec<Option<SeamMeasure>>],
) -> (SeamTable, Vec<String>) {
    let mut table = *base;
    let mut report = vec![format!(
        "{:>4} {:>7} {:>6} {:>7} {:>6} {:>7} {:>7}",
        "seam", "column", "passes", "gap", "step", "gap±", "step±"
    )];
    for (k, &col) in cols.iter().enumerate().take(SEAMS) {
        let good: Vec<&SeamMeasure> = passes
            .iter()
            .filter_map(|m| m.get(k).and_then(|r| r.as_ref()))
            .filter(|r| plausible(r))
            .collect();
        if good.is_empty() {
            report.push(format!(
                "{:>4} {:>7} {:>6}   no usable measurement, keeping {:.2}/{:+.2}",
                k + 1,
                col,
                0,
                base.gaps[k],
                base.dy[k]
            ));
            continue;
        }
        let mut g: Vec<f64> = good.iter().map(|r| r.gap).collect();
        let mut d: Vec<f64> = good.iter().map(|r| r.dy).collect();
        g.sort_by(|a, b| a.partial_cmp(b).unwrap());
        d.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let (gap, dy) = (median(&g), median(&d));
        table.gaps[k] = (gap * 100.0).round() / 100.0;
        table.dy[k] = (dy * 20.0).round() / 20.0;
        report.push(format!(
            "{:>4} {:>7} {:>6} {:>7.2} {:>+6.2} {:>7.2} {:>7.2}",
            k + 1,
            col,
            good.len(),
            gap,
            dy,
            (g[g.len() - 1] - g[0]) / 2.0,
            (d[d.len() - 1] - d[0]) / 2.0
        ));
    }
    (table, report)
}

fn median(sorted: &[f64]) -> f64 {
    let n = sorted.len();
    if n % 2 == 1 {
        sorted[n / 2]
    } else {
        (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0
    }
}

/// Measure one or more full-width 2400 dpi scans made with the seam fill
/// and the stagger correction off, and merge them into `base`.
pub fn calibrate_files(
    paths: &[String],
    base: &SeamTable,
    verbose: bool,
    log: &mut dyn FnMut(&str),
) -> Result<(SeamTable, Vec<String>), Box<dyn std::error::Error>> {
    let mut passes = Vec::new();
    let mut cols = None;
    for path in paths {
        let t = Reader::new(BufReader::with_capacity(1 << 20, File::open(path)?))?;
        if t.dpi != Some(2400) {
            return Err(format!("{path}: seam calibration needs a 2400 dpi scan").into());
        }
        let c = nominal_cols(t.width)?;
        if let Some(prev) = &cols {
            if *prev != c {
                return Err("the scans have different widths".into());
            }
        }
        passes.push(measure_file(path, &c, BAND, STEP, verbose, log)?);
        cols = Some(c);
    }
    let cols = cols.ok_or("no scan given")?;
    Ok(merge(base, &cols, &passes))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn merge_keeps_old_values_for_missing_and_implausible_seams() {
        let base = SeamTable::DEVELOPMENT_UNIT;
        let mut m = vec![None; SEAMS];
        m[0] = Some(SeamMeasure {
            gap: 4.0,
            dy: 0.5,
            bands: 10,
            spread: 0.3,
        });
        m[1] = Some(SeamMeasure {
            gap: -9.98,
            dy: 0.0,
            bands: 1,
            spread: 0.0,
        });
        m[2] = Some(SeamMeasure {
            gap: 4.0,
            dy: 0.0,
            bands: 2,
            spread: 0.1,
        });
        let cols: Vec<usize> = (0..SEAMS).map(|k| 1417 + 1728 * k).collect();
        let (t, report) = merge(&base, &cols, &[m]);
        assert_eq!((t.gaps[0], t.dy[0]), (4.0, 0.5));
        assert_eq!((t.gaps[1], t.dy[1]), (base.gaps[1], base.dy[1]));
        assert_eq!((t.gaps[2], t.dy[2]), (base.gaps[2], base.dy[2]));
        assert_eq!(report.len(), SEAMS + 1);
        assert!(report[2].contains("no usable measurement"));
        assert!(report[4].contains("no usable measurement"));
    }
}
