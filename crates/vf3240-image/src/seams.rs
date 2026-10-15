//! Fill the pixels lost at the CIS chip junctions.
//!
//! The sensor is eighteen chips of 1728 pixels (at 2400 dpi; a sixth of a
//! segment at any resolution). A few pixels are missing at each junction, so
//! everything right of a seam is drawn too far left and lines kink there
//! (`docs/hardware/README.md`, 2026-10-13: spikes at image columns 1416 + 1728k,
//! gaps measured from the halftone lattice, 2D fit). This stage maps every output
//! column to its physical position, interpolates inside a chip with a cubic
//! through the four nearest samples, and bridges a gap linearly between the
//! last pixel of one chip and the first of the next. The output is wider by
//! the sum of the gaps, so the image matches the paper at the nominal dpi.

use vf3240_core::unpack::Geometry;

/// Number of chip junctions inside the image (18 chips of 1728 px, 31104
/// in all; the image is 28751 px from sensor pixel 2040 on, so the first
/// chip lies outside it and the second is partly cropped: 17 chips, 16
/// junctions).
pub const SEAMS: usize = 16;
/// Gap in pixels at 2400 dpi after each chip junction, in image order
/// (seams at image x = 2·chip − start_pixel_offset + k·chip). Measured on
/// this scanner from halftone lattice phase; see the hardware log.
pub const GAPS_2400: [f64; SEAMS] = [
    // Development unit: `--calibrate-seams` over three placements of the
    // printed target (2026-10-15; 45 bands per seam per placement, one
    // placement turned by 180°), per-seam median. The apparent gap moves
    // by up to ±0.5 px (±0.9 at one seam) with how the sheet lies, so a
    // single placement is not enough.
    4.08, 3.76, 3.72, 4.19, 5.37, 4.33, 4.12, 4.91, 3.80, 3.28, 4.51, 4.11, 3.92, 4.48, 4.22, 5.02,
];

/// Vertical stagger of the chips: rows at 2400 dpi by which the chip right
/// of each seam sees the paper later (lower on the page) than the chip left
/// of it. Same measurement as `GAPS_2400`. Stable along each seam to
/// ±0.1 row and halved at 1200 dpi, so it is a mechanical offset of up to
/// 14 µm between neighbouring chips, adding up to 12.5 rows across the bar.
pub const DY_2400: [f64; SEAMS] = [
    0.00, 1.20, 0.95, 0.60, 1.30, 0.55, 0.80, 0.40, 1.25, 0.55, 1.10, 0.00, 1.10, 0.95, 0.95, 0.80,
];

/// Per-unit seam table: the gap and the vertical stagger at each of the
/// 16 junctions, both at 2400 dpi (scaled for other resolutions).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SeamTable {
    pub gaps: [f64; SEAMS],
    pub dy: [f64; SEAMS],
}

impl SeamTable {
    /// Used when no table is given: a round 4 px at every seam, no
    /// stagger. Roughly right for any unit (measured gaps run 3.7–5.5 px),
    /// and never adds a vertical step the unit does not have.
    pub const DEFAULT: SeamTable = SeamTable {
        gaps: [4.0; SEAMS],
        dy: [0.0; SEAMS],
    };

    /// The unit this driver was developed on (`GAPS_2400`, `DY_2400`).
    pub const DEVELOPMENT_UNIT: SeamTable = SeamTable {
        gaps: GAPS_2400,
        dy: DY_2400,
    };

    /// Compact command-line form: 16 comma-separated `gap/step` items,
    /// a bare `gap` meaning step 0, e.g. `4.46/-0.2,3.76/0.8,…`.
    pub fn parse(text: &str) -> Result<Self, String> {
        let mut t = SeamTable {
            gaps: [0.0; SEAMS],
            dy: [0.0; SEAMS],
        };
        let items: Vec<&str> = text.split(',').map(str::trim).collect();
        if items.len() != SEAMS {
            return Err(format!("expected {SEAMS} seams, got {}", items.len()));
        }
        for (k, item) in items.iter().enumerate() {
            let (g, d) = item.split_once('/').unwrap_or((item, "0"));
            t.gaps[k] = g
                .parse()
                .map_err(|_| format!("seam {}: bad gap {g:?}", k + 1))?;
            t.dy[k] = d
                .parse()
                .map_err(|_| format!("seam {}: bad step {d:?}", k + 1))?;
            if !(0.0..=12.0).contains(&t.gaps[k]) || t.dy[k].abs() > 4.0 {
                return Err(format!("seam {}: {item:?} out of range", k + 1));
            }
        }
        Ok(t)
    }

    /// The form `parse` reads.
    pub fn to_arg(&self) -> String {
        (0..SEAMS)
            .map(|k| {
                if self.dy[k] == 0.0 {
                    format!("{:.2}", self.gaps[k])
                } else {
                    format!("{:.2}/{:.2}", self.gaps[k], self.dy[k])
                }
            })
            .collect::<Vec<_>>()
            .join(",")
    }
}

/// Chip junctions inside a row of `in_width` columns starting at full-line
/// column `x_origin`: `(column, seam index)`, ascending. Within the first
/// image segment (CIS segment 1) the junction sits one pixel later than the
/// formula at 2400 dpi (measured: spikes between 1416|1417, 3144|3145,
/// 4872|4873, 6600|6601; from 8327|8328 on they match).
pub fn seam_columns(dpi: u32, in_width: usize, x_origin: usize) -> Option<Vec<(usize, usize)>> {
    let geom = Geometry::for_dpi(dpi)?;
    let chip = geom.seg_pixels / 6;
    let seg_end = geom.seg_pixels - geom.start_pixel_offset;
    let nudge = (dpi as f64 / 2400.0).round() as usize;
    let mut seams: Vec<(usize, usize)> = (1..18)
        .map(|k| k * chip)
        .filter(|&s| s > geom.start_pixel_offset)
        .map(|s| s - geom.start_pixel_offset)
        .map(|s| if s < seg_end { s + nudge } else { s })
        .enumerate()
        .map(|(k, s)| (s, k.min(SEAMS - 1)))
        .filter(|&(s, _)| s >= x_origin && s - x_origin < in_width)
        .map(|(s, k)| (s - x_origin, k))
        .collect();
    seams.sort_by_key(|s| s.0);
    Some(seams)
}

/// Column ranges of the chips in such a row with the vertical offset, in
/// rows at `dpi`, of each (`dy_2400` accumulated across the seams and
/// centred on zero so the image as a whole does not move).
pub fn chip_offsets(
    dpi: u32,
    in_width: usize,
    x_origin: usize,
    dy_2400: &[f64; SEAMS],
) -> Option<Vec<(usize, usize, f64)>> {
    let seams = seam_columns(dpi, in_width, x_origin)?;
    let scale = dpi as f64 / 2400.0;
    // Offset of the leftmost chip in the row relative to chip 0 of the sensor.
    let first_k = seams.first().map_or(SEAMS, |s| s.1);
    let mut acc: f64 = dy_2400[..first_k].iter().sum::<f64>() * scale;
    let mut chips = Vec::with_capacity(seams.len() + 1);
    let mut start = 0;
    for &(col, k) in &seams {
        chips.push((start, col, acc));
        acc += dy_2400[k] * scale;
        start = col;
    }
    chips.push((start, in_width, acc));
    let mean = chips.iter().map(|c| c.2).sum::<f64>() / chips.len() as f64;
    for c in &mut chips {
        c.2 -= mean;
    }
    Some(chips)
}

pub struct Deseam {
    in_width: usize,
    max_value: f64,
    out_width: usize,
    /// For each output column: (source index i, fraction t in [0,1), gap bridge?)
    plan: Vec<(usize, f64, bool)>,
}

impl Deseam {
    /// `gaps` are at 2400 dpi; they scale with the resolution. `x_origin` is
    /// the full-line column the input row starts at (a cropped scan area).
    pub fn new(
        dpi: u32,
        in_width: usize,
        gaps_2400: &[f64; SEAMS],
        x_origin: usize,
        max_value: u16,
    ) -> Option<Self> {
        let scale = dpi as f64 / 2400.0;
        let seams: Vec<(usize, f64)> = seam_columns(dpi, in_width, x_origin)?
            .into_iter()
            .map(|(s, k)| (s, gaps_2400[k] * scale))
            .collect();
        let phys = |x: usize| -> f64 {
            x as f64
                + seams
                    .iter()
                    .filter(|(s, _)| *s <= x)
                    .map(|(_, g)| g)
                    .sum::<f64>()
        };
        let total: f64 = seams.iter().map(|(_, g)| g).sum();
        let out_width = (in_width as f64 + total).floor() as usize;
        let mut plan = Vec::with_capacity(out_width);
        let mut i = 0usize; // source index with phys(i) <= X
        for xo in 0..out_width {
            let target = xo as f64;
            while i + 1 < in_width && phys(i + 1) <= target {
                i += 1;
            }
            let (p0, p1) = (phys(i), phys(i.min(in_width - 2) + 1));
            let span = p1 - p0;
            let t = if span > 0.0 {
                ((target - p0) / span).clamp(0.0, 1.0)
            } else {
                0.0
            };
            plan.push((i.min(in_width - 2), t, span > 1.5));
        }
        Some(Deseam {
            in_width,
            max_value: f64::from(max_value),
            out_width,
            plan,
        })
    }

    pub fn out_width(&self) -> usize {
        self.out_width
    }

    /// Resample one interleaved RGB row of `in_width` pixels into `out`.
    pub fn apply(&self, row: &[u16], out: &mut Vec<u16>) {
        debug_assert_eq!(row.len(), self.in_width * 3);
        out.clear();
        out.reserve(self.out_width * 3);
        let n = self.in_width;
        for &(i, t, bridge) in &self.plan {
            for c in 0..3 {
                let v = |k: isize| f64::from(row[3 * (k.clamp(0, n as isize - 1) as usize) + c]);
                let (v1, v2) = (v(i as isize), v(i as isize + 1));
                let val = if bridge || t == 0.0 {
                    v1 + t * (v2 - v1)
                } else {
                    // Catmull-Rom through the four nearest samples.
                    let (v0, v3) = (v(i as isize - 1), v(i as isize + 2));
                    let (t2, t3) = (t * t, t * t * t);
                    0.5 * ((2.0 * v1)
                        + (-v0 + v2) * t
                        + (2.0 * v0 - 5.0 * v1 + 4.0 * v2 - v3) * t2
                        + (-v0 + 3.0 * v1 - 3.0 * v2 + v3) * t3)
                };
                out.push(val.round().clamp(0.0, self.max_value) as u16);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn output_is_wider_by_the_gap_sum_and_pixels_keep_their_values() {
        let w = 28751;
        let d = Deseam::new(2400, w, &GAPS_2400, 0, 255).unwrap();
        let total: f64 = GAPS_2400.iter().sum();
        assert_eq!(d.out_width(), (w as f64 + total).floor() as usize);
        // A row that is a linear ramp in x maps to a ramp with the same slope
        // in physical position, so a column well inside the first chip is unchanged...
        let row: Vec<u16> = (0..w).flat_map(|x| [(x / 120) as u16; 3]).collect();
        let mut out = Vec::new();
        d.apply(&row, &mut out);
        assert_eq!(out.len(), d.out_width() * 3);
        assert_eq!(&out[..3 * 1000], &row[..3 * 1000]);
        // ...and a column right of the first seam (at 1417) appears shifted right by that gap.
        let x = 3000;
        let shifted = x + GAPS_2400[0].round() as usize;
        assert!(
            (i32::from(out[3 * shifted]) - i32::from(row[3 * x])).abs() <= 1,
            "{} vs {}",
            out[3 * shifted],
            row[3 * x]
        );
    }

    #[test]
    fn gaps_scale_with_the_resolution() {
        let d = Deseam::new(600, 7187, &GAPS_2400, 0, 255).unwrap();
        let total: f64 = GAPS_2400.iter().sum::<f64>() / 4.0;
        assert_eq!(d.out_width(), (7187.0 + total).floor() as usize);
    }

    #[test]
    fn seam_table_round_trips_through_its_compact_form() {
        let t = SeamTable::DEVELOPMENT_UNIT;
        let back = SeamTable::parse(&t.to_arg()).unwrap();
        for k in 0..SEAMS {
            assert!((back.gaps[k] - t.gaps[k]).abs() < 0.005);
            assert!((back.dy[k] - t.dy[k]).abs() < 0.005);
        }
        let bare = SeamTable::parse("4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4").unwrap();
        assert_eq!(bare.dy, [0.0; SEAMS]);
        assert!(SeamTable::parse("4,4").is_err());
        assert!(SeamTable::parse("4/x,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4").is_err());
        assert!(SeamTable::parse("40,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4").is_err());
    }

    #[test]
    fn chip_offsets_accumulate_the_steps_and_are_centred() {
        let chips = chip_offsets(2400, 28751, 0, &DY_2400).unwrap();
        assert_eq!(chips.len(), 17);
        assert_eq!(chips[0].0, 0);
        assert_eq!(chips[0].1, 1417);
        assert_eq!(chips[16].1, 28751);
        for w in chips.windows(2) {
            assert_eq!(w[0].1, w[1].0);
        }
        let steps: Vec<f64> = chips.windows(2).map(|w| w[1].2 - w[0].2).collect();
        for (st, dy) in steps.iter().zip(&DY_2400) {
            assert!((st - dy).abs() < 1e-9);
        }
        let mean: f64 = chips.iter().map(|c| c.2).sum::<f64>() / 17.0;
        assert!(mean.abs() < 1e-9);
        // 1200 dpi: half the rows; a crop keeps the absolute stagger of its chips.
        let half = chip_offsets(1200, 14375, 0, &DY_2400).unwrap();
        assert!((half[1].2 - half[0].2 - DY_2400[0] / 2.0).abs() < 1e-9);
        let crop = chip_offsets(2400, 5000, 4000, &DY_2400).unwrap();
        assert_eq!(crop.len(), 4);
        assert!((crop[1].2 - crop[0].2 - DY_2400[2]).abs() < 1e-9);
    }

    #[test]
    fn a_cropped_area_only_fills_the_seams_inside_it() {
        // 2400 dpi, crop columns 4000..9000: seams 4873, 6601 and 8328 lie inside.
        let d = Deseam::new(2400, 5000, &GAPS_2400, 4000, 255).unwrap();
        let expected: f64 = GAPS_2400[2] + GAPS_2400[3] + GAPS_2400[4];
        assert_eq!(d.out_width(), (5000.0 + expected).floor() as usize);
    }
}
