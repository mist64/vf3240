//! On-the-fly colour registration: the scanner exposes R, G and B at
//! different moments within each line period while the carriage moves, so
//! red and blue land a constant fraction of a line below green
//! (`docs/hardware/README.md`, 2026-10-13: measured on a black-and-white page,
//! also present in the vendor's output). This stage resamples R and B
//! vertically with a Lanczos-3 kernel so `out_c(y) = in_c(y + shift_c)`,
//! leaving G untouched. It delays the stream by `LOOKAHEAD` rows.

const LANCZOS_A: isize = 3;
/// Rows kept after the one being emitted (for positive shifts below 1 line).
const LOOKAHEAD: usize = 3;
/// Rows kept before it.
const LOOKBACK: usize = 2;

/// Per-channel vertical shifts in lines (R, B) relative to G.
///
/// Measured with `examples/fringe` on this scanner (2026-10-13) at 2400 dpi
/// on a black-and-white page (1.4 M edges, sd 0.3 lines). The same values
/// apply at every resolution: a colour line is always three sub-lines, and
/// the calibrated shutters scale with the line period, so both parts of the
/// exposure-time difference are the same fraction of a line (magazine scans
/// at 600 and 300 dpi agree within their content noise, ±0.1).
pub fn default_shifts(dpi: u32) -> (f64, f64) {
    match dpi {
        // Measured on the full-bed magazine scan (2026-10-13); the 2-line
        // staircase shifts the apparent offsets. Only the stairs stage's
        // fallback at this dpi.
        1200 => (0.55, 0.81),
        _ => (0.417, 0.718),
    }
}

fn lanczos(x: f64) -> f64 {
    if x == 0.0 {
        return 1.0;
    }
    let a = LANCZOS_A as f64;
    if x.abs() >= a {
        return 0.0;
    }
    let px = std::f64::consts::PI * x;
    a * px.sin() * (px / a).sin() / (px * px)
}

/// Normalised Lanczos-3 weights for source rows `y - LOOKBACK ..= y + LOOKAHEAD`
/// sampling at `y + shift`, or `None` for a negligible shift.
fn taps(shift: f64) -> Option<[f64; LOOKBACK + LOOKAHEAD + 1]> {
    if shift.abs() < 0.02 {
        return None;
    }
    assert!((0.0..1.0).contains(&shift), "shift must be in [0, 1) lines");
    let mut w = [0.0; LOOKBACK + LOOKAHEAD + 1];
    for (k, wk) in w.iter_mut().enumerate() {
        let offset = k as isize - LOOKBACK as isize;
        *wk = lanczos(shift - offset as f64);
    }
    let sum: f64 = w.iter().sum();
    w.iter_mut().for_each(|x| *x /= sum);
    Some(w)
}

/// Streaming registration filter. Push rows in; it calls `emit` with each
/// output row once enough following rows are available; `finish` flushes the tail.
pub struct Registration {
    row_bytes: usize,
    /// Largest sample value (255 or 65535).
    max_value: f64,
    taps_r: Option<[f64; LOOKBACK + LOOKAHEAD + 1]>,
    taps_b: Option<[f64; LOOKBACK + LOOKAHEAD + 1]>,
    /// Rows received so far, oldest first; at most LOOKBACK + LOOKAHEAD + 1 kept.
    window: std::collections::VecDeque<Vec<u16>>,
    /// Index into the window of the next row to emit.
    received: usize,
    emitted: usize,
    out: Vec<u16>,
}

impl Registration {
    pub fn new(row_bytes: usize, shift_r: f64, shift_b: f64, max_value: u16) -> Self {
        Registration {
            row_bytes,
            max_value: f64::from(max_value),
            taps_r: taps(shift_r),
            taps_b: taps(shift_b),
            window: std::collections::VecDeque::new(),
            received: 0,
            emitted: 0,
            out: vec![0; row_bytes],
        }
    }

    pub fn is_identity(&self) -> bool {
        self.taps_r.is_none() && self.taps_b.is_none()
    }

    /// Compute output row `y` (absolute index) from the window; the window's
    /// front is row `received - window.len()`.
    fn compute(&mut self, y: usize) -> &[u16] {
        let first = self.received - self.window.len();
        let last = self.received - 1;
        let row_at = |i: isize| -> &Vec<u16> {
            let i = (i.max(first as isize) as usize).min(last);
            &self.window[i - first]
        };
        self.out.copy_from_slice(row_at(y as isize));
        for (c, tp) in [(0usize, self.taps_r), (2usize, self.taps_b)] {
            let Some(w) = tp else { continue };
            let srcs: Vec<&Vec<u16>> = (0..w.len())
                .map(|k| row_at(y as isize + k as isize - LOOKBACK as isize))
                .collect();
            for x in (c..self.row_bytes).step_by(3) {
                let v: f64 = srcs
                    .iter()
                    .zip(w.iter())
                    .map(|(s, wk)| wk * f64::from(s[x]))
                    .sum();
                self.out[x] = v.round().clamp(0.0, self.max_value) as u16;
            }
        }
        &self.out
    }

    pub fn push(
        &mut self,
        row: Vec<u16>,
        emit: &mut dyn FnMut(&[u16]) -> std::io::Result<()>,
    ) -> std::io::Result<()> {
        debug_assert_eq!(row.len(), self.row_bytes);
        if self.is_identity() {
            return emit(&row);
        }
        self.window.push_back(row);
        self.received += 1;
        while self.window.len() > LOOKBACK + LOOKAHEAD + 1 {
            self.window.pop_front();
        }
        // Row y can be emitted once row y + LOOKAHEAD has arrived.
        while self.emitted + LOOKAHEAD < self.received {
            let y = self.emitted;
            let out = self.compute(y).to_vec();
            emit(&out)?;
            self.emitted += 1;
        }
        Ok(())
    }

    pub fn finish(
        &mut self,
        emit: &mut dyn FnMut(&[u16]) -> std::io::Result<()>,
    ) -> std::io::Result<()> {
        while self.emitted < self.received {
            let y = self.emitted;
            let out = self.compute(y).to_vec();
            emit(&out)?;
            self.emitted += 1;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(rows: &[Vec<u16>], sr: f64, sb: f64) -> Vec<Vec<u16>> {
        let mut reg = Registration::new(rows[0].len(), sr, sb, 255);
        let mut out = Vec::new();
        let mut emit = |r: &[u16]| {
            out.push(r.to_vec());
            Ok(())
        };
        for r in rows {
            reg.push(r.clone(), &mut emit).unwrap();
        }
        reg.finish(&mut emit).unwrap();
        out
    }

    #[test]
    fn zero_shift_is_a_pass_through() {
        let rows: Vec<Vec<u16>> = (0..10u16).map(|i| vec![i, i + 1, i + 2]).collect();
        assert_eq!(run(&rows, 0.0, 0.0), rows);
    }

    #[test]
    fn row_count_is_preserved_and_g_untouched() {
        let rows: Vec<Vec<u16>> = (0..20u16)
            .map(|i| vec![i * 10, i * 3, 255 - i * 10, i, i, i])
            .collect();
        let out = run(&rows, 0.417, 0.718);
        assert_eq!(out.len(), rows.len());
        for (a, b) in out.iter().zip(&rows) {
            assert_eq!((a[1], a[4]), (b[1], b[4]));
        }
    }

    #[test]
    fn a_step_in_r_moves_up_by_the_shift() {
        // R is 0 for rows < 10 and 200 from row 10 on: sampling at y + 0.5
        // puts the half-level at row 9.5, i.e. row 9 reads ~mid-way.
        let rows: Vec<Vec<u16>> = (0..20u16)
            .map(|i| vec![if i < 10 { 0 } else { 200 }, 7, 0])
            .collect();
        let out = run(&rows, 0.5, 0.0);
        assert!(out[9][0] > 80 && out[9][0] < 120, "row 9 R = {}", out[9][0]);
        assert_eq!(out[5][0], 0);
        assert_eq!(out[14][0], 200);
        assert!(out.iter().all(|r| r[1] == 7 && r[2] == 0));
    }

    #[test]
    fn taps_are_normalised_and_centred_on_the_shift() {
        let w = taps(0.417).unwrap();
        assert!((w.iter().sum::<f64>() - 1.0).abs() < 1e-9);
        assert!(w[2] > w[3] && w[3] > w[1]); // weight at offset 0 > offset +1 > offset −1
    }
}
