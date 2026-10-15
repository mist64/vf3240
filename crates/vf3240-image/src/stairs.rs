//! On-the-fly correction of the motor staircase at 2400 dpi.
//!
//! The carriage advances in one jerk per full motor step, which is four
//! lines at 2400 dpi, so the four lines of a step sample the paper at
//! non-uniform positions, and R and B (exposed at other moments within the
//! line period) at yet other positions. The stick-slip point also drifts
//! slowly along the rail (about one line per 10,000 rows, deterministic,
//! `docs/hardware/README.md` 2026-10-13), so no fixed table fits a whole scan.
//!
//! This stage therefore measures continuously: horizontal edges are spread
//! uniformly over the paper, so the number of edge crossings of a channel
//! falling between rows y and y+1 is proportional to their physical spacing;
//! counted per `y mod 4` over a sliding window of `WINDOW` rows they give the
//! four spacings of each channel, and edges matched across channels give the
//! R/B offsets. Each output row is resampled (monotone cubic through the
//! channel's samples at their measured positions) with the tables of the
//! window centred on it, so the stream lags by about half a window. Windows
//! with too few edges reuse the last good tables; before the first good
//! window the fallback is uniform spacing with the constant R/B offsets.

/// Lines per full motor step: 4 at 2400 dpi, 2 at 1200 dpi.
pub const MAX_PERIOD: usize = 4;
/// Rows per measurement block; tables are refreshed every block.
pub const BLOCK: usize = 200;
/// Blocks in the sliding window (BLOCK × BLOCKS rows ≈ 1000).
pub const BLOCKS: usize = 5;
pub const WINDOW: usize = BLOCK * BLOCKS;
/// Edges per channel a window needs before its tables are trusted.
const MIN_EDGES: u64 = 600;
/// Columns sampled for the edge statistics.
const COL_STEP: usize = 8;
const WIDE: usize = 10;
const NEAR: usize = 6;
const MIN_CONTRAST: i32 = 90;
const MIN_GRAD: i32 = 40;
const LOOKBACK: usize = 6;

/// Physical position of row `y` for one channel: `4k + s[p] + offset`.
#[derive(Debug, Clone, PartialEq)]
pub struct Positions {
    pub period: usize,
    pub s: [f64; MAX_PERIOD],
    pub d: [f64; MAX_PERIOD],
    pub offset: f64,
}

impl Positions {
    pub fn uniform(period: usize, offset: f64) -> Self {
        Positions {
            period,
            s: [0.0, 1.0, 2.0, 3.0],
            d: [1.0; MAX_PERIOD],
            offset,
        }
    }
    /// From edge counts per phase (interval p → p+1), centred so that on
    /// average a row sits at its own index.
    pub fn from_counts(period: usize, counts: &[u64]) -> Self {
        let total: f64 = counts[..period].iter().map(|&c| c as f64).sum();
        let mut d = [1.0; MAX_PERIOD];
        if total > 0.0 {
            for p in 0..period {
                d[p] = (period as f64 * counts[p] as f64 / total).max(0.05);
            }
            let sum: f64 = d[..period].iter().sum();
            d[..period]
                .iter_mut()
                .for_each(|v| *v *= period as f64 / sum);
        }
        let mut s = [0.0; MAX_PERIOD];
        for p in 1..period {
            s[p] = s[p - 1] + d[p - 1];
        }
        let mean_dev: f64 = (0..period).map(|p| s[p] - p as f64).sum::<f64>() / period as f64;
        Positions {
            period,
            s,
            d,
            offset: -mean_dev,
        }
    }
    pub fn pos(&self, y: f64) -> f64 {
        let yi = y.floor();
        let p = (yi.rem_euclid(self.period as f64)) as usize;
        let k = yi - p as f64;
        k + self.s[p] + (y - yi) * self.d[p] + self.offset
    }
}

fn crossing(v: &[i32], min_contrast: i32) -> Option<f64> {
    let (lo, hi) = (*v.iter().min().unwrap(), *v.iter().max().unwrap());
    if hi - lo < min_contrast {
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

/// Edge statistics of one block: counts per channel and phase, and matched
/// (R, G, B) crossing positions in absolute row units.
#[derive(Default, Clone)]
pub struct BlockStats {
    pub counts: [[u64; MAX_PERIOD]; 3],
    pub matched: Vec<(f64, f64, f64)>,
}

/// Measure `rows` (interleaved RGB, all the same width), whose first row has
/// absolute index `y0`. Edges whose window would leave the slice are skipped,
/// so pass `WIDE` extra rows on both sides when possible.
pub fn measure(rows: &[&[u16]], y0: usize, width: usize, period: usize, scale: i32) -> BlockStats {
    let n = rows.len();
    let mut st = BlockStats::default();
    if n < 2 * WIDE + 2 {
        return st;
    }
    let (min_contrast, min_grad) = (MIN_CONTRAST * scale, MIN_GRAD * scale);
    let mut prof = vec![0i32; 3 * n];
    for x in (0..width).step_by(COL_STEP) {
        for (y, row) in rows.iter().enumerate() {
            for c in 0..3 {
                prof[3 * y + c] = i32::from(row[3 * x + c]);
            }
        }
        let v = |i: usize, c: usize| prof[3 * i + c];
        // Own edges per channel.
        for c in 0..3 {
            let mut i = WIDE;
            while i + WIDE + 1 < n {
                let grad = (v(i + 1, c) - v(i - 1, c)).abs();
                if grad >= min_grad
                    && grad >= (v(i + 2, c) - v(i, c)).abs()
                    && grad > (v(i, c) - v(i - 2, c)).abs()
                {
                    let win: Vec<i32> = (i - WIDE..=i + WIDE).map(|k| v(k, c)).collect();
                    if let Some(p) = crossing(&win, min_contrast) {
                        let e = i as f64 - WIDE as f64 + p;
                        st.counts[c][(e.floor() as usize + y0) % period] += 1;
                    }
                    i += WIDE;
                }
                i += 1;
            }
        }
        // Matched edges located on G.
        let mut i = WIDE;
        while i + WIDE + 1 < n {
            let grad = (v(i + 1, 1) - v(i - 1, 1)).abs();
            if grad >= min_grad
                && grad >= (v(i + 2, 1) - v(i, 1)).abs()
                && grad > (v(i, 1) - v(i - 2, 1)).abs()
            {
                let win = |c: usize| (i - WIDE..=i + WIDE).map(|k| v(k, c)).collect::<Vec<_>>();
                if let (Some(pr), Some(pg), Some(pb)) = (
                    crossing(&win(0), min_contrast),
                    crossing(&win(1), min_contrast),
                    crossing(&win(2), min_contrast),
                ) {
                    if (pr - pg).abs() < 4.0 && (pb - pg).abs() < 4.0 {
                        let base = i as f64 - WIDE as f64 + y0 as f64;
                        st.matched.push((base + pr, base + pg, base + pb));
                    }
                }
                i += WIDE;
            }
            i += 1;
        }
    }
    st
}

/// Tables from the statistics of several blocks, or `None` if too few edges.
pub fn tables(stats: &[BlockStats], period: usize) -> Option<[Positions; 3]> {
    let mut counts = [[0u64; MAX_PERIOD]; 3];
    let mut matched: Vec<(f64, f64, f64)> = Vec::new();
    for b in stats {
        for (c, count) in counts.iter_mut().enumerate() {
            for (p, slot) in count.iter_mut().enumerate() {
                *slot += b.counts[c][p];
            }
        }
        matched.extend_from_slice(&b.matched);
    }
    if counts.iter().any(|c| c.iter().sum::<u64>() < MIN_EDGES) || matched.len() < 200 {
        return None;
    }
    let mut pos = [
        Positions::from_counts(period, &counts[0]),
        Positions::from_counts(period, &counts[1]),
        Positions::from_counts(period, &counts[2]),
    ];
    for c in [0usize, 2] {
        let mut diffs: Vec<f64> = matched
            .iter()
            .map(|&(r, g, b)| pos[c].pos(if c == 0 { r } else { b }) - pos[1].pos(g))
            .collect();
        diffs.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let n = diffs.len();
        let core = &diffs[n / 10..n - n / 10];
        let mean = core.iter().sum::<f64>() / core.len().max(1) as f64;
        // A crossing `mean` rows later than G's means the channel imaged a
        // spot `mean` lines earlier along the carriage path.
        pos[c].offset -= mean;
    }
    Some(pos)
}

/// Resample output row `y` of channel `c`, columns `cols`, from `fetch(i)`
/// rows (absolute indices, clamped by the caller) using `pos`; writes
/// channel `c` of `out`. `chip` is `(columns, vertical stagger in rows)`:
/// those columns' chip shows the paper that many rows later, so the output
/// row is taken from that much earlier in the stream.
pub fn resample_row<'a>(
    pos: &Positions,
    c: usize,
    y: usize,
    chip: (std::ops::Range<usize>, f64),
    fetch: &dyn Fn(isize) -> &'a [u16],
    out: &mut [u16],
    max_value: f64,
) {
    let (cols, chip_offset) = chip;
    let target = y as f64 - chip_offset;
    let mut i = y as isize - LOOKBACK as isize;
    while pos.pos((i + 1) as f64) <= target {
        i += 1;
    }
    while i > y as isize - LOOKBACK as isize && pos.pos(i as f64) > target {
        i -= 1;
    }
    let p = |k: isize| pos.pos(k as f64);
    let (p0, p1, p2, p3) = (p(i - 1), p(i), p(i + 1), p(i + 2));
    let (h0, h1, h2) = (p1 - p0, p2 - p1, p3 - p2);
    let tt = (target - p1) / h1;
    let (r0, r1, r2, r3) = (fetch(i - 1), fetch(i), fetch(i + 1), fetch(i + 2));
    let (t2, t3) = (tt * tt, tt * tt * tt);
    let (b00, b10, b01, b11) = (
        2.0 * t3 - 3.0 * t2 + 1.0,
        t3 - 2.0 * t2 + tt,
        -2.0 * t3 + 3.0 * t2,
        t3 - t2,
    );
    let slope = |dl: f64, hl: f64, dr: f64, hr: f64| -> f64 {
        if dl * dr <= 0.0 {
            0.0
        } else {
            let (wl, wr) = (2.0 * hr + hl, hr + 2.0 * hl);
            (wl + wr) / (wl / dl + wr / dr)
        }
    };
    for x in (cols.start * 3 + c..cols.end * 3).step_by(3) {
        let (v0, v1, v2, v3) = (
            f64::from(r0[x]),
            f64::from(r1[x]),
            f64::from(r2[x]),
            f64::from(r3[x]),
        );
        let (d0, d1, d2) = ((v1 - v0) / h0, (v2 - v1) / h1, (v3 - v2) / h2);
        let m1 = slope(d0, h0, d1, h1);
        let m2 = slope(d1, h1, d2, h2);
        let v = b00 * v1 + b10 * h1 * m1 + b01 * v2 + b11 * h1 * m2;
        out[x] = v.round().clamp(0.0, max_value) as u16;
    }
}

/// Streaming stage. Rows go in; output rows come out `WINDOW / 2 + BLOCK`
/// rows later with the tables of the window centred on them.
pub struct Destair {
    width: usize,
    period: usize,
    max_value: f64,
    received: usize,
    emitted: usize,
    /// All rows from `first` on, oldest first.
    rows: std::collections::VecDeque<Vec<u16>>,
    first: usize,
    /// Statistics per completed block, oldest first (kept: BLOCKS + 1).
    stats: std::collections::VecDeque<(usize, BlockStats)>,
    current: [Positions; 3],
    /// Column ranges of the chips with their vertical stagger in rows.
    chips: Vec<(usize, usize, f64)>,
    pub good_blocks: usize,
    pub total_blocks: usize,
    out: Vec<u16>,
}

impl Destair {
    /// `period`: lines per full motor step (4 at 2400 dpi, 2 at 1200 dpi).
    /// `max_value`: 255 for 8-bit samples, 65535 for 16-bit (edge thresholds scale with it).
    pub fn new(width: usize, period: usize, fallback_shifts: (f64, f64), max_value: u16) -> Self {
        // register's convention: out_c(y) = in_c(y + shift) ⇔ channel row y shows pos_G(y) − shift.
        let fallback = [
            Positions::uniform(period, -fallback_shifts.0),
            Positions::uniform(period, 0.0),
            Positions::uniform(period, -fallback_shifts.1),
        ];
        Destair {
            width,
            period,
            max_value: f64::from(max_value),
            chips: vec![(0, width, 0.0)],
            received: 0,
            emitted: 0,
            rows: std::collections::VecDeque::new(),
            first: 0,
            stats: std::collections::VecDeque::new(),
            current: fallback,
            good_blocks: 0,
            total_blocks: 0,
            out: vec![0; width * 3],
        }
    }

    fn block_done(&mut self, b: usize) {
        // Rows of block b with WIDE rows of margin on both sides, as available.
        let y0 = (b * BLOCK).saturating_sub(WIDE).max(self.first);
        let y1 = ((b + 1) * BLOCK + WIDE).min(self.received);
        let slice: Vec<&[u16]> = (y0..y1)
            .map(|y| self.rows[y - self.first].as_slice())
            .collect();
        let st = measure(
            &slice,
            y0,
            self.width,
            self.period,
            (self.max_value as i32 + 1) / 256,
        );
        self.stats.push_back((b, st));
        while self.stats.len() > BLOCKS + 1 {
            self.stats.pop_front();
        }
        self.total_blocks += 1;
    }

    /// Tables for output rows of block `b`: the window of BLOCKS blocks centred on it.
    fn tables_for(&mut self, b: usize) -> [Positions; 3] {
        let lo = b.saturating_sub(BLOCKS / 2);
        let window: Vec<BlockStats> = self
            .stats
            .iter()
            .filter(|(k, _)| *k >= lo && *k < lo + BLOCKS)
            .map(|(_, s)| s.clone())
            .collect();
        if std::env::var("STAIRS_DEBUG").is_ok() {
            let sums: Vec<u64> = (0..3)
                .map(|c| window.iter().map(|w| w.counts[c].iter().sum::<u64>()).sum())
                .collect();
            let matched: usize = window.iter().map(|w| w.matched.len()).sum();
            let g: Vec<u64> = (0..self.period)
                .map(|p| window.iter().map(|w| w.counts[1][p]).sum::<u64>())
                .collect();
            eprintln!("block {b}: window blocks {:?}, edges {sums:?}, matched {matched}, G per phase {g:?}", self.stats.iter().filter(|(k, _)| *k >= lo && *k < lo + BLOCKS).map(|(k, _)| *k).collect::<Vec<_>>());
        }
        match tables(&window, self.period) {
            Some(t) => {
                self.good_blocks += 1;
                self.current = t.clone();
                t
            }
            None => self.current.clone(),
        }
    }

    /// Per-chip column ranges and vertical stagger (`seams::chip_offsets`).
    pub fn with_chip_offsets(mut self, chips: Vec<(usize, usize, f64)>) -> Self {
        if !chips.is_empty() {
            self.chips = chips;
        }
        self
    }

    pub fn push(
        &mut self,
        row: Vec<u16>,
        emit: &mut dyn FnMut(&[u16]) -> std::io::Result<()>,
    ) -> std::io::Result<()> {
        debug_assert_eq!(row.len(), self.width * 3);
        self.rows.push_back(row);
        self.received += 1;
        let y = self.received - 1;
        // A block is complete once WIDE rows beyond it have arrived.
        if y + 1 >= BLOCK + WIDE && (y + 1 - WIDE) % BLOCK == 0 {
            let b = (y + 1 - WIDE) / BLOCK - 1;
            self.block_done(b);
            // Output block b − BLOCKS/2 can now be resampled with a centred window.
            if b >= BLOCKS / 2 {
                self.emit_block(b - BLOCKS / 2, emit)?;
            }
        }
        Ok(())
    }

    fn emit_block(
        &mut self,
        ob: usize,
        emit: &mut dyn FnMut(&[u16]) -> std::io::Result<()>,
    ) -> std::io::Result<()> {
        let pos = self.tables_for(ob);
        let (y_start, y_end) = (ob * BLOCK, ((ob + 1) * BLOCK).min(self.received));
        for y in y_start.max(self.emitted)..y_end {
            self.resample(y, &pos);
            emit(&self.out)?;
            self.emitted = y + 1;
        }
        // Drop rows no longer needed: keep LOOKBACK + 4 before the next output row,
        // and everything the pending blocks' statistics still need (2 blocks + margins).
        let keep_from = self
            .emitted
            .saturating_sub(LOOKBACK + 4)
            .min((ob + 1) * BLOCK);
        let stats_need = self.received.saturating_sub(BLOCK + 2 * WIDE);
        let keep_from = keep_from.min(stats_need);
        while self.first < keep_from && !self.rows.is_empty() {
            self.rows.pop_front();
            self.first += 1;
        }
        Ok(())
    }

    fn resample(&mut self, y: usize, pos: &[Positions; 3]) {
        let (first, last) = (self.first, self.received - 1);
        let rows = &self.rows;
        let fetch = |i: isize| -> &[u16] {
            let i = (i.max(first as isize) as usize).min(last);
            rows[i - first].as_slice()
        };
        let mut out = std::mem::take(&mut self.out);
        for (c, pc) in pos.iter().enumerate() {
            for &(start, end, off) in &self.chips {
                resample_row(
                    pc,
                    c,
                    y,
                    (start..end, off),
                    &fetch,
                    &mut out,
                    self.max_value,
                );
            }
        }
        self.out = out;
    }

    pub fn finish(
        &mut self,
        emit: &mut dyn FnMut(&[u16]) -> std::io::Result<()>,
    ) -> std::io::Result<()> {
        // Measure the partial last block, then emit everything left.
        let done_blocks = self.stats.back().map_or(0, |(b, _)| b + 1);
        let last_block = self.received.saturating_sub(1) / BLOCK;
        for b in done_blocks..=last_block {
            self.block_done(b);
        }
        let mut ob = self.emitted / BLOCK;
        while self.emitted < self.received {
            self.emit_block(ob, emit)?;
            ob += 1;
        }
        Ok(())
    }

    pub fn summary(&self) -> String {
        format!(
            "staircase (period {}): {} of {} blocks measured; last G spacings {:.2?}, R offset {:+.2}, B offset {:+.2}",
            self.period,
            self.good_blocks,
            self.total_blocks,
            &self.current[1].d[..self.period],
            self.current[0].offset - self.current[1].offset,
            self.current[2].offset - self.current[1].offset
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(rows: &[Vec<u16>], width: usize) -> (Vec<Vec<u16>>, String) {
        let mut st = Destair::new(width, 4, (0.42, 0.72), 255);
        let mut out = Vec::new();
        let mut emit = |r: &[u16]| {
            out.push(r.to_vec());
            Ok(())
        };
        for r in rows {
            st.push(r.clone(), &mut emit).unwrap();
        }
        st.finish(&mut emit).unwrap();
        (out, st.summary())
    }

    /// A synthetic page of random horizontal bars sampled at staircase
    /// positions whose phase drifts by `drift` lines per row, R and B
    /// imaging 0.4 / 0.7 lines earlier than G.
    fn synthetic(n: usize, width: usize, drift: f64) -> (Vec<Vec<u16>>, Vec<Vec<u16>>) {
        let mut seed = 12345u64;
        let mut rnd = || {
            seed = seed
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (seed >> 33) as f64 / (1u64 << 31) as f64
        };
        // Bars: (column range, y start, y end, value) in physical lines, blurred by a 0.8-line ramp.
        let bars: Vec<(usize, usize, f64, f64, f64)> = (0..n * width / 60)
            .map(|_| {
                let x0 = (rnd() * width as f64) as usize;
                let x1 = (x0 + 2 + (rnd() * 6.0) as usize).min(width);
                let y0 = rnd() * n as f64;
                (
                    x0,
                    x1,
                    y0,
                    y0 + 3.0 + rnd() * 30.0,
                    if rnd() < 0.5 { 20.0 } else { 230.0 },
                )
            })
            .collect();
        let mut canvas = vec![vec![235.0f64; width]; 4 * n + 40];
        for &(x0, x1, y0, y1, v) in &bars {
            for row in canvas
                .iter_mut()
                .take(((y1 * 4.0) as usize).min(4 * n + 39))
                .skip(((y0 * 4.0) as usize).min(4 * n + 39))
            {
                for cell in row.iter_mut().take(x1).skip(x0) {
                    *cell = v;
                }
            }
        }
        let sample = |p: f64, x: usize| -> u16 {
            // 3-tap vertical blur at quarter-line resolution.
            let q = (p * 4.0).round() as isize;
            let mut acc = 0.0;
            for k in -2..=2isize {
                let yy = (q + k).clamp(0, 4 * n as isize + 39) as usize;
                acc += canvas[yy][x] * [0.1, 0.2, 0.4, 0.2, 0.1][(k + 2) as usize];
            }
            acc.round().clamp(0.0, 255.0) as u16
        };
        let spac = [0.51, 1.13, 2.04, 0.32];
        let pos = |y: usize, shift: f64| -> f64 {
            let u = (y as f64 + drift * y as f64).rem_euclid(4.0);
            let base = y as f64 - (y as f64).rem_euclid(4.0);
            let p = u.floor() as usize;
            let s: f64 = spac[..p].iter().sum();
            base + s + (u - p as f64) * spac[p] - shift
        };
        let stair: Vec<Vec<u16>> = (0..n)
            .map(|y| {
                (0..width)
                    .flat_map(|x| {
                        [
                            sample(pos(y, 0.4), x),
                            sample(pos(y, 0.0), x),
                            sample(pos(y, 0.7), x),
                        ]
                    })
                    .collect()
            })
            .collect();
        let ideal: Vec<Vec<u16>> = (0..n)
            .map(|y| (0..width).flat_map(|x| [sample(y as f64, x); 3]).collect())
            .collect();
        (stair, ideal)
    }

    /// Mean |difference| per channel, skipping `skip` rows at both ends.
    fn mean_abs_diff(a: &[Vec<u16>], b: &[Vec<u16>], skip: usize) -> [f64; 3] {
        let mut sum = [0.0; 3];
        let mut n = 0.0;
        for (ra, rb) in a.iter().zip(b).skip(skip).take(a.len() - 2 * skip) {
            for (k, (x, y)) in ra.iter().zip(rb).enumerate() {
                sum[k % 3] += (f64::from(*x) - f64::from(*y)).abs();
            }
            n += (ra.len() / 3) as f64;
        }
        [sum[0] / n, sum[1] / n, sum[2] / n]
    }

    fn check(drift: f64) {
        let (stair, ideal) = synthetic(3000, 400, drift);
        let (out, summary) = run(&stair, 400);
        assert_eq!(out.len(), stair.len());
        let before = mean_abs_diff(&stair, &ideal, 100);
        let after = mean_abs_diff(&out, &ideal, 100);
        // R and B carry the 0.4 / 0.7 line offsets on top of the staircase and
        // must improve a lot; G's gain is limited by the 2-line gap in each
        // step, where no sample exists (the bars here are sharper than optics).
        let rb_bound = if drift == 0.0 { 0.5 } else { 0.75 };
        for c in [0usize, 2] {
            assert!(
                after[c] < rb_bound * before[c],
                "drift {drift}: channel {c} before {before:.2?} after {after:.2?}; {summary}"
            );
        }
        // With drift the window average smears the pattern; on real pages
        // (optical blur, 1/10000 drift) the hardware log shows the pattern
        // flattened across the whole scan. Here just require G not to get worse.
        let g_bound = if drift == 0.0 { 0.85 } else { 1.0 };
        assert!(
            after[1] < g_bound * before[1],
            "drift {drift}: G before {before:.2?} after {after:.2?}; {summary}"
        );
    }

    #[test]
    fn corrects_a_static_staircase_on_synthetic_bars() {
        check(0.0);
    }

    #[test]
    fn corrects_a_drifting_staircase_on_synthetic_bars() {
        check(1.0 / 10000.0);
    }

    #[test]
    fn chip_stagger_is_taken_out_of_the_right_hand_chip() {
        // A horizontal edge at row 400 in the left half; the right half's chip
        // sees the paper one row later, so its edge is drawn at row 399.
        let width = 240;
        let n = 1500;
        let rows: Vec<Vec<u16>> = (0..n)
            .map(|y| {
                (0..width)
                    .flat_map(|x| {
                        let edge = if x < width / 2 { 400 } else { 399 };
                        [if y < edge { 220 } else { 30 }; 3]
                    })
                    .collect()
            })
            .collect();
        let mut st = Destair::new(width, 4, (0.0, 0.0), 255)
            .with_chip_offsets(vec![(0, width / 2, -0.5), (width / 2, width, 0.5)]);
        let mut out = Vec::new();
        for r in &rows {
            st.push(r.clone(), &mut |o| {
                out.push(o.to_vec());
                Ok(())
            })
            .unwrap();
        }
        st.finish(&mut |o| {
            out.push(o.to_vec());
            Ok(())
        })
        .unwrap();
        assert_eq!(out.len(), n);
        // Sub-pixel edge position (centroid of the gradient) per half.
        let pos = |x: usize| -> f64 {
            let (mut num, mut den) = (0.0, 0.0);
            for y in 390..410 {
                let g = f64::from(out[y][3 * x + 1]) - f64::from(out[y + 1][3 * x + 1]);
                num += g * (y as f64 + 0.5);
                den += g;
            }
            num / den
        };
        let (l, r) = (pos(10), pos(width - 10));
        assert!((l - r).abs() < 0.1, "left {l:.2} right {r:.2}");
        assert!((l - 399.5).abs() < 0.6, "left {l:.2}");
    }

    #[test]
    fn flat_content_passes_through_with_the_fallback_tables() {
        let rows: Vec<Vec<u16>> = (0..1500).map(|y| vec![(y % 7) as u16 + 100; 30]).collect();
        let (out, summary) = run(&rows, 10);
        assert_eq!(out.len(), rows.len());
        assert!(summary.contains(": 0 of"), "{summary}");
    }
}
