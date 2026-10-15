# Hardware observations

Everything below was measured on one VF3240 attached to a Mac (macOS,
libusb 1.0.29, vendor ICA driver moved out of `/Library/Image Capture/Devices`).
Values that are properties of this particular unit (calibration results, seam
gaps, chip stagger) are marked as such.

## Captures in this directory

| File | What |
|---|---|
| `2026-10-13-register-dump-poweron.txt` | Full 9-bit register file via `examples/probe`, idle after power-on, before any write. Connection flags `0x00`. |
| `2026-10-13-buttons-regwatch.txt` | `examples/regwatch` log while the four buttons were pressed: key-status events and the live bits in register `0x31`. |

Scans and still frames are not kept in the repository.

## Registers and status

- `asic::initiate(300,300)` is accepted; `0x02` reads back `0x78`, `0x0b` `0x6c`, `0x35` `0x7e` as documented.
- `0x32` has read-only/input bits (GPIO): written `0x69` it reads back `0xe1` (bit 7 reads 1, bit 3 reads 0). Bit 0 toggles by itself every ~4 s.
- Status `0x101` at idle-at-home is `0xdc`/`0xd8`: bits 7, 6, 4, 3 (+2 when the lamp is on). Bit 6 ("no data yet") and bit 4 ("finished") are set at idle.
- The data-ready counter `0x102` reads 8226 / 7407 for 8-bit still frames and 9995 for a 16-bit one, dropping to 0 after the lines are read; its units are unknown.

## Power saving

- Clearing `0x32` bit 5 keeps the unit awake. Every `asic::initiate` re-arms it through the init table (`0x32 = 0x61/0x7d`, bit 5 set; `0x32` reads `0xe0` after any scan). The unit powers itself down after a few minutes idle unless bit 5 is cleared again after the last init; `job::scan` does that at the end (`0x32` reads `0xc0` afterwards).

## Motor

- 400 steps away at 800 pps, unit 2: status `0xdc → 0xd9` (moving) at 14 ms, home bit cleared at 172 ms, `0xf0` (stopped, not home, bit 5 set) at 716 ms.
- 600 steps toward home from 400 away: home bit set at 586 ms, stopped at 678 ms. The ASIC stops a toward-home move on the home sensor.
- `0x37[4]`/`0x38[4]` put the ASIC in a standalone-move mode that overrides scan-driven motion: positioning onto the strip with the variant that sets them (`move_motor2`, like the vendor's home move) leaves the carriage stationary during the following scan (the whole "image" is the white strip, homing takes 0.17 s). The plain `ScanCtrl_MoveMotor` sequence (no bit-4 writes) must be used for the calibration move. The init table clears the bits, which is why homing before init never shows the problem.
- Motor current (Vref 0–3) does not change the staircase (below); the vendor's level 2 is the mildest and stays the default.

## Still frames and readout

- Still mode (`spStillMode = 1`, `0x0f = 0xff` with `0x02[4] = 0`) does not move the carriage. Data is ready 5 ms after start.
- 225 full steps from home the sensor sees the white strip: channel peaks B 215 / R 241 / G 211 with default exposure and gain. The left ~5 % of the line is dark (segment edge / dummy region).
- 16-bit frames: u16 LE samples ≈ 256 × the 8-bit values (B max 54328, R 61418, G 53629). Per-segment means differ by 15–20 % (segment 1 lowest) before shading.
- The hardware gamma 1.6 LUT (upload + `0x05` bit 3) works; it is applied in 16-bit mode too (the 16-bit top byte matches the 8-bit scan, G mean 158.5 vs 158.6).

## Calibration

- The vendor sequence (AFE offset bisection, exposure bisection, moving white shading, still dark shading, bank upload) converges every time: offsets in ~7 passes, exposure in ~5, 2.1–2.9 s depending on dpi. Hardware shading leaves a static frame of the strip flat at 244.8 ± 0.3 across all columns; on a scan the column-to-column variation on a bright band drops from ~3.7 to ~1 (8-bit levels).
- Repeated calibrations give the same shutters; the B offset alternates between two neighbouring table entries (0x09c/0x098).
- Results on this unit (colour; gray uses the same bisections):

| dpi | AFE offsets R/G/B | shutters R/G/B (ms) | white |
|---|---|---|---|
| 300 | 0x090/0x056/0x080 | 0.265/0.31/0.151 | 173/169/178 |
| 600 | 0x098/0x062/0x088 | 0.505/0.5875/0.265 | |
| 1200 | 0x0a8/0x068/0x098 | 0.989/1.233/0.490 | |
| 2400 | 0x0b0/0x074/0x0a0 | 1.9675/2.457/0.989 | 174/178/178 |

- At 2400 dpi the default 2.62 ms shutters clip B at 255; calibration must run first.
- Gray white frames mean 39307 / 39592 / 39949 / 40476 of 65535 at 300 / 600 / 1200 / 2400 dpi, dark ≈ 2544–2720.

## Resolutions, geometry and timing

| dpi | output (colour) | scan time, full bed | notes |
|---|---|---|---|
| 200 | 2396 × 3359 | 3.9 s (12.4 s total) | calibrates in the 300 dpi bucket; `DPISET = 800`, `CurveData1` (2584 pps, 4 microsteps); aspect correct, confirming the ASIC's 1200 → 800 decimation |
| 300 | 3593 × 5040 | 12.6 s + 5.6 s homing | |
| 600 | 7187 × 10080 | 24.8 s + 5.6 s | slightly USB-bound (21.7 s line budget) |
| 1200 | 14375 × 20160 | 105 s + 5.6 s | |
| 2400 | 28751 × 40320 | 407 s + 5.6 s | 3.48 GB classic TIFF uncompressed |

- Widths above are before the seam fill; the fill adds the sum of the gaps (≈ 68 px at 2400 dpi).
- 2400 dpi timing is exactly the TG period: 3 sub-lines × 3.367 ms = 10.1 ms per line; three back-to-back full-bed runs take 417.02 / 417.01 / 417.01 s with phases identical to ±10 ms (home 0.03 s, calibration 2.86 s, data ready at 4.08 s, last line at 411.37 s, homing 5.6 s). The host is not on the critical path: RSS 18.5 MB, 3.7 % CPU; unpacking 49 µs/line; the 3.48 GB sequential write 1–3 s, absorbed by the scanner's SDRAM line buffer. The 2400 dpi motor runs from a 4-word slope table at time base 4 (`0x1c` = 2).
- Geometry at 2400 and 600 dpi agrees to the pixel when compared at 150 dpi (zero profile shift in X and Y). Magazine halftone dots are individually resolved at 2400 dpi.
- Area selection (`--area x,y,w,h` inches, image coordinates): a 2,3,4,3 in area at 300 dpi matches the same region of the full-bed scan at an offset of 1 column (the seam shift left of the crop) and −2 rows (lead-in accuracy of the vendor's `spScanBeginLine` formula). Data ready after 5.2 s (fast feed over the 3 in).
- Gray mode sits 3 rows lower than the colour scan of the same page (mode-dependent lead-in, not characterised). Gray image mean 160.9 vs 161.6 for the colour scan's 0.3R+0.5G+0.2B; the LED weighting is not distinguishable from the output.
- 16-bit output works at every dpi, colour and gray.

## USB throughput and output files

- USB tops out near 8.5 MB/s: a 16-bit 2400 dpi full bed (186 KB per line) runs at ~40 lines/s instead of 100, i.e. 16.6 min instead of 7; the ASIC pauses the carriage while the host catches up, so the image is unaffected. Per-line synchronous bulk reads of 93–186 KB; the vendor reads 64 KB blocks, so it is no faster. Untested: several outstanding bulk transfers (libusb async).
- Files above 3.5 GiB of pixel data (16-bit full beds at 2400 dpi) are written as BigTIFF; our reader handles both. A 16-bit 2400 dpi full bed is 5.98 GB as BigTIFF with Deflate + predictor and 5.99 GB as PNG. libtiff lists all 630 strips and macOS ImageIO decodes regions (Preview opens it). Pillow fails on both with "unrecognized data stream contents" because the pixel data exceeds 4 GiB, a Pillow limit.

## Colour registration

- There is no lateral colour misregistration: on a black-and-white page at 2400 dpi (0.97 M vertical and 1.38 M horizontal edges, `examples/fringe`) R−G is −0.01…−0.02 px and B−G −0.01…−0.05 px in every segment. Apparent per-segment lateral offsets measured on a magazine page were content (edges that differ per channel).
- Vertically, R and B land 0.42 and 0.72 lines below G at every column (sd 0.3). The vendor's macOS output shows the same (+0.47 / +0.70), as do our 600 dpi (+0.35 / +0.68) and 300 dpi (≈ +0.28 / +0.59) scans: the three exposures happen at different moments within the line period and the ASIC does not compensate. At 1200 dpi the values are 0.55 / 0.81.
- The sign fixes the flash order: with row index increasing along the travel, R and B see the paper 0.42 and 0.72 lines *before* G, so within a line period the order is B, R, G. The staircase phases agree: the slide that G places in one row interval is split for R and B across that interval and the one before it (B 1.34 / 1.69, R 1.88 / 0.80 lines), not the one after.
- `vf3240_image::register` resamples R and B vertically (Lanczos-3) by constant shifts; residuals are within ±0.05 lines at every dpi.
- The `fringe` tool under-reports a synthetic +1.5 px shift as +1.18 (bias ≈ −20 %).

## Motor staircase

- At 2400 dpi the carriage advances in one jerk per full motor step (MotorGear 600 full steps/inch → 4 lines per full step, microstepped at unit 8, 25 full steps/s): line-to-line G differences by line mod 4 are 2.5 / 3.3 / 5.7 / 8.2, autocorrelation +0.99 at lags 4, 8, 12, 16. The vendor's 2400 dpi output has the same (+0.98 at lag 4). Absent at 600 and 300 dpi. At 1200 dpi the period is 2 lines (ratio 4.0 at the top of the bed falling to 2.6 lower down).
- Physical spacings of the four lines of a step, from horizontal-edge crossings per row interval (B/W page): G 0.513 / 1.133 / 2.041 / 0.313 lines; R 0.44 / 0.89 / 1.88 / 0.80; B 0.33 / 0.64 / 1.34 / 1.69 (R and B sample the carriage at other moments). Motor current does not change this: jump/dwell ratio 2.6× at Vref 2, 5.1× at 3, 4.1× at 1, 3.2× at 0.
- The stick-slip point drifts along the rail: the jump sits on interval 3→0 for rows 0–10k, 2→3 until ~21k, 1→2 until ~32k, then 0→1, at the same rows on two different pages, i.e. about one line per 10,000 rows as a deterministic function of carriage position. A fixed table per scan is therefore wrong.
- `vf3240_image::stairs` measures the four spacings per channel continuously in a sliding window (5 blocks × 200 rows) from edge counts and resamples every row (monotone cubic through the channel's samples at their measured positions). Results: jump/dwell ratio 1.1–1.4 in every block at 2400 dpi (raw 2–8) and 1.2–1.27 at 1200 dpi (raw 2.6–4.0); colour residuals within ±0.025 lines; rows interpolated across the 2-line gap stay softer (information limit). Lanczos on the non-uniform grid rings and must not be used. Validated on a synthetic with 0.8-line optical blur: error vs the ideal image 4.6 / 2.8 / 5.5 → 0.7 / 0.8 / 0.5 levels, positions recovered within 0.05 lines. Cost ~1.7 ms/row at 2400 dpi.

## Physical limits of the bed

- The ASIC always delivers the whole sensor line; the host crops it. At 300 dpi (3888 px) the first 208 pixels are dummies (flat gray, no image), 208–217 roll off, 218–251 get only a few percent of the light (on the white strip they read 8–30 against 12 on black, so shading multiplies them into saturation), 252–255 ramp up, and 256–3864 see the strip at full response; the rest is dark segment edge. The vendor's window is 256–3849: nothing usable is lost on the left, 15 full-response pixels (1.3 mm) on the right. Shading is computed and uploaded for all 3888 pixels, so a full-line scan is calibrated like any other.
- The scan origin lies on the white calibration strip: rows 0–40 at 300 dpi show it, the glass starts around row 60.
- **Do not scan past the vendor's bed length.** Asked for 5190 rows at 300 dpi the carriage hit the end stop at row ≈ 5113 (17.04 in from the origin), audibly, and the last 77 rows repeat one line. The glass ends at about row 5080 (16.9 in); the vendor's 5040 rows (16.8 in) stop 3 mm before the glass edge and 6 mm before the hard stop. There is no software end-stop in the ASIC; the requested line count is the only limit.

## Sensor chips: seams and stagger

- The CIS is 18 chips of 1728 pixels at 2400 dpi. Junctions are at image columns 1416 + 1728k (k = 0..15); within the first image segment the new chip starts one pixel later than the formula (spikes between 1416|1417, 3144|3145, 4872|4873, 6600|6601; from 8327|8328 on the formula holds). The vendor does not compensate: its files show the same spikes.
- A few pixels are missing at each junction, and the chip right of each seam also sees the paper a fraction of a row later than the chip left of it. The stagger halves at 1200 dpi at every measurable seam (e.g. 0.45/0.4, 0.65/0.6, 0.6/0.6 rows predicted/measured), so it is a mechanical offset of up to 14 µm between neighbouring chips, accumulating to ~11 rows (116 µm) across the sensor. It is constant along each seam (thirds of the bed agree to ±0.1 row). Whether each chip is displaced or each chip is slightly rotated cannot be told from the seam jumps; the per-chip correction fixes both.
- **The apparent gap and step at a junction depend on how the sheet lies on the glass.** Three placements of the printed target (one turned by 180°) and the magazine page give, per seam, gaps that differ by 0.2–1.5 px (median range 0.5) and steps by 0.1–0.8 rows, while each scan reproduces itself from its file within 0.03 px and two independent methods (the 2D lattice phase fit and a direct 2D patch match of the dot pattern across the seam with the lattice period measured from the same rows) agree within each placement. The effect is therefore in the image, not in the measurement; the likely cause is the segmented rod-lens array, at whose joints the lateral image position depends on the paper's height above the glass. `--calibrate-seams` therefore scans three placements and takes the median; a single placement is good to about ±0.5 px.
- Values for this unit at 2400 dpi: `--calibrate-seams` over the three target placements, per-seam median with the half-range across placements (45 bands per seam per placement, spread 0.2–0.3 px; seam 1 from two placements):

| seam | column | gap (px) | range | step (rows) | range |
|---|---|---|---|---|---|
| 1 | 1417 | 4.08 | ±0.12 | +0.00 | ±0.01 |
| 2 | 3145 | 3.76 | ±0.17 | +1.20 | ±0.03 |
| 3 | 4873 | 3.72 | ±0.29 | +0.95 | ±0.26 |
| 4 | 6601 | 4.19 | ±0.11 | +0.60 | ±0.11 |
| 5 | 8328 | 5.37 | ±0.38 | +1.30 | ±0.19 |
| 6 | 10056 | 4.33 | ±0.51 | +0.55 | ±0.49 |
| 7 | 11784 | 4.12 | ±0.10 | +0.80 | ±0.19 |
| 8 | 13512 | 4.91 | ±0.94 | +0.40 | ±0.21 |
| 9 | 15240 | 3.80 | ±0.41 | +1.25 | ±0.16 |
| 10 | 16968 | 3.28 | ±0.44 | +0.55 | ±0.15 |
| 11 | 18696 | 4.51 | ±0.56 | +1.10 | ±0.21 |
| 12 | 20424 | 4.11 | ±0.23 | +0.00 | ±0.18 |
| 13 | 22152 | 3.92 | ±0.20 | +1.10 | ±0.22 |
| 14 | 23880 | 4.48 | ±0.33 | +0.95 | ±0.33 |
| 15 | 25608 | 4.22 | ±0.26 | +0.95 | ±0.18 |
| 16 | 27336 | 5.02 | ±0.57 | +0.80 | ±0.06 |

- Within one placement the live fill and stagger correction leave residual gaps within ±0.22 px and steps within 0.1 row when measured with that placement's own table; for an arbitrary page the placement dependence above sets the limit.
- An independent edge-based check (`examples/seamdy`, a horizontal rule crossing four seams, page skewed 6.7°, slope × gap removed) gives steps of 0.0 / 0.7 / 0.1 / 0.7 rows where the lattice method says 0.7 / 1.2 / −0.1 / 1.0: same sign, coarser agreement.
- `--measure-seams` in the binary reproduces the hand-run tool on the same scan within 0.02 px and 0.05 row (62 s). `--calibrate-seams` on the printed target takes 213 s for the scan plus the analysis.

## Measurement methods and their validation

- Seam gap and stagger (`seamcal`, `halftone_gap --mode 2d`): lattice vectors from the band's 2D spectrum, phase jump at the seam for up to four of them, gap and step solved jointly. On a synthetic rotated dot screen with known gaps 3 / 4 / 2 px and one 1-row step (`examples/synth_halftone`) it returns 3.04 / 3.99 / 2.00 px and 0.2 / −0.1 / 1.0 rows. The earlier single-vector method (vertical lattice component only) reads 3.35 / 3.89 / 0.93 on the same synthetic and read the real seams about 1 px low because of the stagger; it is valid only where the lattice's horizontal phase period is large compared with the gap. 600 dpi scans have no usable lattice (period below 8 px).
- Staircase spacings from edge counts: horizontal-edge crossings per row interval are proportional to the physical spacing.
- Spike scan (`seams --all`): adjacent-column G difference spikes 3–7× at the junctions in uncorrected scans; 0 spikes after the fill at 300, 600 and 2400 dpi.

## Buttons

- Pressing the four front buttons gives key-status (`0x8E`/`wIndex 0x21`) events `0x04`, `0x08`, `0x20`, `0x40`, each reported once and cleared by the read (a 3 s hold still reports once). Register `0x31` shows the live state, active low (`0xff` → `0xfb`/`0xf7`/`0xdf`/`0xbf` while held). The interrupt endpoint `0x83` produces nothing.
- The power button's short presses drop register `0x33` bit 1 for ~0.35 s with no key-status event and do not power the unit off: a live-only input.
- `vf3240 --multiple` scans on each press; calibration repeats to identical values between presses.

## Vendor comparison

- Our calibrated output matches the vendor macOS driver's pixel-for-pixel within noise (±1 level on uniform patches); the vendor applies no colour matrix and none of the three corrections above.

## Open questions

- The units of the data-ready counter `0x102`.
- The 3-row lead-in difference between gray and colour mode.
- Whether several outstanding bulk transfers raise the 8.5 MB/s USB ceiling.
- Whether the chips are displaced or rotated (a long straight rule would tell).
