# VF3240 gray (8-bit, `Gray8`) scanning

Status: **verified on hardware (2026-10-13)** at 300 dpi: calibration (colour bisections, 16-bit gray white/dark frames, bank-0 upload) and the gray scan produce a correct image whose mean equals the colour scan's weighted luminance within 1 level. Other resolutions: see the hardware log. Everything that colour and gray share is in
`05-scan-parameters.md`, `07-readout.md` and `09-calibration.md`; this file
lists only the differences.

Sources: `Scanner_Calibration2.c`, `Scanner_Calibration_WhiteShading.c`,
`Scanner_Calibration_DarkShading.c`, `Scanner_Calibration_SetScanParameter.c`,
`Scanner_Calibration_CalcGetShadingDataGray2/3.c`, `CtlImage_WriteShadingGray.c`,
`ScanCtrl_SetScanParameter.c`, `ScanCtrl_Pack_3CH_CIS_Gray.c`, `Scanner_ReadScanData.c`.

## 1. Mode selection

`dwScanMode = 1` (`Gray8`; `2` is the 16-bit variant). The scan parameter
block gets `spImageType = 1` and `spMonoChannel = 1` (constant; "filter 2").

## 2. Scan programming differences (`ScanCtrl_SetScanParameter`)

| Item | Colour | Gray |
|---|---|---|
| `CCDLMT` (reg `0x0C` bits 3:0, step 37) | 2 | **0** — one sensor line per output line |
| `CtlImage_SetColorDepth(bits, channel)` (reg `0x04` bits 7:4) | 24-bit → 0 | 8-bit gray, channel 1 → **Filter 2 → `0x20`**; 16-bit gray → `0x60` |
| Reg `0x60` bit 2 (step 17) | 0 | **1** — single-line mode with three LED pulses inside the line |
| TG period (`fTGTime`, 05 §2.4) | 0.387 / 0.717 / 1.683 / 3.367 ms | **1.161 / 2.151 / 5.049 / 10.101 ms** (300/600/1200/2400; ≈ 3 × colour) |
| Shutter registers `0x8A/0x8D/0x90` (05 §4.4) | raw R, G, B tick counts | **cumulative end times**: `0x8A = R×3/10`, `0x8D = R×3/10 + G×5/10`, `0x90 = that + B×2/10` (integer division after each product) |
| Scan length `0x25` | `3 × lines` | `1 × lines` |
| `Final_TG` (motor speed, 06 §5) | `TG × 3 / gear_per_res` | `TG × 1 / gear_per_res` — same carriage speed as colour because TG is 3× longer |
| `m_ImgLineSize` (raw bytes per line) | `3 × W` (8-bit) | **`W`** (8-bit) / `2W` (16-bit), `W = 3 × seg_pixels` |
| MAXWD (`0x28`) | from `m_ImgLineSize` | same formula with the gray line size |
| Everything else (margins, DPISET, SDRAM banks, §3.3 ladder, AFE gain/offset, Vref) | | identical |

The exposure values `R, G, B` used in the gray encoding are the per-channel
shutters from calibration (§4), i.e. gray is a weighted sum of the three LED
exposures 0.3 R + 0.5 G + 0.2 B within one line period.

## 3. Readout (`07-readout.md` §3.3, §4.2)

One raw line is a **single plane** of `W` samples in the same
segment-interleaved, mirrored order as each colour plane:
`packed[i] = raw[σ(s, k)]`, `σ(s,k) = W − 1 − (3k + s)`. The segment
rotation is the same (segment 0 moved to the end) but the overlap constant is
`+2` bytes = **2 pixels in 8-bit gray, 1 pixel in 16-bit gray**. Crop as in
colour (`dwNewXPos`, `dwXExtent`).

## 4. Calibration (`Scanner_Calibration2`, gray branch)

1. Lamp pulse and the 225/250-step positioning move: as colour.
2. **AFE offset bisection and exposure bisection run in colour mode**
   (`pScan_Parameters->dwScanMode = 0` is forced before both), so they are
   exactly 09 §4–§5 and yield `wOffsetR/G/B` and `fExposureTimeR/G/B`.
3. **White shading frame** in the real mode: `dwScanMode = 1`,
   `dwColorBits = 0x10`, `dwColorDepth = 16`, `dwBytesPerLine = dwXExtent × 2`,
   `dwCISBytesPerLine = dwCISPixelsPerLine × 2`, lines per dpi as colour
   (69 / 81 / 40 or 23 / 23), moving at 464.9 pps etc. (09 §3.2). The raw
   frame is `lines × W` u16 LE samples. Per sample index:
   `Scanner_Calibration_CalcGetShadingDataGray3(buf, white, W, H, V, 1)`
   sorts the `H` values of the column and averages ranks
   `(H − V)·3/4 .. (H − V)·3/4 + V` (`V = dwValidHeight`, the same value the
   colour path passes to `Color3`).
4. **Dark shading frame**: 10 lines, same gray 16-bit still setup;
   `CalcGetShadingDataGray2` = plain mean per sample index.
5. **Upload**: `Scanner_Calibration_WriteShadingToDevice(1, 6, W, …)` →
   `CtlImage_WriteShadingGray(white, dark, W, 6)`: the same pair table as
   colour (`[dark, min(0xFFFF, ShadingBase × 0xF000 / (white − dark))]`,
   data from word 12, 4-word skip at 252 of each 256-word block unless
   `reg 0x06 >> 5 == 2`) written **once, to `shd_bank_address[0]`** (bank
   `0xD0`), then `0x01` bit 1 := 0.
6. Write-back of offsets and exposures to `g_Scan_Parameters` as colour;
   `ShadingData.ShadingGray.{pwWhite,pwDark}Shading` instead of the colour triple.

## 5. Open questions

- Which hardware channel the gray line is taken from ("filter 2", `spMonoChannel = 1`)
  and whether the shading bank 0 is applied to it: to be verified by scanning
  the white strip in gray with and without the upload.
- (resolved) `dwValidHeight = dwYExtent >> 1` for both colour and gray, so the gray rank window is the same as colour's (26..59 of 69 at 300 dpi).
- The 16-bit gray 1-pixel overlap (07 open questions).
