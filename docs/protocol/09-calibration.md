# VF3240 calibration

Status: **verified on hardware (2026-10-13)** for 300, 600 and 2400 dpi colour: offset and exposure bisection converge, white/dark shading and the bank upload produce stripe-free scans. `CalcMaxRGBData` is implemented as the max over sections (the intent), not the vendor's literal last-section value. Positioning must use `ScanCtrl_MoveMotor` (see §2.2 and 06 §9 Q3).

Scope: everything `MyScanner_StartScan` does between the LED bit after
`Api_InitiateASIC3` and `Scanner_SwitchLampPower(1)` — i.e.
`Scanner_CheckCalibration` and `Scanner_Calibration2` with its four phases
(AFE offset, exposure, white shading, dark shading) and the shading upload —
plus the gamma LUT (`Scanner_SetGamma`) that follows. Everything is resolved
for a **300 dpi, 24-bit colour, flatbed** scan; other resolutions are given
where they come from a small table. Gray-mode calibration
(`CalcGetShadingDataGray2/3`, `CtlImage_WriteShadingGray`) is out of scope.

The goal of this document is that a reimplementation can produce the same
AFE offsets, the same shutter times, the same per-pixel `[dark, gain]`
shading table in the same device memory, and the same gamma LUT as the vendor
driver, so that the real scan with `spShading = 1, spGammaMapping = 1`
(`05-scan-parameters.md` §3–§4) returns a shading-corrected image.

Terminology and primitives are those of the earlier documents:

- "reg W", "bits(addr, start, n, v)", "W2/W3", "AFE W" as in `05 §0`.
  AFE W a = d is one `0x83` transfer with payload `[51 a 5D d>>8 5E d&0xFF]`
  (`04-asic-init.md` §3.2).
- "still frame" = `spStillMode = 1` programming of `05 §5.2` / `06 §5.3`.
- raw line, plane order B,R,G, within-plane sample index σ(s,k) = W−1−(3k+s),
  and "packed line" as in `07-readout.md` §3.
- `W = dwCISPixelsPerLine = 3888`, `S = dwCISPixelsPerSeg = 1296` at 300 dpi.
- Rust primitives in `crates/vf3240-core`: `setup::program(ScanSetup{still,
  shading, gamma, depth16, lines, ..})`, `motor::move_motor2`, `scan::start /
  wait_data_ready / read_line / stop / wait_motor_stop`, `lamp::switch`,
  `afe::write_afe_register`, `bulk::write_block`, `image::gamma_lut /
  upload_gamma`. Where a phase needs something those do not yet express (motor
  resolution 600 with fixed 464.9 pps, `spScanBeginLine = 0`, table unit 8,
  the `Api_MoveMotor` unit-2 variant) it is called out.

## Source of truth

Decompiled corpus `re/exports/libviisanvf3240dr.so/decomp/<Function>.c`
(addresses from `functions.txt`, structs from `types.h`); data tables read
from the ELF
`vendor/linux-sane-1.0.2.0/unpacked/usr/local/lib64/sane/aviisandr-adv/VF3240/libviisanvf3240dr.so`
(`nm -S` for symbol → VMA/size, section headers for VMA → file offset,
`python3 -I` to unpack).

| Function | Address | Role |
|---|---|---|
| `MyScanner_StartScan` | `0x11a125` | the caller; fixes the order (§10) |
| `Scanner_CheckCalibration` | `0x11ef08` | file-based vs. live decision (§1) |
| `Scanner_CalibrationFromFile` | `0x11f4c7` | file path — unreachable (§1) |
| `Scanner_Calibration2` | `0x11fe9a` | the live calibration driver (§2) |
| `Scanner_MotorMove` | `0x11ae6c` | `Api_MoveMotor` + `Api_WaitMotorStop` (§2.2) |
| `ScanCtrl_MoveMotor` | `0x1111b8` | unit-2 standalone move (§2.2) |
| `Scanner_Calibration_SetScanParameter` | `0x122583` | still-frame programming (§3.1) |
| `Scanner_Calibration_Shading_SetScanParameter` | `0x122e15` | moving white-shading programming (§3.1) |
| `Scanner_Calibration_StartScan / StopScan` | `0x1236e2 / 0x123706` | `Api_StartScan`; `Api_StopScan` + `Api_WaitMotorStop` (§3.3) |
| `Scanner_Calibration_ReadCISScanData` | `0x123766` | `Api_ReadScanData` (packed) (§3.3) |
| `Scanner_Calibration_ReadCISScanDataWithout3CHPack` | `0x12372f` | `Api_ReadScanDataWithout3CHPack` (raw) (§3.3) |
| `Scanner_Calibration_Offset2` | `0x12098a` | AFE offset bisection (§4) |
| `Scanner_Calibration_CalcAveragePerBigSegment2` | `0x120d4e` | dark-level statistic (§4.3) |
| `CtlAfe_SetAFEOffset` | `0x10c446` | AFE W 5, 6, 7 (§4.5) |
| `Scanner_Calibration_ExposureTime` | `0x120fcb` | shutter bisection (§5) |
| `Scanner_Calibration_CalcMaxRGBData` | `0x121418` | white-level statistic (§5.3) |
| `Scanner_Calibration_WhiteShading` | `0x12168e` | moving white capture (§6) |
| `Scanner_Calibration_CalcGetShadingDataColor3` | `0x121efd` | sorted-window mean (§6.3) |
| `Scanner_Calibration_BubbleSort` | `0x121c05` | ascending sort (§6.3) |
| `Scanner_Calibration_DarkShading` | `0x1219bc` | still dark capture (§7) |
| `Scanner_Calibration_CalcGetShadingDataColor2` | `0x121cd4` | plain mean (§7.2) |
| `Scanner_Calibration_WriteShadingToDevice` | `0x122503` | channel → bank dispatch (§8) |
| `CtlImage_WriteShadingColor` | `0x10e928` | pair table build + bulk upload (§8) |
| `CtlImage_SetDefaultShading` | `0x10d642` | **no callers**; same encoding, offset 2 (§8.5) |
| `Scanner_SetGamma` / `CtlImage_SetGamma` | `0x11aeb3 / 0x10f13a` | gamma LUT (§9) |
| `AFEOffsetValue` | `.data 0x22d6e0`, 1024 bytes | 512 × u16 offset table (§4.4) |
| `g_szKDataPath` | `.bss 0x22dbe0`, 40 bytes | calibration file directory (§1) |
| `shd_bank_address` | `.bss 0x22e740` | bank bulk addresses (`04 §4.7`) |
| `CtlImage_ShadingBase` | `.bss 0x22e76c` | 0x4000 (`04 §4.7`) |

---

## 0. Summary of the sequence

```
MyScanner_StartScan                                     (300 dpi colour)
  CCD_DPIHW = 1200; ScanTBunit = 8                       <- note: unit 8 during calibration
  Api_OpenDevice2; Api_InitiateASIC3(300, 300)           04-asic-init.md
  bits(0x3A, bit2, 1) = 1                                scan LED
  Scanner_CheckCalibration()  -> 0 in practice           §1
  Scanner_Calibration2()                                  §2
    bucket dpi -> 300; StartPixelOffset 0x10B; XExtent 3596
    lamp on, sleep1(100), lamp off                        §2.1
    Scanner_MotorMove(800.0, 0xE1 = 225, dir 1)           §2.2  carriage onto the white strip
    Scanner_Calibration_Offset2        lamp off, still, 8-bit, 1 line, ≤15 frames   §4
    Scanner_Calibration_ExposureTime   lamp on,  still, 8-bit, 1 line, ≤15 frames   §5
    Scanner_Calibration_WhiteShading   lamp on,  moving 464.9 pps, 16-bit, 69 lines §6
    Scanner_Calibration_DarkShading    lamp off, still, 16-bit, 10 lines            §7
    g_Scan_Parameters.Afe_Info.wOffsetR/G/B, fExposureTimeR/G/B, ShadingData <- results
    Scanner_Calibration_WriteShadingToDevice(0, 6, 3888, ShadingData)             §8
  CCD_DPIHW = 1200; ScanTBunit = 4                        (recomputed, 05 §3.1)
  Scanner_SwitchLampPower(1)                              06 §7.5
  Scanner_SetGamma(0x10)                                  §9
  Scanner_SetScanParameter(0, 0, 1, &g_Scan_Parameters)   05 §3–4, with the calibrated values
  Api_StartScan
```

No home move happens between calibration and the real scan (§10).

---

## 1. `Scanner_CheckCalibration` — file-based calibration is dead

`Scanner_CheckCalibration.c`: copies `g_Scan_Parameters` (0xC0 bytes) and
buckets `dwResolution` (the fourth word of the copy):

| `dwResolution` | bucket |
|---|---|
| < 301 (200, 300) | **300** |
| < 601 | 600 |
| < 1201 | 1200 |
| else | 2400 |

For colour (`dwScanMode == 0`) it then `fopen(…, "rb")`s seven files and
returns 1 only if **all** exist:

```
"%s%s_%dC.ini"  g_szKDataPath, "XT5750HS_AFE",        bucket     -> XT5750HS_AFE_300C.ini
"%s%s_%dC.dat"  g_szKDataPath, "XT5750HS_SHADING_WR", bucket     -> XT5750HS_SHADING_WR_300C.dat
                                "XT5750HS_SHADING_WG"             (and WB, DR, DG, DB)
```

(gray: `XT5750HS_AFE_%dG.ini`, `XT5750HS_SHADING_W_%dG.dat`,
`XT5750HS_SHADING_D_%dG.dat`.) Note the format has **no separator** between
the directory and the file name.

Why it is effectively dead:

1. `g_szKDataPath` lives in `.bss` (40 bytes, `0x22dbe0`) → empty string at
   load. The only writes anywhere in the corpus are
   `strncpy(g_szKDataPath, "/tmp", 5)` in `Scanner_Calibration_WhiteShading.c`
   and `Scanner_Calibration_DarkShading.c` (after their captures). So the
   first scan of a process looks for `XT5750HS_AFE_300C.ini` in the
   current working directory, every later scan for `/tmpXT5750HS_AFE_300C.ini`.
2. Nothing ever writes those files: the only `fopen` write modes in the
   corpus are in `MyScanner_SetPowerSaverStatus` / `Scanner_ThreadStart`
   (grep of `decomp/*.c` for `"wb"` / `"w"`).
3. `MyScanner_StartScan_Calibration2File` (an alternative entry that would
   write them) is not referenced by the SANE shim, which only calls
   `StartScan1` (24 references, no other `StartScan*` symbol in
   `re/exports/libsane-aviisandrvf3240.so.1.0.27/decomp`).

Hence `g_isSmartCalibration = 0` and `Scanner_Calibration2` runs on
**every** scan. `Scanner_CalibrationFromFile` need not be implemented. (A
reimplementation may of course cache results itself.)

---

## 2. `Scanner_Calibration2` — driver

`Scanner_Calibration2.c`. Works on a **private copy** `p` of
`g_Scan_Parameters`; only the fields listed in §2.4 are written back.

### 2.1 Setup of the copy and the lamp pulse

```
p = copy of g_Scan_Parameters
p.dwResolution = bucket(p.dwResolution)           // same ladder as §1 → 300
per bucket:  dwStartPixelOffset / dwCISPixelsPerLine / dwCISPixelsPerSeg
   2400: 0x858 / 0x7980 / 0x2880
   1200: 0x42C / 0x3CC0 / 0x1440
    600: 0x216 / 0x1E60 / 0x0A20
    300: 0x10B / 0x0F30 / 0x0510          <- 267 / 3888 / 1296
p.dwNewXPos = p.dwStartPixelOffset                 // 267; dwXPos is ignored
Scanner_SwitchLampPower(1); sleep1(100); Scanner_SwitchLampPower(0)
```

`p.dwXResolution / dwYResolution` keep the real values (300). The lamp pulse
is reg 3 bit5 = 0, bit4 = 0, bit4 = 1 (`06 §7.2`), ~100 ms busy-wait, then
bit4 = 0 (`Scanner_SwitchLampPower` checks `0x101` bit2 first, `06 §7.5`).

### 2.2 Positioning move: `Scanner_MotorMove(800.0, 0xE1, 1)`

300 dpi uses `0xE1 = 225` steps; every other bucket `0xFA = 250`. Direction 1
= away from home. `Scanner_MotorMove` = `Api_MoveMotor` (→
`ScanCtrl_MoveMotor`, **not** `MoveMotor2`) followed by `Api_WaitMotorStop`
(`CtlMotor_WaitMotorStop`: poll reg `0x101` bit0 until 0, 60 s budget).

`ScanCtrl_MoveMotor(fpps, Steps, iDir)` is `06 §4.1` steps 3–10 with
`unit = 2`, Vref argument 1 (dead), **no** `0x37/0x38` bit-4 writes and no
short-move fix-up:

| Step | Wire | Value (800 pps, 225 steps) |
|---|---|---|
| `CtlSensor_SetTBTime(1)` | bits(0x1C, 2:0) = 0 | `nTBtime = 1` |
| `CtlMotor_SetMotorDirect(1)` | bits(0x02, bit2) = 0 | away from home |
| `LoadPPSCurveToFastTable(CurveData2, 510, 800.0, 2, 1)` | `0x38[5]=1, 0x37[5]=0, 0xA0[3:3]=2`; `0x3B = 0x50`, `0x3B = 0`, `0x3C = 0`; bulk **48 words** (96 bytes) to `0x1000C000`, W2(0xAE, 48); same to `0x10010000`, W2(0xB0, 48) | `word = 4e6/(pps·1·2)`: `[65535, 65535, 48780, 33333, 25641, 20833, …, 2528, 2500]` (47 curve entries < 800 → 47 + target = 48, already a multiple of 4) |
| `CtlMotor_EnableFastMove(1)` | bits(0x02, bit3) = 1 | |
| `CtlMotor_SetFeedSteps(2·225 − 2·48 = 354)` | W3(0x3D, 354) | `0x3D..3F = 00 01 62` |
| `CtlMotor_SetMotorPowerBit(1)` | bits(0x02, bit4) = 1 | |
| `CtlMotor_SetMotorStartMove()` | reg W 0x0F = 0xFF | |

Total travel = 354 + 2 × 48 = 450 unit-steps = **225 full steps** (at
`MotorGear` 600 steps/in: 0.375 in ≈ 9.5 mm from home). Hardware note
(2026-10-13): this position sees the white strip. In Rust this is
`move_motor2(regs, 800.0, 225, Away, 2)` minus the two `0x37/0x38` writes;
the `0x37/0x38` difference is not expected to matter but has not been tested.

### 2.3 Phase parameters (fields of `p` before each phase)

| Field | Offset2 (§4) | ExposureTime (§5) | WhiteShading (§6) | DarkShading (§7) |
|---|---|---|---|---|
| `dwScanMode` | 0 | 0 | `g.dwScanMode` (0) | 0 |
| `dwColorBits` | 24 | 24 | 48 | 48 |
| `dwColorDepth` | 8 | 8 | **16** | **16** |
| `dwXExtent` | 3596 | 3596 | 3596 | 3596 |
| `dwYExtent` | 1 | 1 | **69** (0x45) | **10** |
| `dwBytesPerLine` | 10788 | 10788 | 21576 | 21576 |
| `dwCISBytesPerLine` | 11664 | 11664 | 23328 | 23328 |
| AFE | offsets/idx reset from `g` (0x80 / 0x7F) | offsets as found by §4 | as found | as found |
| exposure | from `g` (0.35/0.33/0.16) | reset from `g` | as found by §5 | as found |

`dwXExtent = dwResolution × 0xE0C / 300` (integer) = 3596 at 300, 7192 at
600, 14384 at 1200, 28768 at 2400. White-shading line counts per bucket:
300 → 0x45 = 69; 600 → 0x51 = 81; 1200 → 0x28 = 40 if `dwYResolution ==
1200`, 0x17 = 23 if 2400; 2400 → 0x17 = 23. Dark shading is always 10 lines.

### 2.4 Write-back to `g_Scan_Parameters` (tail of `Scanner_Calibration2.c`)

Only after all four phases succeed:

```
g.Afe_Info.wOffsetR/G/B      = p.Afe_Info.wOffsetR/G/B        // AFEOffsetValue[idx], §4
g.Afe_Info.fExposureTimeR/G/B = p.Afe_Info.fExposureTimeR/G/B // §5
g.ShadingData                 = p.ShadingData (6 pointers)      // colour
Scanner_Calibration_WriteShadingToDevice(0, 6, p.dwCISPixelsPerLine = 3888, p.ShadingData)
```

**Not** written back: `wOffsetIdxR/G/B`, `wGainR/G/B` (stay 0x29),
`fTGTime`/`fMaxTGTime` (0.387), and all the geometry changes of §2.1
(`dwStartPixelOffset` stays 0x100, `dwNewXPos` = `dwXPos + 0x100`,
`dwXExtent` = 3593 for the real scan). So the real scan's
`Scanner_SetScanParameter` (`05 §3.3` steps 11–12 and `§4.2` step 18) writes
AFE 2/3/4 = 0x29, **AFE 5/6/7 = the calibrated offsets** and
**0x8A/0x8D/0x90 = the calibrated shutter times** in ticks
(`trunc(ms_f32 × 1000 / 0.25)`).

Failure of any phase returns 0 → `MyScanner_StartScan` returns 0 (and the
copy leaks; irrelevant).

---

## 3. Frame programming and readout helpers

### 3.1 `Scanner_Calibration_SetScanParameter(still, autohome, HWshading, p)` and `_Shading_` variant

Both build a `ScanParameter` (`05 §3.2` layout) and call
`Api_SetScanParameter`, then do their own register writes. Dispatch is on
`p.dwResolution` (the bucket, so 300 for a 200 dpi scan). Differences from
the real-scan struct of `05 §3.2` are in bold:

| Field | `_SetScanParameter` (still phases) | `_Shading_SetScanParameter` (white) | real scan (`05 §3.2`) |
|---|---|---|---|
| `spScanBeginPixel` | 0 | 0 | 0 |
| `spScanBeginLine` | **0** | **0** | 810 |
| `spScanLinePixels` | 1296 | 1296 | 1296 |
| `spTotalScanLines` | `p.dwYExtent` (1 / 10) | 69 | 5040 |
| `spPixelResolution` | 300 | 300 | 300 |
| `spMotorResolution` | 300 | **600 (every dpi)** | 300 |
| `spImageType` | 0 | 0 | 0 |
| `spMonoChannel` | 1 | 1 | 1 |
| `spPixelDepth` | `p.dwColorDepth`: 8 (§4, §5), **16** (§7) | **16** | 8 |
| `spExposureTime` | 0.387 | 0.387 | 0.387 |
| `spShutterTime[0..2]` | `p.fExposureTimeR/G/B` (current bisection value) | calibrated | calibrated |
| `spTimingIdx` | 4 (300/600/200/100), 2 (1200), 1 (2400) | same | 4 |
| `spDummyLines`, `spRepeatParams`, `spDisBackTrack`, `spSWCCDPacking`, `spADFscan` | 0 | 0 | 0 |
| `spStillMode` | **1** (arg) | 0 | 0 |
| `spAutoGoHome` | 0 | 0 | 0 |
| `spShading` | **0** (arg) | **0** | 1 |
| `spGammaMapping` | **0** | **0** | 1 |
| `spMotorFixedSpeed` | 0 | **1** → `Final_PPS = 464.9` | 0 |

Register writes after `Api_SetScanParameter` returns (both variants, 300
branch = the `else` of an `if 2400 / 1200 / 600` ladder): exactly `05 §3.3`
steps 1–10 for 300 dpi (`0x35[0]=1, 0x36[4]=1, 0x33[4]=0, 0x4F[3]=1, 0x68=00,
0x69=6D, 0x13[7:5]=1, 0x12[7:5]=7, 0x1E[3]=1, 0x19[7]=1`), then
`Api_SetAFEGain(p.wGainR, G, B)` = AFE W 2/3/4 = 0x0029 and
`Api_SetAFEOffset(p.wOffsetR, G, B)` = **AFE W 5/6/7 = the current offset
values** (§4.5). Then:

- `_SetScanParameter` (still): **no Vref writes.** Reg `0x3B` stays 0 (from
  `WriteColorTiming`, `05 §5.2` caveat); `0x3C` stays at whatever the last
  table load left (0 after §2.2).
- `_Shading_SetScanParameter`: `CtlMotor_SetMotorVref(0,0,2,2)` → reg W
  `0x3B = 0x0A`; `CtlMotor_SetMotorVref_2(0,0,2,2)` → `0x3C = 0x0A`.

### 3.2 What `ScanCtrl_SetScanParameter` then programs for each frame type

Relative to the 300 dpi colour table of `05 §4.2` (8-bit, moving, 5040
lines) the three frame types differ only in these registers/values. The
still-mode substitutions are `05 §5.2` (flat 4-word slope table of
`1547 = 0x060B` to `0x10000000/0x10004000/0x10008000`, `0xA4 = 0xAA = 0xAC =
4`, `0x3D = 1`, `0x02[3] = 0`, `0x02[4] = 0`, `0xB2 = 0xB5 = 0`).

| Register / value | still 8-bit, 1 line (§4, §5) | moving 16-bit, 69 lines (§6) | still 16-bit, 10 lines (§7) |
|---|---|---|---|
| host `m_16bitscanflg` / `iColorBits` / `m_ImgLineSize` | 0 / 24 / **11664** | 1 / 48 / **23328** | 1 / 48 / **23328** |
| reg `0x04` bits 7:4 (`CtlImage_SetColorDepth(iColorBits, 1)`) | 0 → `0x04 = 0x02` | **BitSet** → `0x04 = 0x42` | `0x42` |
| `0x25..27` scan length `= 3 × lines` | 3 (`00 00 03`) | 207 (`00 00 CF`) | 30 (`00 00 1E`) |
| `0x28..2A` MAXWD (`05 §4.5`, `dwImgLen = m_ImgLineSize`) | 33953 (`00 84 A1`) | **35411 (`00 8A 53`)** | 35411 |
| `0x8A/0x8D/0x90` shutter | current bisection values (§5) | calibrated | calibrated |
| `0x7D..7F` TG | 1547 | 1547 | 1547 |
| motor block | still (`05 §5.2`) | see below | still |
| `0x02` final | 0x00 | **0x10** (power on, fast off, dir away, backtrack on) | 0x00 |
| `0x05` bit3 (gamma), `0x01` bit5 (shading) | 0, 0 (reg 6 untouched = 0x10) | 0, 0 | 0, 0 |
| `sleep1(trunc(3 × 0.387) = 1)` | yes | yes | yes |

Everything else (timing table, margins 0x82/0x85, DPISET, SDRAM banks
0xE0..0xF8, assorted bits, `ClrCounter`) is identical to `05 §4.2`.

**Motor programming of the white-shading frame** (`spMotorResolution = 600`,
`spMotorFixedSpeed = 1`, `ScanTBunit = 8` because calibration runs before
`MyScanner_StartScan` recomputes it, `05 §3.1`):

| Step (`06 §5.2` numbering) | Value |
|---|---|
| 7 `SetMotorVref(1,1,1,1)` | `0x3B = 0x55` (dead) |
| 8 `SetScanTableUnit(8)` | `0x38[5]=1, 0x37[5]=0, 0xA0[0:3] = 3`; `m_iScanTableUnit = 8` |
| 9 `SetFastTableUnit(2)` | `0xA0[3:3] = 2`; `m_iFastTableUnit = 2` |
| 10 `Final_TG = trunc(1 × 1547 / (600/600)) × 3` | **4641** |
| 11 `CalculateTBTIME(4641)` | tb = 1 → `0x1C[2:0] = 0` |
| 12 `Final_PPS` | **464.9** (fixed; = the 600 dpi colour scan speed, `06 §5.4`) |
| 13 `LoadPPSCurveToScanTable(CurveData2, 510, 464.9, 8, 1)` | `word = 4e6/(pps·1·8) = 500000/pps`: **28 words** `[65535, 21739, 12195, 8333, …, 1075, 1075, 1075, 1075]` (25 curve entries < 464.9, + target 1075, padded to 28); bulk 56 bytes to `0x10000000`, `0xA4 = 0x001C`; same to `0x10004000`/`0xAA`, `0x10008000`/`0xAC`; `m_iTableSum = 162063`, `m_iTableHigh = 1075`; `0x3B = 0`, `0x3C = 0` |
| 14 `LoadPPSCurveToFastTable(…, 1500.0, 2, 1)` | 184 words as in `06 §5.2` step 14 |
| 15 `CalculateFeedReg(trunc(0 / 2) = 0)` | `0xA2:A3 = 0x0008`; `tmpline = 0 − 2·92 − 28/8 − 1 < 1` → `max(0 − 3, 0) = 0` → **feed 0**: `0x3D..3F = 00 00 00`, `0x02[3] = 0` (no fast move) |
| 16 `SetMotorPowerBit(1)` | `0x02[4] = 1` |
| 17 `SetZ1Z2Value` | `b_value = read20(0x3D) = 0`; `Z1 = (162063 + 239 × 1075) % 4641 = 1298` → `0xB2..B4 = 00 05 12`; `Z2 = (162063 + (0 − 1)·1075) % 4641` in **uint32 arithmetic** = `(162063 − 1075) % 4641 = 3194` → `0xB5..B7 = 00 0C 7A` |
| 19 `EnableAutoGoHome(0)`, 21 `SetMotorDirect(1)` | `0x02[5] = 0`, `0x02[2] = 0` |

So the white-shading scan **starts where the carriage is** (no feed) and
runs the scan ramp up to 464.9 pps while the ASIC captures 207 sub-lines.
Nominal travel: 69 lines × 1.161 ms = 80 ms × 464.9 pps ≈ **37 full steps
(≈ 1.6 mm)** if the motor free-runs at the table speed, or 69 steps (2.9 mm)
if the ASIC advances one step per line at motor resolution 600 — open
question (§11). `Scanner_Calibration_StopScan` waits for the motor, so the
carriage is stationary before the dark frame.

In Rust terms: `setup::program` needs three additions for this frame —
`motor_resolution` (600) in the `Final_TG` formula, a `fixed_pps:
Option<f32>` (464.9) overriding `ticks_to_pps`, `begin_line = 0` (then
`CalculateFeedReg` yields feed 0 / fast off, and Z2 must use wrapping u32
arithmetic — it already does), and `scan_unit = 8`.

### 3.3 Start, read, stop

- `Scanner_Calibration_StartScan` = `Api_StartScan` = `ScanCtrl_StartScan`:
  `0x01[0] = 1`, `0x0F = 0xFF`, then `ScanCtrl_ChkScanDataReady` poll
  (`07 §1`). In still mode `0x0F = 0xFF` with `0x02[4] = 0` does not move the
  carriage (hardware-confirmed for still frames).
- Reads: **all `dwYExtent` lines in one call**, `Api_Read…(buf, dwYExtent)`
  → one `CMDASIC_ReadRawImage(buf, m_ImgLineSize × lines)` (`07 §2.1`):
  11664 bytes (§4, §5), **1 609 632 bytes** (69 × 23328, §6), 233 280 bytes
  (§7) in a single bulk IN request. No data-ready poll inside the read.
  - `ReadCISScanDataWithout3CHPack` (§4, §6, §7): raw plane order, B,R,G
    planes of W samples, σ order (`07 §3`).
  - `ReadCISScanData` (§5 only): `ScanCtrl_ReadScanData` = the same read
    followed by `ScanCtrl_Pack_3CH_CIS_Color` per line → **packed line**,
    pixel `i = s·S + k`, bytes `R, G, B` (`07 §3.3`). No segment rotation and
    no crop (those are in `Scanner_ReadScanData`, the SANE layer).
- `Scanner_Calibration_StopScan` = `Api_StopScan` (`0x01[0] = 0`) +
  `Api_WaitMotorStop` (poll `0x101` bit0 until clear, 60 s). No homing.

The crop constants `dwNewXPos = 0x10B = 267` and `dwXExtent = 3596` are only
consumed by the exposure statistic (§5.3): `[267, 3863)` in packed pixel
index. The offset and shading statistics operate on the full raw line.

---

## 4. Offset calibration — `Scanner_Calibration_Offset2`

### 4.1 Frame

Lamp off (`Scanner_SwitchLampPower(0)` — a no-op if `0x101` bit2 already
reads 0), still, 8-bit colour, 1 line, raw read (§3.2 column 1). Buffer =
`dwYExtent × dwCISBytesPerLine` = 11664 bytes.

### 4.2 Loop (per channel c ∈ {R, G, B}, independent state)

```
dwAdjMaxNum = 15;  nLowBound = 6;  nUpBound = 7
wMin[c] = 0x1FF ; wMax[c] = 0 ; found[c] = 0          // "Min"/"Max" are the vendor's names: Min = index that read too DARK, Max = index that read too BRIGHT
idx[c]  = p.wOffsetIdx[c] = 0x7F ; val[c] = p.wOffset[c] = 0x80   // from MyScanner_SetParameter2 defaults
for pass in 0..15:
    lamp off; Scanner_Calibration_SetScanParameter(1, 0, 0, p)    // writes AFE 5/6/7 = val[R], val[G], val[B]
    StartScan; read 1 raw line; StopScan
    (d0, d1, d2) = CalcAveragePerBigSegment2(buf, 3888, 1, 11664)
    for (c, d) in ((R, d0), (G, d1), (B, d2)):
        if found[c]: continue
        if d > 7:   wMax[c] = idx[c]; idx[c] = (wMin[c] + wMax[c]) / 2; val[c] = AFEOffsetValue[idx[c]]
        elif d < 6: wMin[c] = idx[c]; idx[c] = (wMin[c] + wMax[c]) / 2; val[c] = AFEOffsetValue[idx[c]]
        else:       found[c] = 1
    if all found: break
return 1   // always; non-convergence keeps the last val[c] (no error, no log)
```

Integer division, `WORD` arithmetic. Target window is **[6, 7] inclusive** on
the 8-bit dark average. First-pass outcomes: too bright → `idx = (0x1FF +
0x7F) / 2 = 0x13F`, `val = 0x13F`; too dark → `idx = (0x7F + 0) / 2 = 0x3F`,
`val = 0xC0`. Because `wMin` only moves down and `wMax` only up, the index
walks monotonically; after 15 passes the bracket is at most
`0x1FF / 2^15` wide, i.e. it always converges if the response is monotonic.

### 4.3 Statistic — `CalcAveragePerBigSegment2(buf, W, H, bpl, &d0, &d1, &d2)`

`dwHeight` is ignored (only line 0 is read). With `S = W / 3 = 1296`:

```
for i in trunc(S × 0.3) = 388 .. trunc(S × 0.7) = 907 (exclusive):     // 519 samples
    sum0 += buf[2W + 3i + 2]
    sum1 += buf[2W + 3i + 0]
    sum2 += buf[2W + 3i + 1]
d0 = (sum0 / 519) & 0xFF ; d1 = (sum1 / 519) & 0xFF ; d2 = (sum2 / 519) & 0xFF
```

(`*pData` is initialised to 0xFF and replaced by `min(0xFF, avg)`, which is
just `avg`.) All three sums come from the **third plane (byte offset 2W =
7776) = the G plane** of the raw line, at sample indices σ = 3i + 2, 3i, 3i +
1. By `07 §3.2` (`σ ≡ 2 (mod 3)` ⇒ segment 0, `σ ≡ 0` ⇒ segment 2, `σ ≡ 1`
⇒ segment 1, pixel `k = 1295 − i` ∈ [389, 907]):

| Output | drives | raw samples | = CIS segment, pixels |
|---|---|---|---|
| `d0` | `wOffsetR` → AFE reg 5 | G plane, σ = 3i+2 | **segment 0**, k ∈ [389, 907] |
| `d1` | `wOffsetG` → AFE reg 6 | G plane, σ = 3i | **segment 2**, same k |
| `d2` | `wOffsetB` → AFE reg 7 | G plane, σ = 3i+1 | **segment 1**, same k |

The function name says what it is: a per-**segment** dark average (the
middle 40 % of each CIS chip, one colour plane — with the lamp off the plane
does not matter). The three "R/G/B" AFE offset registers are therefore tuned
on three *segments*, which strongly suggests the AFE's three channels
digitise the three CIS segments (one sample per segment per pixel clock,
`07 §3.2`), i.e. AFE 5 ↔ segment 0, AFE 6 ↔ segment 2, AFE 7 ↔ segment 1,
and "gain R/G/B" (AFE 2/3/4) are per-segment gains too. This is inferred from
the loop pairing only; verify on hardware by changing AFE 5 alone and
observing which segment's black level moves (§11).

### 4.4 `AFEOffsetValue[512]` (`.data` `0x22d6e0`, u16 little-endian)

The table is `AFEOffsetValue[i] = 0xFF − i` for `i < 256` and `= i` for
`256 ≤ i < 512`, i.e. a **9-bit sign-magnitude** sweep: index 0..255 = sign 0
with magnitude 255 → 0, index 256..511 = sign 1 (bit 8) with magnitude 0 →
255. Lower index = brighter dark level (the bisection assumes this). Full
dump (16 per row, row label = first index):

| idx | +0 | +1 | +2 | +3 | +4 | +5 | +6 | +7 | +8 | +9 | +10 | +11 | +12 | +13 | +14 | +15 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 0x0FF | 0x0FE | 0x0FD | 0x0FC | 0x0FB | 0x0FA | 0x0F9 | 0x0F8 | 0x0F7 | 0x0F6 | 0x0F5 | 0x0F4 | 0x0F3 | 0x0F2 | 0x0F1 | 0x0F0 |
| 16 | 0x0EF | 0x0EE | 0x0ED | 0x0EC | 0x0EB | 0x0EA | 0x0E9 | 0x0E8 | 0x0E7 | 0x0E6 | 0x0E5 | 0x0E4 | 0x0E3 | 0x0E2 | 0x0E1 | 0x0E0 |
| 32 | 0x0DF | 0x0DE | 0x0DD | 0x0DC | 0x0DB | 0x0DA | 0x0D9 | 0x0D8 | 0x0D7 | 0x0D6 | 0x0D5 | 0x0D4 | 0x0D3 | 0x0D2 | 0x0D1 | 0x0D0 |
| 48 | 0x0CF | 0x0CE | 0x0CD | 0x0CC | 0x0CB | 0x0CA | 0x0C9 | 0x0C8 | 0x0C7 | 0x0C6 | 0x0C5 | 0x0C4 | 0x0C3 | 0x0C2 | 0x0C1 | 0x0C0 |
| 64 | 0x0BF | 0x0BE | 0x0BD | 0x0BC | 0x0BB | 0x0BA | 0x0B9 | 0x0B8 | 0x0B7 | 0x0B6 | 0x0B5 | 0x0B4 | 0x0B3 | 0x0B2 | 0x0B1 | 0x0B0 |
| 80 | 0x0AF | 0x0AE | 0x0AD | 0x0AC | 0x0AB | 0x0AA | 0x0A9 | 0x0A8 | 0x0A7 | 0x0A6 | 0x0A5 | 0x0A4 | 0x0A3 | 0x0A2 | 0x0A1 | 0x0A0 |
| 96 | 0x09F | 0x09E | 0x09D | 0x09C | 0x09B | 0x09A | 0x099 | 0x098 | 0x097 | 0x096 | 0x095 | 0x094 | 0x093 | 0x092 | 0x091 | 0x090 |
| 112 | 0x08F | 0x08E | 0x08D | 0x08C | 0x08B | 0x08A | 0x089 | 0x088 | 0x087 | 0x086 | 0x085 | 0x084 | 0x083 | 0x082 | 0x081 | 0x080 |
| 128 | 0x07F | 0x07E | 0x07D | 0x07C | 0x07B | 0x07A | 0x079 | 0x078 | 0x077 | 0x076 | 0x075 | 0x074 | 0x073 | 0x072 | 0x071 | 0x070 |
| 144 | 0x06F | 0x06E | 0x06D | 0x06C | 0x06B | 0x06A | 0x069 | 0x068 | 0x067 | 0x066 | 0x065 | 0x064 | 0x063 | 0x062 | 0x061 | 0x060 |
| 160 | 0x05F | 0x05E | 0x05D | 0x05C | 0x05B | 0x05A | 0x059 | 0x058 | 0x057 | 0x056 | 0x055 | 0x054 | 0x053 | 0x052 | 0x051 | 0x050 |
| 176 | 0x04F | 0x04E | 0x04D | 0x04C | 0x04B | 0x04A | 0x049 | 0x048 | 0x047 | 0x046 | 0x045 | 0x044 | 0x043 | 0x042 | 0x041 | 0x040 |
| 192 | 0x03F | 0x03E | 0x03D | 0x03C | 0x03B | 0x03A | 0x039 | 0x038 | 0x037 | 0x036 | 0x035 | 0x034 | 0x033 | 0x032 | 0x031 | 0x030 |
| 208 | 0x02F | 0x02E | 0x02D | 0x02C | 0x02B | 0x02A | 0x029 | 0x028 | 0x027 | 0x026 | 0x025 | 0x024 | 0x023 | 0x022 | 0x021 | 0x020 |
| 224 | 0x01F | 0x01E | 0x01D | 0x01C | 0x01B | 0x01A | 0x019 | 0x018 | 0x017 | 0x016 | 0x015 | 0x014 | 0x013 | 0x012 | 0x011 | 0x010 |
| 240 | 0x00F | 0x00E | 0x00D | 0x00C | 0x00B | 0x00A | 0x009 | 0x008 | 0x007 | 0x006 | 0x005 | 0x004 | 0x003 | 0x002 | 0x001 | 0x000 |
| 256 | 0x100 | 0x101 | 0x102 | 0x103 | 0x104 | 0x105 | 0x106 | 0x107 | 0x108 | 0x109 | 0x10A | 0x10B | 0x10C | 0x10D | 0x10E | 0x10F |
| 272 | 0x110 | 0x111 | 0x112 | 0x113 | 0x114 | 0x115 | 0x116 | 0x117 | 0x118 | 0x119 | 0x11A | 0x11B | 0x11C | 0x11D | 0x11E | 0x11F |
| 288 | 0x120 | 0x121 | 0x122 | 0x123 | 0x124 | 0x125 | 0x126 | 0x127 | 0x128 | 0x129 | 0x12A | 0x12B | 0x12C | 0x12D | 0x12E | 0x12F |
| 304 | 0x130 | 0x131 | 0x132 | 0x133 | 0x134 | 0x135 | 0x136 | 0x137 | 0x138 | 0x139 | 0x13A | 0x13B | 0x13C | 0x13D | 0x13E | 0x13F |
| 320 | 0x140 | 0x141 | 0x142 | 0x143 | 0x144 | 0x145 | 0x146 | 0x147 | 0x148 | 0x149 | 0x14A | 0x14B | 0x14C | 0x14D | 0x14E | 0x14F |
| 336 | 0x150 | 0x151 | 0x152 | 0x153 | 0x154 | 0x155 | 0x156 | 0x157 | 0x158 | 0x159 | 0x15A | 0x15B | 0x15C | 0x15D | 0x15E | 0x15F |
| 352 | 0x160 | 0x161 | 0x162 | 0x163 | 0x164 | 0x165 | 0x166 | 0x167 | 0x168 | 0x169 | 0x16A | 0x16B | 0x16C | 0x16D | 0x16E | 0x16F |
| 368 | 0x170 | 0x171 | 0x172 | 0x173 | 0x174 | 0x175 | 0x176 | 0x177 | 0x178 | 0x179 | 0x17A | 0x17B | 0x17C | 0x17D | 0x17E | 0x17F |
| 384 | 0x180 | 0x181 | 0x182 | 0x183 | 0x184 | 0x185 | 0x186 | 0x187 | 0x188 | 0x189 | 0x18A | 0x18B | 0x18C | 0x18D | 0x18E | 0x18F |
| 400 | 0x190 | 0x191 | 0x192 | 0x193 | 0x194 | 0x195 | 0x196 | 0x197 | 0x198 | 0x199 | 0x19A | 0x19B | 0x19C | 0x19D | 0x19E | 0x19F |
| 416 | 0x1A0 | 0x1A1 | 0x1A2 | 0x1A3 | 0x1A4 | 0x1A5 | 0x1A6 | 0x1A7 | 0x1A8 | 0x1A9 | 0x1AA | 0x1AB | 0x1AC | 0x1AD | 0x1AE | 0x1AF |
| 432 | 0x1B0 | 0x1B1 | 0x1B2 | 0x1B3 | 0x1B4 | 0x1B5 | 0x1B6 | 0x1B7 | 0x1B8 | 0x1B9 | 0x1BA | 0x1BB | 0x1BC | 0x1BD | 0x1BE | 0x1BF |
| 448 | 0x1C0 | 0x1C1 | 0x1C2 | 0x1C3 | 0x1C4 | 0x1C5 | 0x1C6 | 0x1C7 | 0x1C8 | 0x1C9 | 0x1CA | 0x1CB | 0x1CC | 0x1CD | 0x1CE | 0x1CF |
| 464 | 0x1D0 | 0x1D1 | 0x1D2 | 0x1D3 | 0x1D4 | 0x1D5 | 0x1D6 | 0x1D7 | 0x1D8 | 0x1D9 | 0x1DA | 0x1DB | 0x1DC | 0x1DD | 0x1DE | 0x1DF |
| 480 | 0x1E0 | 0x1E1 | 0x1E2 | 0x1E3 | 0x1E4 | 0x1E5 | 0x1E6 | 0x1E7 | 0x1E8 | 0x1E9 | 0x1EA | 0x1EB | 0x1EC | 0x1ED | 0x1EE | 0x1EF |
| 496 | 0x1F0 | 0x1F1 | 0x1F2 | 0x1F3 | 0x1F4 | 0x1F5 | 0x1F6 | 0x1F7 | 0x1F8 | 0x1F9 | 0x1FA | 0x1FB | 0x1FC | 0x1FD | 0x1FE | 0x1FF |

A reimplementation can use the closed form instead of the table. The
default index 0x7F → value 0x80 (sign 0, magnitude 128).

### 4.5 What is written to the AFE

`Api_SetAFEOffset(wOffsetR, wOffsetG, wOffsetB)` → `CtlAfe_SetAFEOffset` →
`CtlAfe_WriteAFERegister(5, R)`, `(6, G)`, `(7, B)` — the **table value**
(9-bit), not the index, one `0x83` transfer each:

| Value | Wire payload (AFE 5; 6/7 analogous) |
|---|---|
| 0x080 (default) | `51 05 5D 00 5E 80` |
| 0x13F (first "too bright" step) | `51 05 5D 01 5E 3F` |
| 0x0C0 (first "too dark" step) | `51 05 5D 00 5E C0` |

The writes happen inside `Scanner_Calibration_SetScanParameter` on every
pass (after `WriteColorTiming` has written the AFE row `68 0A 2F 2F 2F 00 00
00`, so the order within a pass is AFE 0..7 from the timing row, then AFE
2/3/4 = 0x29, then AFE 5/6/7 = current offsets).

---

## 5. Exposure calibration — `Scanner_Calibration_ExposureTime`

### 5.1 Frame

Lamp on, still, 8-bit colour, 1 line, **packed** read
(`Scanner_Calibration_ReadCISScanData`). AFE offsets = the §4 result.
Shutter registers per pass from `p.fExposureTimeR/G/B` via the normal
`ScanCtrl_SetScanParameter` step 18: `0x8A/0x8D/0x90 = trunc(ms_f32 × 1000 /
0.25)`, reg `0x60` bit2 = 0.

### 5.2 Loop (per channel, independent)

```
dwAdjMaxNum = 15;  nLowBound = 0xAA (170);  nUpBound = 0xB4 (180)
fMax[c] = p.fMaxTGTime = 0.387 ; fMin[c] = 0.01 ; found[c] = 0
cur[c]  = p.fExposureTime[c]  = 0.35 / 0.33 / 0.16          // float32, from MyScanner_SetParameter2
for pass in 0..15:
    lamp on; Scanner_Calibration_SetScanParameter(1, 0, 0, p)   // shutter regs <- cur[]
    StartScan; read 1 packed line; StopScan
    (mR, mG, mB) = CalcMaxRGBData(buf, p)
    for c:
        if found[c]: continue
        if m[c] > 180:  fMax[c] = cur[c]; cur[c] = (float)((fMax[c] + fMin[c]) / 2.0)
        elif m[c] < 170: fMin[c] = cur[c]; cur[c] = (float)((fMax[c] + fMin[c]) / 2.0)
        else: found[c] = 1
    if all found: break
sprintf(local, "Exp.R(%d)=%f", …)   // formatted into a stack buffer, never printed
return 1                            // always; non-convergence keeps the last midpoint
```

Arithmetic: `fMax`, `fMin`, `cur` are `float`; the midpoint is computed in
double and stored back as float. Bounds: **0.01 ms .. fMaxTGTime** (= `fTGTime`
= 0.387 ms at 300/200 dpi; 0.717 / 1.683 / 3.367 at 600 / 1200 / 2400, `05
§2.4`), i.e. the shutter can never exceed the line period. Target window
**[170, 180]** on the 8-bit white level. 15 passes bisect a 0.377 ms span to
0.011 µs — well below one tick (0.25 µs), so a non-monotonic or noisy
response is the only way to not converge.

First-pass values (float32, ticks = `trunc(ms × 4000)`):

| ch | initial | ticks | if too bright → | ticks | if too dark → | ticks |
|---|---|---|---|---|---|---|
| R | 0.35 | 1399 | 0.18 | 719 | 0.3685 | 1473 |
| G | 0.33 | 1320 | 0.17 | 680 | 0.3585 | 1434 |
| B | 0.16 | 639 | 0.085 | 340 | 0.2735 | 1093 |

Hardware note: with the defaults the white strip reads ≈ 241/211/215, i.e.
all three channels start "too bright" and the first bisection step halves
the shutter times.

### 5.3 Statistic — `CalcMaxRGBData(buf, p, &mR, &mG, &mB)`

As decompiled (`Scanner_Calibration_CalcMaxRGBData.c`):

```
H = p.dwYExtent = 1 ; N = 0x15F = 351 sections ; w = p.dwXExtent / 351 = 3596 / 351 = 10
x0 = p.dwNewXPos = 267 ; bpl = p.dwCISBytesPerLine = 11664
mR = mG = mB = 0
for i in 0..H:
    for j in 0..351:
        aR = aG = aB = 0
        for k in 0..w:
            px = x0 + j*w + k                        // packed pixel index
            aR += buf[i*bpl + 3*px + 0]; aG += buf[.. + 1]; aB += buf[.. + 2]
        aR /= w; aG /= w; aB /= w
    mR = max(mR, aR & 0xFF); mG = max(mG, aG & 0xFF); mB = max(mB, aB & 0xFF)   // <-- outside the j loop
```

The `max` update sits **outside the section loop**, so only the last section
survives: the statistic is the **mean of the 10 packed pixels 267 + 3500 ..
3509 = [3767, 3776]** (segment 2, k = 1175..1184) per channel, from the one
line. The evident intent is the maximum over all 351 ten-pixel section means
of the window [267, 3863) ("CalcMax"). A reimplementation has to choose:
replicate the literal behaviour (bit-exact vendor results, but the exposure
is set by a 10-pixel patch near the left end of the raw line and the rest of
the strip may clip), or implement the intent (max of section means, more
robust, lower exposure). Either way the shading step (§6/§8) normalises to
0xF000 afterwards, so the choice mainly affects head-room. Decide on
hardware (§11).

Note on coordinates: packed index `i = s·S + k`; no segment rotation or crop
has been applied (`07 §4.2–4.3` live in `Scanner_ReadScanData`, not here).

---

## 6. White shading — `Scanner_Calibration_WhiteShading`

### 6.1 Frame

Lamp on (`Scanner_SwitchLampPower(1)`), **moving** at 464.9 pps away from
home from the current position (§3.2, white-shading column), 16-bit colour,
69 lines, raw read of 69 × 23328 = 1 609 632 bytes in one bulk IN. Shutter
= §5 result, AFE offsets = §4 result, gain 0x29. The capture loop `for (i = 1;
i < 2; i++) { …; if (i == 1) break; Scanner_MotorMove(800.0, 0x50, 1); }`
executes exactly once; the 0x50-step move is dead code.

Buffers: three `malloc(dwCISBytesPerLine / 3 = 7776)` = 3888 × u16 arrays
`pwWhiteShadingR/G/B` (`SHADING_COLOR`, `types.h`), i.e. **one entry per
raw sample σ of a plane**, not per segment and not per output pixel.

### 6.2 Motor and position

See §3.2. Feed 0, no fast move, scan ramp to 464.9 pps, direction away from
home, auto-go-home off. The carriage ends 37–69 full steps further out (open
question which) and stays there.

### 6.3 Statistic — `CalcGetShadingDataColor3(buf, R, G, B, W = 3888, H = 69, V = H >> 1 = 34, 1)`

```
for σ in 0..W:
    for j in 0..H:                                   // u16 little-endian samples
        t0[j] = u16 at byte 2·(σ + j·3W)             // plane 0 = B
        t1[j] = u16 at byte 2·(σ + j·3W + W)         // plane 1 = R
        t2[j] = u16 at byte 2·σ + j·6W + 4W          // plane 2 = G   (same thing, written differently)
    BubbleSort ascending (t0), (t1), (t2)
    lo = ((H − V) × 3) >> 2 = (35 × 3) >> 2 = 26 ;  hi = V + lo = 60
    B[σ] = (Σ t0[26..60)) / 34 ;  R[σ] = (Σ t1[26..60)) / 34 ;  G[σ] = (Σ t2[26..60)) / 34
```

So per raw sample and plane: sort the 69 line values, **drop the 26 lowest
and the 9 highest, average the 34 in between** (sorted ranks 26..59), integer
division, stored as u16. (Not the middle half: the window is biased toward
the upper values, 3/4 of the discarded samples are low ones.) Plane → output
mapping: plane 0 → `pwWhiteShadingB`, plane 1 → `pwWhiteShadingR`, plane 2 →
`pwWhiteShadingG` (consistent with `07 §3.1`). `isWhiteShading` is unused.

Generic form for other buckets: `lo = ((H − (H >> 1)) × 3) >> 2`, `count = H >> 1`.

---

## 7. Dark shading — `Scanner_Calibration_DarkShading`

Lamp off, still, 16-bit colour, 10 lines, raw read of 233 280 bytes. AFE
offsets and shutter as calibrated. Three 3888 × u16 arrays
`pwDarkShadingR/G/B`.

### 7.1 Statistic — `CalcGetShadingDataColor2(buf, R, G, B, W = 3888, H = 10)`

Per raw sample σ and plane: plain mean of the 10 u16 LE values (sum as u32,
integer division by 10), same plane → output mapping as §6.3 (plane 0 → B,
1 → R, 2 → G).

After this phase `g_szKDataPath` is set to `"/tmp"` (§1); no file is written.

---

## 8. Upload — `Scanner_Calibration_WriteShadingToDevice(0, 6, 3888, SD)` → `CtlImage_WriteShadingColor`

### 8.1 Channel → bank

`WriteShadingToDevice` passes `(whiteB, whiteR, whiteG, darkB, darkR, darkG,
(ushort)3888, 6)` to `CtlImage_WriteShadingColor(pwWhite0, pwWhite1,
pwWhite2, pwDark0, pwDark1, pwDark2, dwWidth, dwOffset)`, which writes
channel `c` to `shd_bank_address[c]`:

| Bank c | Bulk address (`04 §4.7`, iOptRes 1200) | Register | Data | = raw plane |
|---|---|---|---|---|
| 0 | `0x10014000` | `0xD0 = 0x0A` | B white/dark | plane 0 |
| 1 | `0x10024000` | `0xD1 = 0x12` | R white/dark | plane 1 |
| 2 | `0x10034000` | `0xD2 = 0x1A` | G white/dark | plane 2 |

So **bank index = raw plane index**, and within a bank the table is indexed
by the raw sample index σ (interleaved, reversed — exactly the order the
samples arrive in a raw line). The ASIC applies shading per raw sample
before the host ever sees the data; no unpacking is involved. (At iOptRes
2400 the addresses are `0x10014000 / 0x10034000 / 0x10054000`.)

### 8.2 Pair table — exact construction (per bank)

```
width  = 3888 ; base = CtlImage_ShadingBase = 0x4000 ; scanmod = RegTable[0x06] >> 5   // = 0 here (reg 6 = 0x10/0x18)
buf    = malloc(((width × 0x600) / 0xFC + 0x100) × 2)   // 47908 bytes, NOT zeroed
Bcnt   = dwOffset × 2 = 12                                // word index
for σ in 0..width:
    buf[Bcnt]     = dark[σ]                                          // u16
    diff          = (dark[σ] < white[σ]) ? white[σ] − dark[σ] : 1   // u32
    gain          = (base × 0xF000) / diff  = 0x3C000000 / diff      // u32 integer division
    buf[Bcnt + 1] = (gain < 0x10000) ? gain : 0xFFFF
    Bcnt += 2
    if scanmod != 2 and Bcnt % 0x100 == 0xFC: Bcnt += 4             // skip words 0xFC..0xFF of every 256-word block
CMDASIC_WriteBulkData(shd_bank_address[c], buf, Bcnt × 2)             // Bcnt = 7908 words → 15816 bytes
free(buf)
```

Facts that follow:

- Each pair is `[dark, gain]` in that order, each a host `u16` →
  **little-endian** on the wire (`CMDASIC_WriteBulkData` sends the host
  bytes as-is: `0x82` setup with `bmode = 1`, payload `{u32 addr, u32 len}`
  LE, then bulk OUT on EP `0x02`, `06 §4.4`).
- `gain = 0x3C000000 / (white − dark)`; unity (0x4000) when `white − dark =
  0xF000`; saturates at 0xFFFF when `white − dark ≤ 0x3C00` (15360);
  `white ≤ dark` is treated as a difference of 1 → 0xFFFF. With `ShadingBase
  = 0x4000` (reg 6 bit3 = 1, `05 §4.2` step 34) the corrected output is
  presumably `(raw − dark) × gain / 0x4000`, mapping the calibrated white
  level to 0xF000 = 240/255 (inferred from the constants, not from code).
- Word position of sample σ: `w(σ) = 12 + 2σ + 4·⌊(12 + 2σ) / 252⌋`.
  Block 0 holds σ = 0..119 at words 12..251, block b ≥ 1 holds 126 samples
  at words 256b .. 256b+251; σ = 3887 lands at word 7906. 31 blocks, 120
  skipped words, upload length **15816 bytes** per bank.
- Words 0..11 (the `dwOffset = 6` pairs) and the 4 skipped words per block
  are **uninitialised heap memory** in the vendor driver; write zeros.
  The `scanmod == 2` exception (reg 6 bits 7:5 = 2) never applies: init sets
  those bits to 0 (`CtlDevice_SetScanMOD`, `05 §4.1`).
- The meaning of the 6-pair offset is unknown (§11). `CtlImage_SetDefaultShading`
  (uncalled) uses `Bcnt = 4`, i.e. a 2-pair offset, with a flat table
  `dark = 0, white = 0xF000` → `gain = 0x4000` for `12 × iOptRes` samples;
  it additionally zeroes the gain of samples 240..247 / 260..267 / 280..287
  in banks 0/1/2. It is never called, so none of that is on the wire.
- Bank spacing is 0x10000 bytes (64 KiB); 15816 bytes fit with room.

### 8.3 Register writes around the upload

Before: none (the banks were set at init, `04 §4.7`). After the third bank:
`bits(0x01, bit1, 1) = 0`. Reg 1 bit1 is never set anywhere in the corpus
(only cleared here, in `WriteShadingGray` and `SetDefaultShading`), so this
is a full-byte write of the unchanged shadow — `0x83 [01 80]` at this point
(bit7 set by the last frame's programming, bit0 cleared by `StopScan`, bit5
= 0). Its semantics are unknown; replicate it.

Then `MyScanner_StartScan` continues (§10). The real scan's
`Scanner_SetScanParameter` enables shading (`0x01` bit5 = 1, `0x06` bit3 =
1) and gamma.

### 8.4 Totals

Per calibration: 3 bulk OUT of 15816 bytes (+ 3 setup control transfers);
no other data upload. Host-side arrays: 6 × 3888 u16.

---

## 9. Gamma — `Scanner_SetGamma(0x10)` → `CtlImage_SetGamma(16)`

Called after `Scanner_SwitchLampPower(1)` and before
`Scanner_SetScanParameter` (§0). Confirms `05 §4.6`:

```
g = 16 / 10.0 = 1.6                                      (double)
for i in 0..256:
    t[i] = (i < 255) ? (uint16)(int)(pow((double)((i+1) × 256) / 65536.0, 1.0 / g) × 65536.0) : 0xFFFF
```

`pow` is the C `double` `pow`; truncation toward zero via `(int)` then
`(ushort)`. The same table is computed three times (R, G, B tables are
identical). Register writes, in order: `W2(0xC5, 0)`, `W2(0xC7, 0)`,
`W2(0xC9, 0)`, reg W `0xBD = 0x00`, reg W `0xBE = 0x07`; then three
`CMDASIC_WriteBulkData` of 512 bytes (u16 LE) to `0x01000000`, `0x01000200`,
`0x01000400`.

Test vector (`gamma = 1.6`, exponent 0.625):

| i | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | … | 127 | … | 252 | 253 | 254 | 255 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `t[i]` | 0x0800 | 0x0C56 | 0x0FE5 | 0x1306 | 0x15DF | 0x1883 | 0x1AFE | 0x1D58 | | 0xA5FE | | 0xFE1E | 0xFEBF | 0xFF5F | 0xFFFF |

First bytes on the wire: `00 08 56 0C E5 0F 06 13 …`; last: `1E FE BF FE 5F
FF FF FF`. `image::gamma_lut(1.6)` implements this formula; these values
check it.

---

## 10. Order relative to the real scan and the carriage position

From `MyScanner_StartScan.c`, verbatim order: open → `Api_InitiateASIC3` →
`0x3A[2] = 1` → `Scanner_CheckCalibration` → `Scanner_Calibration2` →
recompute `CCD_DPIHW` (1200) and `ScanTBunit` (**4** at 300 dpi; it was 8
during calibration) → `Scanner_SwitchLampPower(1)` → `Scanner_SetGamma(0x10)`
→ `Scanner_SetScanParameter(0, 0, 1, &g)` → `isFiestGetRows = 1` →
`Scanner_ColorMatrixEnhance_Init` (host only) → `Api_StartScan`.

Carriage: `StartScan1` homed the carriage before `MyScanner_StartScan`
(`05 §0`). Calibration then moves it **225 full steps out** (§2.2) and the
white-shading scan adds ≈ 37–69 steps (§3.2). **Nothing moves it back**:
`Scanner_Calibration_StopScan` only waits, `Scanner_Calibration2` has no
home move, and `MyScanner_StartScan` goes straight to `Scanner_SetScanParameter`.
The real scan is programmed with `spScanBeginLine = 810` (y = 0) →
`CalculateFeedReg(405)` → fast feed of 306 unit-steps + ramps, with
auto-go-home off — i.e. the vendor's "home to first image line" distance of
405 full steps is measured **from the post-calibration position**, not from
home. Total nominal travel from home to the first captured line is therefore
≈ 225 + (37..69) + 405 ≈ 670–700 full steps ≈ 1.1–1.2 in.

Consequence for a reimplementation that skips or reorders calibration: either
perform the same 225-step move (and a white-shading-length move) before the
real scan, or add the missing travel to `spScanBeginLine` (units of
2 × full steps at 300 dpi, since `CalculateFeedReg` receives
`spScanBeginLine / (iOptRes / MotorGear) = spScanBeginLine / 2`). Verify the
resulting top margin against a ruler on hardware (§11).

---

## 11. Open questions

1. **AFE channel ↔ segment mapping.** §4.3 ties AFE offset 5/6/7 to CIS
   segments 0/2/1 (G plane). Test: with the lamp off, set AFE 5 to 0x13F and
   6/7 to 0x80; only one segment's black level should change. Confirm the
   same for gains 2/3/4 with the lamp on.
2. **`CalcMaxRGBData` literal vs. intended** (§5.3): decide whether to
   replicate the 10-pixel-patch behaviour or take the max over all 351
   sections. Measure the resulting white level across the whole strip with
   each choice.
3. **White-shading travel** (§3.2/§6.2): does the ASIC free-run the motor at
   464.9 pps (≈ 37 steps during 69 lines) or step once per line (69 steps)?
   Measure the carriage position before/after via a subsequent timed move or
   by scanning a ruler afterwards. This also fixes the "post-calibration
   position" of §10.
4. **Shading table offset 6 pairs and the 4-word skips** (§8.2): why the
   ASIC wants the data to start at word 12 and skips words 252..255 of each
   256-word block (SDRAM page boundary? `0xFC` = 252 = 2 × 126). Test: upload
   a table with a deliberate step at σ = 120 (the first block boundary) and
   check that the step appears at the right raw sample.
5. **Corrected-value formula**: `(raw − dark) × gain / 0x4000` → 0xF000 at
   white is inferred from the constants. Verify by scanning the strip with
   shading on: white should read ≈ 0xF0 (8-bit) regardless of exposure.
6. **Reg 1 bit1** cleared after the upload (§8.3): purpose unknown; never set.
7. **Vref during still frames**: `0x3B = 0` (from `WriteColorTiming`) and
   `0x3C` = 0 after the §2.2 move; harmless with the motor unpowered, but
   note the real scan restores `0x0A/0x0A`.
8. **Non-convergence policy**: the vendor silently keeps the last bisection
   value after 15 passes for both offset and exposure. Decide whether to
   error out instead (recommended: log and continue, matching the vendor).
9. **Single 1.6 MB bulk read** for the white frame (§3.3): the vendor issues
   one `ReadRawImage` of 1 609 632 bytes. `07 §2.1` notes short reads are
   silently accepted by the vendor; our implementation must check the
   transferred length and may need to split the read if libusb/the device
   limits a single transfer.
10. **`ScanTBunit = 8` for the white-shading scan** vs. 4 for the real 300 dpi
    scan: the scan slope table is built with unit 8 (28 words, `0xA0[0:3] =
    3`). Whether unit 8 vs. 4 changes the physical speed (`06 §9`) affects
    question 3.
11. **200 dpi**: calibration runs entirely in the 300 bucket (geometry 0x10B /
    3888 / 1296, shading banks for 3888 samples) while the real 200 dpi scan
    uses 2592 samples per plane (`05 §2.3`); whether the ASIC indexes the
    shading banks the same way at DPISET 800 is unverified.
