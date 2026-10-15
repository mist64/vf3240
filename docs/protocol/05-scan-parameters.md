# VF3240 scan parameters

Status: **verified on hardware (2026-10-13)** for 300 dpi colour, uncorrected: still-mode and moving scans programmed from this document produce correct images (`docs/hardware/`). Also verified for 600 and **2400 dpi colour** (full-bed scans from the §4.4 values; the 2400 path exercises TimingIdx 1, AFE row 0, the `0x1E` bit-3 branch of §3.3 and `TBTIME` code 2). 1200 dpi verified (full bed, 2026-10-13 evening). **200 dpi verified** (full bed 2396×3359 in 3.9 s of scanning: the 300 dpi window with `DPISET = 800` and `CurveData1` at 2584 pps produce a correctly scaled image, confirming the inferred 1200 → 800 decimation). Bit semantics remain static inference.

Scope: everything between ASIC init (`Api_InitiateASIC3`) and "start scan"
(`Api_StartScan`) for a flatbed scan — the register programming that defines
resolution, exposure, geometry, colour depth and the image/shading/gamma
enables. Motor-table programming (`CtlMotor_SetScanTableUnit`,
`SetFastTableUnit`, `LoadPPSCurveTo*Table`, `CalculateFeedReg`,
`SetZ1Z2Value`) is named where it occurs in the order but documented in
`06-motor.md`.

Every value is resolved concretely for **300 dpi, 24-bit colour, flatbed,
full bed**; other resolutions are given where they come from a small table.

## Source of truth

Decompiled corpus `re/exports/libviisanvf3240dr.so/decomp/<Function>.c`
(addresses from `functions.txt`, struct layouts from `types.h`), and data
tables read out of the ELF
`vendor/linux-sane-1.0.2.0/unpacked/usr/local/lib64/sane/aviisandr-adv/VF3240/libviisanvf3240dr.so`
(symbol → bytes via `nm -S` + section-header mapping; all tables quoted below
were dumped from the `.data` section at the stated VMAs). The SANE shim side
is `re/exports/libsane-aviisandrvf3240.so.1.0.27/decomp/` plus its ELF
`.../lib64/sane/libsane-aviisandrvf3240.so.1`.

| Function | Address | Role |
|---|---|---|
| `StartScan1` | `0x118172` | shim entry → `MyScanner_*` sequence |
| `MyScanner_SetParameter2` | `0x119823` | fills `g_Scan_Parameters` |
| `MyScanner_GetParameter2` | `0x119fb3` | reports ppl/bpl/len back |
| `MyScanner_StartScan` | `0x11a125` | init → calibrate → lamp → gamma → set params → start |
| `Scanner_SetScanParameter` | `0x11aedf` | builds `ScanParameter`, per-dpi register bits, AFE, Vref |
| `ScanCtrl_SetScanParameter` | `0x11193c` | the bulk of the register programming |
| `WriteColorTiming` | `0x110fe8` | CCD/AFE timing table upload |
| `CtlDevice_LoadInitRegData3` | `0x10cbb8` | ASIC init (only consulted for shadow state) |

Wire primitives are those of `01-usb-transport.md` / `02-opcode-map.md`:
"reg W" = `CMDASIC_WriteRegister` (one `0x83` control write, payload
`[addr, data]`); "bits" = `CMDASIC_WriteRegisterBit(addr, startBit, nBits,
value)` which read-modify-writes the **host shadow** (`CMDASIC_RegTable`) and
issues one `0x83` write; "W2"/"W3" = `CMDASIC_WriteTwiRegister` /
`CMDASIC_WriteTriRegister`, which issue **one** `0x83` multi-write whose
payload is `[addr, hi, addr+1, mid, addr+2, lo]` — i.e. multi-byte registers
are **big-endian, consecutive addresses**. "AFE W" = `CtlAfe_WriteAFERegister
(afeAddr, data16)` = one `0x83` multi-write with payload
`[0x51, afeAddr, 0x5D, data>>8, 0x5E, data&0xFF]` (ASIC regs `0x51`/`0x5D`/
`0x5E` form an indirect port to the analog front end).

---

## 0. Where this sits in the sequence

```
sane_start (shim)
  StartScan1(mode, dpi, x, y, xext, yext, isColorMatch=1, &ppl, &bpl, &len, &memBufBaseLines)
    MyScanner_Open()                  CCD_DPIHW = 1200; ScanTBunit = 2  (host globals only)
    MyScanner_CarriageHome()          open, Api_InitiateASIC, lamp off, Api_MoveMotor2(1900.0,12000,0,4), wait, close
    MyScanner_SetParameter2(mode, dpi, dpi, x, y, xext, yext, gamma=0x10)   -> g_Scan_Parameters  (sec. 2)
    MyScanner_GetParameter2(...)                                             -> ppl/bpl/len        (sec. 2)
    MyScanner_StartScan(NULL)
      CCD_DPIHW = (dpi < 301) ? 1200 : 2400;  ScanTBunit = 8
      Api_OpenDevice2(0x2e5a, 0xd011)
      Api_InitiateASIC3(xres, yres)   -> CtlDevice_LoadInitRegData3  (ASIC init, separate document)
      bits(0x3A, bit2, 1)             "scan LED on" (MyScanner_SetScanLED uses the same bit)
      Scanner_CheckCalibration() ? Scanner_CalibrationFromFile() : Scanner_Calibration2()   (sec. 5.2)
      CCD_DPIHW / ScanTBunit recomputed (table in sec. 3.1)
      Scanner_SwitchLampPower(1)      reg 3: bit5 = 0 (XP lamp off), bit4 = 0, bit4 = 1 (lamp on)
      Scanner_SetGamma(0x10)          -> CtlImage_SetGamma(16): gamma 1.6 LUT upload (sec. 4.6)
      Scanner_SetScanParameter(still=0, autohome=0, HWshading=1, &g_Scan_Parameters)   (sec. 3, 4)
      g_Scan_Parameters.isFiestGetRows = 1
      Scanner_ColorMatrixEnhance_Init()   host-side only, no register traffic (grep: 0 CMDASIC calls)
      Api_StartScan()                 -> ScanCtrl_StartScan: CtlImage_EnableScan(1), CtlMotor_SetMotorStartMove, ChkScanDataReady  (out of scope)
```

`StartScan1` rejects any `dwDPI` not in {200, 300, 600, 1200, 2400}
(`StartScan1.c`). `g_isDoColorMatch = isColorMatch`; the shim always passes 1.

---

## 1. SANE shim geometry

### 1.1 Option defaults (`sane_aviisandrvf3240_open`, "init_options")

| Opt | Name | Type/unit | Constraint | Default |
|---|---|---|---|---|
| 1 | `mode` | string | `mode_list` = {`Color24`, `Gray8`} | `Color24` |
| 2 | `source` | string | `source_list` = {`FLAT_FRONT`} | `FLAT_FRONT` |
| 3 | `resolution` | int, DPI | `DPI_list` = `{5, 200, 300, 600, 1200, 2400}` (word list, first word is the count) | **200** |
| 4 | `size` | string | `size_list` (`A3`, …) | `A3` |
| 5 | `preview` | bool | — | 0 |
| 6 | `tl-x` | fixed, mm | `x_range` | 0 |
| 7 | `tl-y` | fixed, mm | `y_range` | 0 |
| 8 | `br-x` | fixed, mm | `x_range` | `x_range.max` |
| 9 | `br-y` | fixed, mm | `y_range` | `y_range.max` |

Ranges (shim ELF, `.data`): `x_range = {min 0, max 0x01304AC0, quant 0}` →
**304.30 mm**; `y_range = {0, 0x01AAB851, 0}` → **426.72 mm**.

The `workInfo` struct (`controlParameter`, shim `types.h`) is initialised in
inches: `m_fLeft = 0.0, m_fTop = 0.0, m_fRight = 11.98, m_fBottom = 16.8,
m_nScanMode = 0, m_nXResolution = 200, m_nYResolution = 200`. 11.98 in =
304.3 mm and 16.8 in = 426.7 mm, i.e. the option maxima. The option setter
is a jump table Ghidra could not recover (`sane_aviisandrvf3240_control_option`),
so the mm→inch conversion (`/25.4`) is **inferred** from these equalities,
not read from code. `m_nScanMode`: 0 = `Color24`, 1 = `Gray8`
(`calc_image_paramters` maps 0 → `SANE_FRAME_RGB`/8 bit, 1 → GRAY/8).

### 1.2 What `sane_start` passes to `StartScan1`

From `sane_aviisandrvf3240_start.c` (handle offsets: `0x28c fLeft, 0x290
fTop, 0x294 fRight, 0x298 fBottom, 0x29c m_nScanMode, 0x2a0 m_nXResolution`),
all in **float32** arithmetic, truncated to integer:

```
dwScanMode = m_nScanMode                         (0 colour, 1 gray)
dwDPI      = m_nXResolution                      (x and y resolution are the same value)
dwX        = (ulong)((float)(11.98 - fRight) * dpi)     <-- measured from the RIGHT edge of the bed
dwY        = (long)(dpi * fTop)
dwXExtent  = (ulong)((fRight - fLeft) * dpi)
dwYExtent  = (ulong)((fBottom - fTop) * dpi)
isColorMatch = 1
```

Note the x origin: `dwX` is the distance from the right-hand bed edge
(`11.98 - fRight`), i.e. CIS pixel 0 is on the right side of the glass. For
the full bed `dwX = 0` regardless of dpi.

Full-bed values (float32 emulation of the above; the odd `3593` comes from
`11.98f * 300f` rounding to `3593.99976` then truncating — **verify against
the shim's own `dwXPos=…` debug log on real hardware**):

| dpi | dwX | dwY | dwXExtent | dwYExtent |
|---|---|---|---|---|
| 200 | 0 | 0 | 2396 | 3359 |
| **300** | **0** | **0** | **3593** | **5040** |
| 600 | 0 | 0 | 7187 | 10080 |
| 1200 | 0 | 0 | 14375 | 20160 |
| 2400 | 0 | 0 | 28751 | 40320 |

### 1.3 What `StartScan1` returns (300 dpi colour)

`MyScanner_GetParameter2` → `ppl = dwXExtent = 3593`, `bpl = 3 * 3593 =
10779`, `len = dwYExtent = 5040`, `memBufBaseLines = dwCISBytesPerLine /
dwCISBytesPerLine = 1`. Side effect: `g_Block_Buffer_Size =
dwCISBytesPerLine = 11664` (see 2.2). `sane_start` then publishes
`SANE_Parameters{format RGB, last_frame 1, bytes_per_line = ppl*3 = 10779,
pixels_per_line = 3593, lines = 5040, depth 8}`.

---

## 2. `g_Scan_Parameters` — `MyScanner_SetParameter2` / `GetParameter2`

### 2.1 Struct layout (`types.h`, `struct tagSCAN_PARAMETERS`, 0xC0 bytes — host-side only)

| Off | Type | Field | Set by |
|---|---|---|---|
| 0x00 | DWORD | `dwScanMode` | 0 colour, 1 gray (input mode 3/4 = 16-bit variants map to 0/1) |
| 0x04 | DWORD | `dwColorBits` | 24 / 8 / 48 / 16 |
| 0x08 | DWORD | `dwColorDepth` | 8 or 16 (bits per channel) |
| 0x0C | DWORD | `dwResolution` | = dwXRes |
| 0x10 | DWORD | `dwXPos` | from shim (right-edge origin) |
| 0x14 | DWORD | `dwYPos` | from shim |
| 0x18 | DWORD | `dwXExtent` | output pixels per line |
| 0x1C | DWORD | `dwYExtent` | output lines |
| 0x20 | DWORD | `dwBytesPerLine` | xext × {3, 1, 6, 2} |
| 0x24 | DWORD | `dwGamma` | 0x10 (fixed by `StartScan1`) |
| 0x28 | DWORD | `dwNewXPos` | `dwXPos + dwStartPixelOffset` — crop start inside the raw CIS line |
| 0x2C | DWORD | `dwStartPixelOffset` | per-dpi table below |
| 0x30 | ptr | `pCallback` | progress callback (NULL from shim) |
| 0x38 | DWORD | `dwCISPixelsPerSeg` | per-dpi table |
| 0x3C | DWORD | `dwCISPixelsPerLine` | per-dpi table (= 3 × PerSeg) |
| 0x40 | DWORD | `dwCISBytesPerLine` | PerLine × {3, 1, 6, 2} |
| 0x44 | DWORD | `dwXResolution` | = dwXRes |
| 0x48 | DWORD | `dwYResolution` | = dwYRes (shim passes dpi for both) |
| 0x4C | BOOL | `isFiestGetRows` | set 1 just before `Api_StartScan` |
| 0x50 | BOOL | `isCancelScan` | |
| 0x54 | `AFE_INFO` | `Afe_Info` | see 2.4 |
| 0x80 | `SHADING_DATA` | `ShadingData` | 8 pointers (colour R/G/B white+dark, gray white+dark) |

`AFE_INFO` (`types.h`): `WORD wGainR, wGainG, wGainB, wOffsetR, wOffsetG,
wOffsetB, wOffsetIdxR, wOffsetIdxG, wOffsetIdxB; float fTGTime,
fExposureTimeR, fExposureTimeG, fExposureTimeB, fMaxTGTime;` (offsets
0x54..0x7B).

### 2.2 Mode → colour fields (`MyScanner_SetParameter2`)

| input `dwScanMode` | stored `dwScanMode` | `dwColorBits` | `dwColorDepth` | `dwBytesPerLine` | `dwCISBytesPerLine` |
|---|---|---|---|---|---|
| 0 (Color24) | 0 | 24 | 8 | xext × 3 | PerLine × 3 |
| 1 (Gray8) | 1 | 8 | 8 | xext | PerLine |
| 3 (Color48) | 0 | 48 | 16 | xext × 6 | PerLine × 6 |
| 4 (Gray16) | 1 | 16 | 16 | xext × 2 | PerLine × 2 |

Any other mode → returns FALSE. 300 dpi colour: `dwBytesPerLine = 10779`,
`dwCISBytesPerLine = 3888 × 3 = 11664`.

### 2.3 Per-dpi CIS geometry (`MyScanner_SetParameter2`, thresholds `< 200` reject, `< 300`, `< 600`, `< 1200`, `< 2400`, else)

| dpi | `dwStartPixelOffset` | `dwCISPixelsPerSeg` | `dwCISPixelsPerLine` (= 3 segs) | `dwNewXPos` (full bed) |
|---|---|---|---|---|
| 200..299 | 0x0AA = 170 | 0x360 = 864 | 0xA20 = 2592 | 170 |
| **300..599** | **0x100 = 256** | **0x510 = 1296** | **0xF30 = 3888** | **256** |
| 600..1199 | 0x1FF = 511 | 0xA20 = 2592 | 0x1E60 = 7776 | 511 |
| 1200..2399 | 0x3FD = 1021 | 0x1440 = 5184 | 0x3CC0 = 15552 | 1021 |
| 2400+ | 0x7F8 = 2040 | 0x2880 = 10368 | 0x7980 = 31104 | 2040 |

The CIS is three segments of `PerSeg` pixels; at 300 dpi a segment is 1296 px
(4.32 in), so the sensor line is 12.96 in (329 mm) — wider than the 11.98 in
bed. `Scanner_ReadScanData` (`0x11cee2`) later unpacks the raw
`dwCISBytesPerLine` block (`ScanCtrl_Pack_3CH_CIS_Color`, a segment rotation,
then a crop starting at pixel `dwNewXPos` for `dwXExtent` pixels). That data
path is out of scope here; it only matters that **the ASIC is programmed in
per-segment pixels** (`spScanLinePixels = dwCISPixelsPerSeg`) and the raw
line is 3 × that.

Surprise: `Scanner_Calibration2` uses *different* start offsets for the same
dpi (300: 0x10B, 600: 0x216, 1200: 0x42C, 2400: 0x858) and a fixed
calibration width `dwXExtent = dpi * 0xE0C / 300` (3596 px at 300 dpi).

### 2.4 Per-dpi exposure defaults (`MyScanner_SetParameter2`)

Units are milliseconds (converted to 0.25 µs ticks in 4.4). `fMaxTGTime` is
always set equal to `fTGTime`. The last row is the fall-through for every
resolution not equal to 2400/1200/600, i.e. 300 **and** 200.

| dpi | mode | `fTGTime` | `fExposureTimeR` | `G` | `B` |
|---|---|---|---|---|---|
| 2400 | colour | 3.367 | 2.62 | 2.62 | 2.62 |
| 2400 | gray | 10.101 | 2.62 | 2.62 | 2.62 |
| 1200 | colour | 1.683 | 1.315 | 1.315 | 0.65 |
| 1200 | gray | 5.049 | 1.315 | 1.315 | 0.65 |
| 600 | colour | 0.717 | 0.67 | 0.67 | 0.35 |
| 600 | gray | 2.151 | 0.67 | 0.67 | 0.35 |
| **300 / 200** | **colour** | **0.387** | **0.35** | **0.33** | **0.16** |
| 300 / 200 | gray | 1.161 | 0.35 | 0.33 | 0.16 |

Gray line period is ≈ 3 × colour (one line carries all three LED pulses, see
4.4). AFE defaults, all modes/dpis: `wGainR/G/B = 0x29`, `wOffsetR/G/B =
0x80`, `wOffsetIdxR/G/B = 0x7F`.

**Important:** `Scanner_Calibration2` (run inside `MyScanner_StartScan`
before `Scanner_SetScanParameter`) writes its results back into
`g_Scan_Parameters.Afe_Info.wOffsetR/G/B` and `fExposureTimeR/G/B`
(`Scanner_Calibration2.c`, tail). So on the vendor path the shutter registers
and AFE offsets of the real scan are **calibration results**, not the table
above; only `fTGTime` and `wGain` survive untouched. The table is what an
uncalibrated first scan uses.

### 2.5 `MyScanner_GetParameter2`

Recomputes `bpl` from `(dwScanMode, dwColorDepth)` (same ×3/×1/×6/×2 rule),
returns `xres, yres, ppl = dwXExtent, bpl, len = dwYExtent, memBufBaseLines
= 1`, and sets `g_Block_Buffer_Size = dwCISBytesPerLine`.

---

## 3. `Scanner_SetScanParameter(still, autohome, HWshading, p)`

Called from `MyScanner_StartScan` as `(0, 0, 1, &g_Scan_Parameters)`.

### 3.1 Host globals fixed before the call (`MyScanner_StartScan`)

| dpi | `CCD_DPIHW` → `CtlSensor_iOptRes` (set in `LoadInitRegData3`) | `ScanTBunit` |
|---|---|---|
| 200 | 1200 | 4 |
| **300** | **1200** | **8** |
| 600 | 2400 | 8 |
| 1200 | 2400 | 2 |
| 2400 | 2400 | 8 |

(`ScanTBunit` logic: `dpi < 301 → 1200 else 2400`; the TBunit nest resolves
to 8 for 300/600/2400, 2 for 1200, 4 for 200 given xres = yres.)
`CtlSensor_iOptRes` is the "hardware dpi" the CIS is clocked at; everything
below is expressed relative to it.

### 3.2 The `ScanParameter` struct (`types.h`, 52 bytes, host-side)

| Off | Type | Field | Value at 300 dpi colour | How |
|---|---|---|---|---|
| 0 | uint | `spScanBeginPixel` | 0 | constant |
| 4 | uint | `spScanBeginLine` | **810** (0x32A) | see below |
| 8 | uint | `spScanLinePixels` | 1296 | `dwCISPixelsPerSeg` |
| 12 | uint | `spTotalScanLines` | 5040 | `dwYExtent` |
| 16 | ushort | `spPixelResolution` | 300 | dpi |
| 18 | ushort | `spMotorResolution` | 300 | `dwYResolution` |
| 20 | uchar | `spImageType` | 0 | 0 colour (`dwScanMode` 0), 1 gray (`dwScanMode` 1 or 2) |
| 21 | uchar | `spMonoChannel` | 1 | constant (gray uses channel 1 = "filter 2", see 4.2 step 7) |
| 22 | uchar | `spPixelDepth` | 8 | `dwColorDepth` |
| 24 | float | `spExposureTime` | 0.387 | `fTGTime` |
| 28 | float[3] | `spShutterTime` | 0.35, 0.33, 0.16 | `fExposureTimeR/G/B` |
| 40 | uchar | `spTimingIdx` | **4** | 2400 → 1, 1200 → 2, 600/300/200 → 4 |
| 41 | uchar | `spDummyLines` | 0 | constant |
| 42 | uchar | `spRepeatParams` | 0 | constant (1 would skip the whole programming block) |
| 43 | uchar | `spStillMode` | 0 | arg `still` |
| 44 | uchar | `spAutoGoHome` | 0 | arg `autohome` |
| 45 | uchar | `spDisBackTrack` | 0 | constant |
| 46 | uchar | `spSWCCDPacking` | 0 | constant |
| 47 | uchar | `spShading` | 1 | arg `HWshading` |
| 48 | uchar | `spGammaMapping` | 1 | constant (calibration variants use 0) |
| 49 | uchar | `spADFscan` | 0 | constant |
| 50 | uchar | `spMotorFixedSpeed` | 0 | constant |

**`spScanBeginLine`** ("leading_edge"), colour and gray branches identical:

| xres | formula |
|---|---|
| 200..299 | `(long)((float)dwYPos / (float)dwYRes * 2400.0) + 0x35C (860)` |
| **300..599** | `(long)(dwYPos / dwYRes * 2400.0) + 0x32A (810)` |
| 600..1199 | `(long)(dwYPos / dwYRes * 4800.0) + 0x582 (1410)` |
| 1200..2399 | `(long)(dwYPos / dwYRes * 4800.0) + 0x51E (1310)` |
| 2400+ | `(long)(dwYPos / dwYRes * 4800.0) + 0x4A6 (1190)` |

i.e. the y offset in units of 2 × `CtlSensor_iOptRes` lines per inch, plus a
fixed distance from the home position to the glass edge. It is consumed only
by `CtlMotor_CalculateFeedReg(spScanBeginLine / (iOptRes / MotorGear))`
(`MotorGear = 600`, so ÷2 at 300 dpi → 405; ÷4 at 600+ dpi) — see
`06-motor.md`.

### 3.3 Register writes done by `Scanner_SetScanParameter` itself (after `Api_SetScanParameter` returns)

The dispatch is `if xres == 2400 … else if 1200 … else if 600 … else` — the
`else` covers **300 and 200**. In order:

| Step | xres | Wire operation | Notes |
|---|---|---|---|
| 1 | all | bits(0x35, bit0, 1) = 1 | init table had 0x7F; `LoadInitRegData3` ends with bit0 = 0, bit1 = 1; now 0x7F again |
| 2 | all | bits(0x36, bit4, 1) = 1 | init 0x0F → 0x1F |
| 3 | all | bits(0x33, bit4, 1) = 0 | timing table set 0x0E → stays 0x0E |
| 4 | 2400, 300, 200 | bits(0x4F, bit3, 1) = 1 | `0x4F` is **not** in the init table; shadow starts at 0 → 0x08 |
| 4' | 1200, 600 | bits(0x4F, bit3, 1) = 0 | |
| 5 | 2400, 300, 200 | reg W 0x68 = 0x00 | |
| 6 | 2400, 300, 200 | reg W 0x69 = 0x6D | |
| 7 | 2400, 300, 200 | bits(0x13, bits 7:5, 3) = 1 | init 0x1F → 0x3F |
| 8 | 2400, 300, 200 | bits(0x12, bits 7:5, 3) = 7 | init 0x00 → 0xE0 |
| 9 | 2400 | bits(0x1E, bit3, 1) = 0 | |
| 9' | 300, 200 | bits(0x1E, bit3, 1) = 1 | |
| 10 | 300, 200 | bits(0x19, bit7, 1) = 1 | |
| 5'' | 1200, 600 | bits(0x36, bit1, 1) = 1 | |
| 6'' | 1200 / 600 | bits(0x33, bit1, 1) = 1 / 0 | |
| 7'' | 1200 / 600 | bits(0x32, bit0, 1) = 1 / 0 | init 0x69 (GPIO000 table 0x61, GPIO111 0x7D) |
| 11 | all | `Api_SetAFEGain(0x29, 0x29, 0x29)` → AFE W 2 = 0x0029, 3 = 0x0029, 4 = 0x0029 | each: `0x83` payload `51 02 5D 00 5E 29` etc. Gain is masked `& 0xFF` |
| 12 | all | `Api_SetAFEOffset(0x80, 0x80, 0x80)` → AFE W 5 = 0x0080, 6 = 0x0080, 7 = 0x0080 | after calibration these are the calibrated offsets |
| 13 | all | `CtlMotor_SetMotorVref(0, 0, 2, 2)` → reg W 0x3B = 0x0A | field layout `gohome<<6 \| ffast<<4 \| ssback<<2 \| fscan`, 2 bits each; -1 = keep |
| 14 | all | `CtlMotor_SetMotorVref_2(0, 0, 2, 2)` → reg W 0x3C = 0x0A | same layout, second motor-current register |

The meaning of the per-dpi bits in 0x12/0x13/0x19/0x1E/0x32/0x33/0x35/0x36/
0x4F/0x68/0x69 is not recoverable from names; they are presumably CIS clock /
multiplexer configuration that differs between the 1200-dpi-base and
2400-dpi-base modes (note 2400 and 300/200 share most bits, 600 and 1200
share the other set). Reproduce them verbatim.

---

## 4. `Api_SetScanParameter` → `ScanCtrl_SetScanParameter(pSPM)`

`Api_SetScanParameter` is a one-line pass-through.

### 4.1 Globals consumed (ELF `.data` initial values; none are changed on the flatbed path)

| Global | Value | Used for |
|---|---|---|
| `CtlSensor_iOptRes` | 1200 (300/200 dpi) or 2400 | = `CCD_DPIHW` at init time |
| `CtlDevice_PixelTime` | `Clock_Pixel / Clock_ASIC = 12 / 48 = 0.25` | ms×1000 → ticks divisor (ticks are 0.25 µs; i.e. `ticks = µs × 4`) |
| `MotorGear` | 0x258 = 600 | motor steps per inch base |
| `ScanTBVref`, `FastTBVref` | 1, 1 | Vref written by ScanCtrl, then overwritten by sec. 3.3 step 13 |
| `FastTBunit` | 2 | motor |
| `FastTBPPS` | 1500.0f | motor |
| `BGRLineSequence`, `CCDStaggerStyle` | 0, 0 (bss) | gray/colour line order; stagger off |
| `m_iDiffBufLines` | 0 (bss; only set when `m_ccdpackingflg`, which is hard-coded 0) | adds to scan length |
| `CtlImage_ShadingBase` | 0x4000 (`ShadingGain = 4`, not 8) | reg 6 bit3 when shading on |
| `shd_full_range` | 96 at iOptRes 1200, 192 at 2400 (`SDRamSet_ShadingBank`) | SDRAM bank layout |
| `CtlSensor_nTBtime` | 1 (set by `ScanCtrl_MoveMotor2` during `CarriageHome`; otherwise bss 0 → still-mode divide-by-zero) | still mode slope |
| Shadow regs from init | 0x0B = 0x6C, 0x08 = 0x40, 0x06 = 0x10, 0x1F/0x20/0x21/0x22 = 00/03/00/56 | derived values below |

Shadow derivation: init table reg 0x0B = 0x8A; `SetSDRAMType(DRAM_SEL=4,
ChipN=1)` → bits 2:0 = 4 → 0x8C, reg 8 bit7 = 0, bit1 = 0; `SetScannerClock
(48)` → `(0x8C & 0x1F) | 0x60 = 0x6C`; `CtlADF_ADFInit` sets reg 8 bit6 →
0x40; `SetScanMOD(12)` sets reg 6 bits 7:5 = 0 and reg 0x14 bit0 = 0.

### 4.2 Step table, 300 dpi 24-bit colour flatbed (non-still)

Everything from step 1 to 35 is inside `if (spRepeatParams == 0)`; steps 36+
run unconditionally.

| Step | Function | Wire operation | Address/value (300 dpi colour) | Notes |
|---|---|---|---|---|
| 1 | — | bits(0x0C, bits 3:0, 4) = 0 | reg 0x0C: 0x02 → 0x00 | init table had 0x02 |
| 2 | — | W3(0x7D, 1) | 0x7D=00 0x7E=00 0x7F=01 | TG period parked at 1 while reprogramming |
| 3 | — | host only | `m_stillscanflg = 0; m_ccdpackingflg = 0; m_16bitscanflg = 0; m_ScanPixelRes = 300; gBGRLineSequence = 0; ginADFscan = 0; ADFType = 0; iColorBits = 3 × 8 = 24; CCDLMT = 2 (colour) / 0 (gray); m_ImgLineSize = ((24 × 1296) >> 3) × 3 = 11664` | `m_ImgLineSize = (iColorBits × spScanLinePixels / 8) × 3` — bytes per raw line = 3 segments; equals `dwCISBytesPerLine` |
| 4 | `CtlSensor_SetDummyLine(min(spDummyLines,31)=0)` | bits(0x1D, bits 4:0, 5) = 0 | reg 0x1D: 0x20 → 0x20 | `CtlSensor_nStepPerLine = n + 1 = 1` |
| 5 | `CtlMotor_SetStartStop(spDisBackTrack != 1 → 1)` | bits(0x02, bit6, 1) = (arg == 0) = 0 | reg 0x02: 0x78 → 0x38 | |
| 6 | `WriteColorTiming(spTimingIdx = 4)` | one `0x83` multi-write of 37 pairs (two transfers: 32 pairs = 64 bytes, then 5 pairs) + 8 AFE writes | table in 4.3, row `itm = 2` | idx 1 → row 0, 2 → row 1, 4 → row 2, 8 → row 3 |
| 7 | `CtlImage_SetColorDepth(24, 1)` | bits(0x04, bits 7:4, 4) = `Filter \| LineArt<<3 \| BitSet<<2` = 0 | reg 0x04: 0x02 → 0x02 | 24 → Filter 0, LineArt 0, BitSet 0. 8-bit gray ch1 → Filter 2 (0x20); 16-bit colour → BitSet (0x40); 16-bit gray → 0x60; 1-bit → LineArt (0x80 \| filter) |
| 8 | `CtlSensor_SetDummyPixel()` | W2(0x80, `m_nDummyPixel`) | 0x80=00 0x81=AF (175) | `m_nDummyPixel = (reg21<<8 \| reg22) × 2 + (reg1F<<8 \| reg20) = 0x56×2 + 3` from the shadow |
| 9 | host | `CtlSensor_iTimingIdx = 4; XRate = iOptRes / spPixelResolution = 4.0` | | 0x43/0x85 dpi special cases unused |
| 10 | `CtlSensor_SetScanMargin(0, (0 + 1296 × 4.0) & 0xFFFF = 5184)` | W3(0x82, `begin/idx + dummy`) ; W3(0x85, `that + end/idx − begin/idx`) | 0x82..84 = 00 00 AF (175); 0x85..87 = 00 05 BF (1471) | window in units of `TimingIdx` optical pixels |
| 11 | `CtlSensor_SetDPIHW(1200)` | bits(0x05, bits 7:6, 2) = 1 | reg 0x05: (timing table wrote 0x00) → 0x40 | arg is `CtlSensor_iOptRes`: 300/600 → 0, 1200 → 1, 1600/2400 → 2, 3200/4800 → 3 |
| 12 | `CtlSensor_SetDPISET(300)` | W2(0x2C, `TimingIdx × (DPIHW / XRate)`) = 4 × 1200/4 | 0x2C=04 0x2D=B0 (1200) | = TimingIdx × pixRes; output dpi = DPISET / TimingIdx relative to DPIHW (inferred) |
| 13 | host | `m_iScanDiff = m_iStgDiff = 0; m_iDiffBufIdx = 0` | | packing path dead |
| 14 | `CtlImage_SetScanLength((CCDLMT+1) × (5040 + 0))` | W3(0x25, 15120) | 0x25..27 = 00 3B 10 | colour counts 3 sub-lines per output line |
| 15 | host | `CtlSensor_wTGPeriod = (long)(0.387f × 1000.0 / 0.25) = 1547` | | float32 0.387 is 0.38699999 → 1547, not 1548 |
| 16 | — | W3(0x7D, 1547) | 0x7D..7F = 00 06 0B | **TG (line) period in 0.25 µs ticks** |
| 17 | colour | bits(0x60, bit2, 1) = 0 | reg 0x60: 0x00 | gray sets 1 |
| 18 | colour | W3(0x8A, 1399); W3(0x8D, 1320); W3(0x90, 639) | 0x8A..8C = 00 05 77; 0x8D..8F = 00 05 28; 0x90..92 = 00 02 7F | R/G/B LED on-time in ticks (`(long)(ms × 1000 / 0.25)`); gray encoding in 4.4 |
| 19 | `CtlMotor_SetMotorVref(1,1,1,1)` | reg W 0x3B = 0x55 | | overwritten by sec. 3.3 step 13 |
| 20 | `CtlMotor_SetScanTableUnit(ScanTBunit = 8)` | — | | `06-motor.md` |
| 21 | `CtlMotor_SetFastTableUnit(FastTBunit = 2)` | — | | `06-motor.md` |
| 22 | host | `Final_TG = (long)(nStepPerLine × wTGPeriod / (MotorGear / spMotorResolution)) × (CCDLMT+1) = (1 × 1547 / 2.0) × 3 = 773 × 3 = 2319` | | |
| 23 | `CtlSensor_CalculateTBTIME(2319)` → `CtlSensor_SetTBTime(1)` | bits(0x1C, bits 2:0, 3) = 0 | reg 0x1C: 0x20 → 0x20 | `btmp = (Final_TG >> 16) + 1`, rounded up to 1/2/4/8/16/32 → code 0..5; `CtlSensor_nTBtime = 1` |
| 24 | host | `Final_PPS = (1e6 / 0.25) / 2319 = 1724.9; sn_Scancuvid = (PPS ≤ 2095) = 1` | | |
| 25 | `CtlMotor_LoadPPSCurveToScanTable(MotorTableSet[1]…, 1724.9, 8, 1, &cnt)` | — | | `06-motor.md` |
| 26 | `CtlMotor_LoadPPSCurveToFastTable(MotorTableSet[1]…, 1500.0, 2, 1, &cnt)` | — | | `06-motor.md` (`FastTBPPS` 1500 ≤ 2095 → set 1) |
| 27 | `CtlMotor_CalculateFeedReg(810 / (1200/600) = 405)` | — | | `06-motor.md` |
| 28 | `CtlMotor_SetMotorPowerBit(1)` | bits(0x02, bit4, 1) = 1 | reg 0x02 → 0x38 (already set) | |
| 29 | `CtlMotor_SetZ1Z2Value()` | — | | `06-motor.md` |
| 30 | `CtlImage_SDRamSet_ImageBuffer(1, 0)` | 12 × W2 + 1 reg W | see 4.5 | **always called with ImageType = 1 ("gray" bank layout) even for colour** |
| 31 | `CtlImage_SetMAXWD(11664, 1, 2)` | W3(0x28, 33953) | 0x28..2A = 00 84 A1 | formula in 4.5 |
| 32 | `CtlMotor_EnableAutoGoHome(0)` | bits(0x02, bit5, 1) = 0 | reg 0x02: 0x38 → 0x18 | |
| 33 | `CtlImage_EnableGamma(1)` | bits(0x05, bit3, 1) = 1 | reg 0x05: 0x40 → 0x48 | gamma LUT enable |
| 34 | `CtlImage_EnableShading(1)` | bits(0x01, bit5, 1) = 1; then bits(0x06, bit3, 1) = (ShadingBase != 0x2000) = 1 | reg 0x01: 0x00 → 0x20; reg 0x06: 0x10 → 0x18 | the reg 6 write only happens when enabling |
| 35 | `CtlMotor_SetMotorDirect(1)` | bits(0x02, bit2, 1) = (dir == 0) = 0 | reg 0x02 → 0x18 | |
| 36 | — | bits(0x19, bit0, 1) = 1 | reg 0x19: 0x00 → 0x01 | "assorted bits", first group |
| 37 | — | bits(0x0C, bits 3:0, 4) = CCDLMT = 2 | reg 0x0C → 0x02 | colour line-multiplex count; 0 for gray |
| 38 | — | bits(0x01, bit7, 1) = 1 | reg 0x01 → 0xA0 | |
| 39 | — | bits(0x08, bit4, 1) = 1 | reg 0x08: 0x40 → 0x50 | |
| 40 | (ADF block) | skipped | `ginADFscan == 0` | |
| 41 | — | bits(0x01, bit7, 1) = 1 | 0xA0 | second group (unconditional) |
| 42 | — | bits(0x1E, bit0, 1) = 1 | reg 0x1E: 0x10 → 0x11 | |
| 43 | — | bits(0x19, bit0, 1) = 1 | 0x01 | |
| 44 | — | bits(0x1E, bit2, 1) = 1 | 0x15 | |
| 45 | — | bits(0x08, bit4, 1) = 0 | 0x50 → 0x40 | pulse of bit4 (39→45) |
| 46 | — | bits(0x1B, bit7, 1) = 1 | reg 0x1B: 0x00 → 0x80 | |
| 47 | — | bits(0x19, bit2, 1) = 1 | 0x05 | |
| 48 | host | `m_ImgRemainSize = (5040 + 0) × 11664 = 58 786 560` | | total raw bytes expected |
| 49 | `CtlImage_EnablePattern(0)` | bits(0x06, bits 2:0, 3) = 0 | reg 0x06 → 0x18 | 1/2 would select a test pattern (also writes 0xFE bits 3:0 = 8, 0xFF = 0) |
| 50 | `CtlImage_ClrCounter(1, 1)` | reg W 0x0D = 0x05 | | bit0 = clear scan counter, bit2 = clear feed counter |
| 51 | host | `adfscancnt = 0; sleep1((uint)((CCDLMT+1) × 0.387f) = 1)` | | `sleep1(n)` busy-waits n × 1000 `clock()` ticks ≈ n ms |

Return 1. Control then goes back to sec. 3.3.

### 4.3 `WriteColorTiming` tables (ELF `.data`)

`CCD_ColorASICTiming` @ `0x22B900`, `RegRec[4][37]` (`{ushort nCtrl; uchar
nValue;}`, 4-byte stride, 0x94 bytes per row). **All four rows are
byte-identical.** Row contents (addr = value), written in this order in one
`CMDASIC_WriteMultiRegister` call:

```
05=00 16=00 18=00 1A=0E 1C=20 21=00 22=56 52=08 53=0A 54=04 55=06 56=00 57=02
58=02 59=02 5A=52 5C=40 5F=00 60=00 61=00 70=04 71=06 72=07 73=08 74=00 75=00
76=00 77=00 78=07 79=FC 7A=03 7B=FF 7C=FF 3B=00 30=E0 33=0E 37=00
```

Side effects to be aware of: this zeroes reg 0x05 (DPIHW bits and the gamma
enable are re-set in steps 11/33), writes reg 0x1C = 0x20 (TBTIME bits re-set
in step 23), reg 0x3B = 0 (Vref re-set in step 19 / sec. 3.3), and
0x21/0x22 = 0x00/0x56 (consumed by step 8).

`CCD_ColorAFETiming` @ `0x22BB60`, declared as used `[itm][16]` bytes
(`addr, data` pairs, 8 AFE registers) but the symbol is only **0x30 bytes = 3
rows**; `itm = 3` (TimingIdx 8, never produced by `Scanner_SetScanParameter`)
would read into the following symbol `CurveData1`. Rows:

| itm (TimingIdx) | AFE 0 | AFE 1 | AFE 2 | AFE 3 | AFE 4 | AFE 5 | AFE 6 | AFE 7 |
|---|---|---|---|---|---|---|---|---|
| 0 (idx 1, 2400 dpi) | 0x68 | 0x0B | 0x2F | 0x2F | 0x2F | 0x00 | 0x00 | 0x00 |
| 1 (idx 2, 1200 dpi) | 0x68 | 0x0A | 0x2F | 0x2F | 0x2F | 0x00 | 0x00 | 0x00 |
| **2 (idx 4, 600/300/200 dpi)** | **0x68** | **0x0A** | **0x2F** | **0x2F** | **0x2F** | **0x00** | **0x00** | **0x00** |

(`AFE_INIT_REG` at init is the same with AFE 1 = 0x08.) AFE 2/3/4 are the
R/G/B gains and 5/6/7 the offsets — they are immediately overridden by sec.
3.3 steps 11/12 (gain 0x29, offset 0x80). AFE register 0 = 0x68 and 1 =
0x0A/0x0B are configuration words of an unidentified AFE chip.

### 4.4 Per-dpi derived values

| dpi | iOptRes | TimingIdx | AFE row | XRate | seg px | DPIHW bits (reg 5 7:6) | DPISET (0x2C) | margin end (0x85) | TG colour (0x7D) | TG gray | m_ImgLineSize colour / gray | ScanLength colour (0x25) full bed | Final_TG colour → TBTIME code |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 200 | 1200 | 4 | 2 | 6 | 864 | 01 | 800 | 1471 | 1547 | 4644 | 7776 / 2592 | 3 × 3359 = 10077 | 1545 → 0 |
| **300** | **1200** | **4** | **2** | **4** | **1296** | **01** | **1200** | **1471** | **1547** | **4644** | **11664 / 3888** | **15120** | **2319 → 0** |
| 600 | 2400 | 4 | 2 | 4 | 2592 | 10 | 2400 | 2767 | 2868 | 8604 | 23328 / 7776 | 30240 | 8604 → 0 |
| 1200 | 2400 | 2 | 1 | 2 | 5184 | 10 | 2400 | 5359 | 6731 | 20195 | 46656 / 15552 | 60480 | 40386 → 0 |
| 2400 | 2400 | 1 | 0 | 1 | 10368 | 10 | 2400 | 10543 | 13468 | 40403 | 93312 / 31104 | 120960 | 161616 → code 2 (TB 4) |

Margin begin (0x82) is always `m_nDummyPixel = 175` for full bed. 200 dpi
reuses the 300 dpi window (5184/4 = 1296 groups) with DPISET 800, so the ASIC
must be decimating 1200 → 800 internally; 200 dpi output per segment is 864
px (inferred, unverified).

Shutter registers (ticks = `(long)(ms_f32 × 1000.0 / 0.25)`):

| dpi | mode | reg 0x60 bit2 | 0x8A | 0x8D | 0x90 |
|---|---|---|---|---|---|
| 300/200 | colour | 0 | 1399 | 1320 | 639 |
| 600 | colour | 0 | 2680 | 2680 | 1399 |
| 1200 | colour | 0 | 5260 | 5260 | 2599 |
| 2400 | colour | 0 | 10479 | 10479 | 10479 |
| 300/200 | gray | 1 | 419 | 1079 | 1206 |
| 600 | gray | 1 | 804 | 2144 | 2423 |
| 1200 | gray | 1 | 1578 | 4208 | 4727 |
| 2400 | gray | 1 | 3143 | 8382 | 10477 |

Gray encoding (`spImageType == 1`): with `R, G, B` the raw tick counts,
`0x8A = R×3/10`, `0x8D = R×3/10 + G×5/10`, `0x90 = R×3/10 + G×5/10 + B×2/10`
(integer division after each product) — cumulative end-times of three LED
pulses weighted 0.3/0.5/0.2 inside one line, with reg 0x60 bit2 = 1
selecting that single-line mode. Colour writes the raw counts and bit2 = 0.

### 4.5 SDRAM image buffer and MAXWD (`CtlImage_SDRamSet_ImageBuffer`, `CtlImage_SetMAXWD`)

```
RamSize[(reg0B & 7) - 1] = RamSize[3] = 0x4000          (DRAM_SEL = 4)
chips = 1                                                (reg 8 bit7 = 0, bit1 = 0)
ImageType = 1 (always):  mem_bank_range = (0x4000 × 1 − shd_full_range − 40) >> 1
   iOptRes 1200: (16384 − 96 − 40) / 2 = 8124 (0x1FBC)
   iOptRes 2400: (16384 − 192 − 40) / 2 = 8076 (0x1F8C)
for i in 0..3:  reg[4i]   = shd_full_range + 40
                reg[4i+1] = reg[4i] + range − 1
                reg[4i+2] = reg[4i+1] + 1
                reg[4i+3] = reg[4i+2] + range − 1
written as W2((0x70 + i) × 2, reg[i]) for i in 0..12  → 2-byte regs at 0xE0, 0xE2, … 0xF6
```

300 dpi (iOptRes 1200): `0xE0/E1 = 0x0088, 0xE2/E3 = 0x2043, 0xE4/E5 =
0x2044, 0xE6/E7 = 0x3FFF`, repeated identically at 0xE8..0xEF and 0xF0..0xF7.
Then `reg W 0xF8 = 0x01` (Channel 0; ImageType 0 would write 0x05, channel
1/2 → 0x23/0x45). At iOptRes 2400: `0x00E8, 0x2073, 0x2074, 0x3FFF`.

`CtlImage_SetMAXWD(dwImgLen, ImgType = 1, resv_line = 2)`:

```
base = (reg06 >> 5 == 2) ? 0 : (((mem_bank_range × 1024) >> 8) − 1) × 4     reg 6 >> 5 == 0 here
     = (8124 × 4 − 1) × 4 = 129980          (iOptRes 1200);  (8076 × 4 − 1) × 4 = 129212 (2400)
div  = (ImgType == 0) ? 6.0 : 2.0  → 2.0
W3(0x28, (long)((base + (dwImgLen / 2.0 / div) × resv_line) / 4.0))
```

| dpi (colour) | m_ImgLineSize | reg 0x28..0x2A |
|---|---|---|
| 200 | 7776 | 33467 (0x0082BB) |
| **300** | **11664** | **33953 (0x0084A1)** |
| 600 | 23328 | 35219 (0x008993) |
| 1200 | 46656 | 38135 (0x0094F7) |
| 2400 | 93312 | 43967 (0x00ABBF) |

### 4.6 Gamma LUT (`Scanner_SetGamma(16)` → `CtlImage_SetGamma`, just before `Scanner_SetScanParameter`)

`gamma = 16 / 10 = 1.6`. Three 256-entry `uint16` tables, `t[i] =
(uint16)(int)(pow((i+1) × 256 / 65536, 1/1.6) × 65536)` for i < 255,
`t[255] = 0xFFFF`. Registers: `W2(0xC5, 0), W2(0xC7, 0), W2(0xC9, 0), reg W
0xBD = 0x00, reg W 0xBE = 0x07`, then three bulk writes
(`CMDASIC_WriteBulkData`: `0x82` setup with `bmode = 1`, then bulk OUT) of
512 bytes each to ASIC addresses `0x01000000`, `0x01000200`, `0x01000400`.
`CtlImage_SetDefaultGamma(1)` at init uploads the identity ramp
(`(i+1) × 256`) with the same register writes, so the init state is already a
linear LUT.

### 4.7 Expected shadow/register state after programming (300 dpi colour, HW shading + gamma on)

Useful for read-back verification (`0x8E` reads). Assumes the init table and
no calibration side effects on these registers.

| Reg | Value | Reg | Value | Reg | Value |
|---|---|---|---|---|---|
| 0x01 | 0xA0 | 0x1C | 0x20 | 0x3B | 0x0A |
| 0x02 | 0x18 | 0x1D | 0x20 | 0x3C | 0x0A |
| 0x04 | 0x02 | 0x1E | 0x1D | 0x4F | 0x08 |
| 0x05 | 0x48 | 0x25..27 | 00 3B 10 | 0x60 | 0x00 |
| 0x06 | 0x18 | 0x28..2A | 00 84 A1 | 0x68 / 0x69 | 0x00 / 0x6D |
| 0x08 | 0x40 | 0x2C/2D | 04 B0 | 0x7D..7F | 00 06 0B |
| 0x0C | 0x02 | 0x32 | 0x69 | 0x80/81 | 00 AF |
| 0x0D | 0x05 | 0x33 | 0x0E | 0x82..84 | 00 00 AF |
| 0x12 | 0xE0 | 0x35 | 0x7F | 0x85..87 | 00 05 BF |
| 0x13 | 0x3F | 0x36 | 0x1F | 0x8A..8C | 00 05 77 |
| 0x19 | 0x85 | 0x3A | bit2 = 1 | 0x8D..8F | 00 05 28 |
| 0x1B | 0x80 | | | 0x90..92 | 00 02 7F |

Plus the SDRAM bank registers of 4.5, the 37 timing-table registers of 4.3,
AFE 0..7 = `68 0A 29 29 29 80 80 80`, and reg 3 bit4 = 1 (lamp).

---

## 5. Variants

### 5.1 Running uncorrected: HW shading and gamma disabled

Only two `ScanParameter` fields and the registers they drive differ; nothing
else in sections 3–4 depends on them.

| | vendor default | uncorrected |
|---|---|---|
| `spShading` | 1 | **0** → `CtlImage_EnableShading(0)`: bits(0x01, bit5) = 0 and **no** reg 6 bit3 write (reg 6 stays 0x10) |
| `spGammaMapping` | 1 | **0** → `CtlImage_EnableGamma(0)`: bits(0x05, bit3) = 0 (reg 5 = 0x40) |

With shading off, `Scanner_Calibration2` / `Scanner_Calibration_WriteShadingToDevice`
and the SDRAM shading banks (regs 0xD0..0xD2, set at init) are irrelevant and
the whole calibration pass can be skipped; the scan then uses the sec. 2.4
exposure defaults and AFE gain 0x29 / offset 0x80 verbatim. With gamma off
the LUT upload (4.6) can be skipped too (the init identity LUT is harmless
either way). This is exactly what the vendor's own calibration frames do:
`Scanner_Calibration_SetScanParameter` builds the same struct with
`spShading = 0, spGammaMapping = 0, spScanBeginLine = 0` (and
`spMotorResolution = dpi`).

### 5.2 "Still mode" (`spStillMode = 1`, no carriage motion)

Used by `Scanner_Calibration_Offset2`, `_ExposureTime` and `_DarkShading`
(all call `Scanner_Calibration_SetScanParameter(1, 0, 0, p)`); only
`_WhiteShading` moves the carriage (`Scanner_Calibration_Shading_SetScanParameter
(0, 0, 0, p)`, with `spMotorResolution = 600` fixed and `spMotorFixedSpeed =
1` → `Final_PPS = 464.9`). In `ScanCtrl_SetScanParameter`, `m_stillscanflg
= 1` replaces steps 19–29 as follows:

| Step | Function | Wire operation | Value | Notes |
|---|---|---|---|---|
| 19s | host | `wSlope[0..3] = wTGPeriod / CtlSensor_nTBtime` | 1547 / 1 = 1547 | `nTBtime` must be non-zero: it is only set by `SetTBTime`, which `CarriageHome` (`ScanCtrl_MoveMotor2`) calls with 1 |
| 20s | `CtlMotor_WriteScanSlopeTable(wSlope, 4)` | bulk write 8 bytes (4 × u16 LE) to `0x10000000`, then W2(0xA4, 4); same data to `0x10004000`, W2(0xAA, 4); to `0x10008000`, W2(0xAC, 4) | | slope tables 1/2/3 each a flat 4-entry ramp |
| 27s | `CtlMotor_SetFeedSteps(1)` | W3(0x3D, 1 & 0xFFFFF) | 0x3D..3F = 00 00 01 | instead of `CalculateFeedReg` |
| 27s' | `CtlMotor_EnableFastMove(0)` | bits(0x02, bit3, 1) = 0 | | |
| 28s | `CtlMotor_SetMotorPowerBit(0)` | bits(0x02, bit4, 1) = 0 | reg 2 ends 0x00 (with autohome 0, dir 1) | motor driver unpowered |
| 29s | `CtlMotor_WriteMotorRemainder(0, 0)` | W3(0xB2, 0); W3(0xB5, 0) | | instead of `SetZ1Z2Value` |

Not executed in still mode: `SetMotorVref`, `SetScanTableUnit`,
`SetFastTableUnit`, `CalculateTBTIME` (reg 0x1C untouched),
`LoadPPSCurveTo*`, `CalculateFeedReg`, `SetZ1Z2Value`. Everything else —
timing table, margins, DPI, exposure/shutter, scan length, SDRAM banks,
MAXWD, enables, the "assorted bits", `ClrCounter` — is identical, and
`Api_StartScan` is then issued the same way (`CtlImage_EnableScan(1)` +
`CtlMotor_SetMotorStartMove`; with reg 2 bit4 = 0 the carriage does not
move). So a still-mode scan of N lines is the cleanest experiment for
validating exposure/AFE/geometry on hardware without touching the motor
tables: set `spStillMode = 1`, `spTotalScanLines = N`, read N × 11664 bytes.

Caveat: the still path still calls `WriteColorTiming`, which writes reg 0x3B
= 0 (Vref) and is never restored on that path; `Scanner_Calibration_SetScanParameter`
does not call `SetMotorVref` either, so Vref stays 0 during still frames.

---

## Open questions

1. **Calibration overwrites.** On the vendor path `g_Scan_Parameters.Afe_Info
   .wOffsetR/G/B` and `fExposureTimeR/G/B` are replaced by
   `Scanner_Calibration2` results before `Scanner_SetScanParameter` runs; the
   shutter/offset values in 4.4 and 3.3 are the uncalibrated defaults.
   Capture the vendor driver's actual 0x8A/0x8D/0x90 and AFE 5/6/7 writes on
   hardware to see how far calibration moves them.
2. **Float truncation artefacts**: `dwXExtent = 3593` (not 3594) and `TG =
   1547` (not 1548) rest on Ghidra's float32/double typing of the
   expressions. Both are off-by-one-ish and must be confirmed from the shim's
   debug log (`dwXExtent=%d`) and a USB capture of reg 0x7D.
3. **Reg 0x4F** is never written by the init table, so the vendor's
   read-modify-write of bit3 assumes the other bits are 0. The real power-on
   value is unknown; read it back before the first write.
4. **Semantics** of the per-dpi bits in 0x12, 0x13, 0x19, 0x1B, 0x1E, 0x32,
   0x33, 0x35, 0x36, 0x4F, 0x68/0x69, and of 0x60 bit2, 0x08 bit4 pulse,
   0x0C bits 3:0 (CCDLMT) are inferred from context only.
5. **`CtlDevice_PixelTime` units**: `Clock_Pixel / Clock_ASIC = 12/48 = 0.25`
   makes TG = µs × 4; whether the ASIC's TG clock is really 4 MHz (vs. a
   48 MHz pixel clock with an internal ÷12) is unverified. The register values
   are what matter.
6. **200 dpi** reuses the 300 dpi margin window with DPISET = 800; whether
   the ASIC really outputs 864 px/segment at 200 dpi is unverified.
7. **SANE mm → inch conversion** in `sane_control_option` is a jump table
   Ghidra did not recover; the `/25.4` is inferred from the defaults matching
   the range maxima. `m_nYResolution` is never used by `sane_start` (x dpi is
   passed as both).
8. **`m_iDiffBufLines`** is assumed 0 (bss, only assigned under the dead
   `m_ccdpackingflg` path). If a non-zero value ever appears, scan length,
   `m_ImgRemainSize` and the first `ReadRawImage` in `ScanCtrl_StartScan`
   change.
9. **`CtlSensor_nTBtime`** for still mode relies on an earlier `SetTBTime`
   (via `CarriageHome`); an implementation that skips homing must call
   `CtlSensor_SetTBTime(1)` (bits(0x1C, 2:0) = 0) itself or it divides by
   zero.
10. The AFE chip identity (regs 0 = 0x68, 1 = 0x0A/0x0B) and the exact gain
    /offset scaling of 0x29 / 0x80 are unknown.
