# VF3240 image readout

Status: **verified on hardware (2026-10-13)** for 300 dpi 8-bit colour: data-ready poll, one-line bulk reads, plane order, segment interleave, rotation and crop all produce a correct image. **Gray and 16-bit verified (2026-10-13 evening):** 8-bit gray at 200–2400 dpi, 16-bit colour and gray at 300 dpi and 16-bit colour at 2400 dpi produce correct images with the single-plane / u16 LE layouts and the 1-pixel overlap of §7; the 16-bit output's top byte equals the 8-bit scan of the same page (G mean 158.5 vs 158.6), so the gamma LUT applies in 16-bit mode too.

Source of truth: `libviisanvf3240dr.so` decompiled corpus at
`re/exports/libviisanvf3240dr.so/decomp/` (DWARF-named, compilation units
`MSTScanSDK/scancontrol/gscanctrl.c`, `MSTScanSDK/asiccmd/`,
`MSTScanSDK/Cache.cpp`, `MSTScanSDK/Thread.cpp` and the `Scanner_*` /
`MyScanner_*` layer), plus the SANE shim at
`re/exports/libsane-aviisandrvf3240.so.1.0.27/decomp/`. Initial values of
`.data` globals were read from the vendor ELF
`vendor/linux-sane-1.0.2.0/unpacked/usr/local/lib64/sane/aviisandr-adv/VF3240/libviisanvf3240dr.so`
(section `.data` at VMA `0x22b2a0`; symbol addresses from `objdump -t`).

This document uses the transport terminology of
[`01-usb-transport.md`](01-usb-transport.md) and the opcode table of
[`02-opcode-map.md`](02-opcode-map.md). It covers the span from "scan
started" to a finished RGB (or gray) image in host memory for a **flatbed**
scan. ADF branches are mentioned only where they share code.

## Scope and call graph

The SANE shim resolves exactly these driver exports with `dlsym`
(`loadMSTSDK.c`): `Initialize`, `LoadDriver`, `FreeDriver`, `GetDeviceInfo`,
`GetParameters`, `GetKeyStatus`, `GetCoverStatus`, `SetPowerSaving`,
`StartScan1`, `ReadImage`, `CancelScaner`, `StopScan`. The only scan mode the
shim offers is the string `"Color24"` (`sane_aviisandrvf3240_open.c:176`),
i.e. `dwScanMode = 0`, 8 bits per channel.

```
sane_start   -> StartScan1(mode, dpi, x, y, w, h, colorMatch=1, ...)
                  MyScanner_Open, MyScanner_CarriageHome          (pre-scan, out of scope)
                  MyScanner_SetParameter2  -> geometry globals (section 4.3)
                  MyScanner_GetParameter2  -> g_Block_Buffer_Size := dwCISBytesPerLine
                  MyScanner_StartScan
                    Api_OpenDevice2, Api_InitiateASIC3, calibration   (out of scope)
                    Scanner_SwitchLampPower(1), Scanner_SetGamma
                    Scanner_SetScanParameter -> Api_SetScanParameter -> ScanCtrl_SetScanParameter
                    isFiestGetRows := 1
                    Api_StartScan -> ScanCtrl_StartScan             (section 1)

sane_read    -> ReadImage(lines, nRGBOrder=1, buf, &xfer)
                  MyScanner_ReadScanData
                    [first call] Scanner_ThreadStart                (section 5)
                        thread: Scanner_WriteDataToCache -> Scanner_ReadDataFromScanner
                                  Scanner_Calibration_ReadCISScanDataWithout3CHPack
                                    Api_ReadScanDataWithout3CHPack
                                      ScanCtrl_ReadScanDataWithout3CHPack
                                        CMDASIC_ReadRawImage        (section 2)
                                  Cache_Write
                    Scanner_ReadScanData                            (section 4)
                      Scanner_ReadDataFromCache -> Cache_Read
                      ScanCtrl_Pack_3CH_CIS_Color / _Gray           (section 3/4)
                      rotate, crop, channel order
                then: byte swap in the shim                         (section 4.5)

sane_cancel  -> CancelScaner (sets isCancelScan) ; StopScan         (section 6)
```

Two globals decide which branches are live. Both are set in
`ScanCtrl_SetScanParameter.c:67-68`:

| Global | Value on this device | Effect |
|---|---|---|
| `m_ccdpackingflg` | always `0` (hard-coded) | every `OpenDiffBuf` / `OpenFetchRawImage` / `ReadPackingImage` / `FreeDiffBuf` branch is **dead**; `m_iDiffBufLines` is only assigned inside the dead branch and stays at its `.bss` value `0` |
| `m_16bitscanflg` | `spPixelDepth == 16` | selects the 2-byte paths in `ScanCtrl_StartScan` and the pack functions |

## 1. Start of scan and the data-ready poll

### 1.1 `ScanCtrl_StartScan` (flatbed path, `ginADFscan == 0`)

| Step | Call | Register write | Evidence |
|---|---|---|---|
| 1 | `CtlImage_EnableScan(1)` | reg `0x01` bit 0 := 1 (`CMDASIC_WriteRegisterBit(1, 0, 1, 1)`) | `CtlImage_EnableScan.c` |
| 2 | `CtlMotor_SetMotorStartMove()` | reg `0x0F` := `0xFF` | `CtlMotor_SetMotorStartMove.c` |
| 3 | `ScanCtrl_ChkScanDataReady()` | poll, see 1.2 | `ScanCtrl_StartScan.c` |

If any step returns 0, `ScanCtrl_StartScan` returns 0 immediately, which
propagates unchanged through `Api_StartScan` → `MyScanner_StartScan`
(`isOK = 0`) → `StartScan1`. The shim **ignores the return value of
`StartScan1`** (`sane_aviisandrvf3240_start.c`) and returns
`SANE_STATUS_GOOD` regardless. Our implementation should not.

After step 3 nothing else happens on the flatbed path: the trailing
`m_ccdpackingflg` block (a priming `CMDASIC_ReadRawImage` of
`m_iDiffBufLines` lines) is dead code.

For context, the registers that make the device know *how long* the scan is
were already written at the end of `ScanCtrl_SetScanParameter` (same
compilation unit, immediately before `Api_StartScan`):

| Register | Value | Meaning (inferred from name) | Evidence |
|---|---|---|---|
| `0x25` (3-byte) | `(CCDLMT+1) * spTotalScanLines` = `3 * dwYExtent` for colour, `1 * dwYExtent` for gray | scan length in sensor lines; colour counts one line per colour exposure | `CtlImage_SetScanLength.c`, `ScanCtrl_SetScanParameter.c:193` |
| `0x28` (3-byte) | `CtlImage_SetMAXWD(m_ImgLineSize, 1, 2)` → `(base + ((m_ImgLineSize/2)/2) * 2) / 4` with `base = ((mem_bank_range*0x400 >> 8) - 1) * 4` unless `RegTable[6] >> 5 == 2` | on-device SDRAM image-buffer high-water mark | `CtlImage_SetMAXWD.c`, `ScanCtrl_SetScanParameter.c:400` |
| `0x0D` | `0x05` (`CtlImage_ClrCounter(1,1)`) | clear scan-line and feed counters | `CtlImage_ClrCounter.c`, `ScanCtrl_SetScanParameter.c:528` |
| — | `sleep1((CCDLMT+1) * spExposureTime)` | settle before `Api_StartScan` | `ScanCtrl_SetScanParameter.c:534` |

`CCDLMT` is `2` for colour, `0` for gray (`ScanCtrl_SetScanParameter.c:83-87`),
so `CCDLMT + 1` is the channel count.

### 1.2 `ScanCtrl_ChkScanDataReady` — exact poll loop

Transcribed from `ScanCtrl_ChkScanDataReady.c`:

```
ScanFinish = 0
t0 = gettickcount()
loop A:
    r101 = ReadRegister(0x101)            ; 1 byte, opcode 0x8E (see 02-opcode-map)
    if read failed            -> return 0
    if r101 & 0x10            -> ScanFinish = 1 ; goto phase B (which then returns 1 at once)
    if (r101 & 0x40) == 0     -> goto phase B
    if gettickcount() - t0 >= 60000 -> return 0
    repeat loop A                          ; no sleep between iterations

phase B:
t0 = gettickcount()
loop B:
    if ScanFinish             -> return 1
    r102 = ReadFourRegister(0x102)        ; 4-byte read, opcode 0x8E, wIndex (0x02<<8)|0x22, wValue 0x018E
    if read failed            -> return 0
    if r102 != 0              -> return 1
    if gettickcount() - t0 >= 60000 -> return 0
    repeat loop B                          ; no sleep between iterations
```

Notes:

- Both loops are **tight**: the only pacing is the USB control-transfer
  round trip. There is no `sleep1` in either loop.
- **Timing is CPU time, not wall-clock.** `gettickcount()` is
  `clock() * 1000 / 1000000` (`gettickcount.c`) and `clock()` returns
  process CPU time. Because the loop mostly blocks in the kernel waiting for
  USB completions, the "60000 ms" limit is 60 s of *CPU time consumed*, and
  the wall-clock timeout can be much longer. The same applies to every
  `sleep1(ms)` in this driver — it is a busy-spin on `clock()`
  (`sleep1.c`). An implementation should use a monotonic wall clock and real
  sleeps; 60 s is the intended budget.
- `nR102` is pre-set to `0x200` before each read; it is overwritten on a
  successful read and irrelevant on failure. No semantic meaning.
- Known bits of status register `0x101` (all 1-byte reads, from other
  functions in the corpus):

  | Bit | Mask | Meaning | Evidence |
  |---|---|---|---|
  | 0 | `0x01` | motor moving | `CtlMotor_WaitMotorStop.c`, `Api_GetMotorStatus.c` |
  | 2 | `0x04` | lamp state | `CtlLamp_GetLampState.c` |
  | 3 | `0x08` | carriage at home sensor | `Api_CheckMotorinHome.c` |
  | 4 | `0x10` | **(inferred)** scan finished / scan-length counter reached | only user is `ScanCtrl_ChkScanDataReady` (`ScanFinish = 1`) |
  | 5 | `0x20` | lamp status (`xpasel`) | `Api_GetLampStatus.c` |
  | 6 | `0x40` | **(inferred)** "no image data yet" — while set, keep waiting; when clear, data (or at least a non-zero `0x102`) is expected | only user is `ScanCtrl_ChkScanDataReady` |

- Register `0x102`, read as 4 bytes: **(inferred)** an available-data counter
  (lines or bytes in the SDRAM image buffer). Non-zero means "there is
  something to read". Its unit is unknown; the code only tests for zero.
  Byte order of the 4-byte read is whatever `CMDASIC_ReadFourRegister`
  assembles (see `02-opcode-map.md`, payload `[b3,b2,b1,b0,0x55]`).

Return value for the caller: `1` means "scan is running and at least one
line is (or will shortly be) available"; `0` means a USB error or a 60 s
timeout, and the caller abandons the scan without sending any stop command.

## 2. Bulk image read

### 2.1 `CMDASIC_ReadRawImage(buf, len)`

One call reads exactly `len` bytes of raw image data:

| Step | Transfer | Evidence |
|---|---|---|
| 1 | control OUT, `bRequest 0x0C`, `wValue 0x0082`, `wIndex 0x0000` (`bmode = 0`), 8-byte payload = `u32 LE 0x10000000` ‖ `u32 LE len` | `CMDASIC_SetupBulkLength.c`, `CMDASIC_ReadRawImage.c:10` |
| 2 | bulk IN on EP `0x81`, `len` bytes, timeout `libusb_timeout` = **30000 ms** (`.data` @ `0x22b2a8`) | `CMDIO_BulkRead.c` |

Confirmed: the payload is the `SetupCmd` union with `val[0] = addr`,
`val[1] = len` — two little-endian `uint32` on x86-64. `0x10000000` is a
constant "image FIFO" address; `CMDASIC_ReadBulkData` uses the same opcode
with arbitrary addresses for SDRAM/shading-bank access.

`CMDIO_BulkRead` returns success iff `libusb_bulk_transfer` returned `>= 0`;
`trans_bytes` is **ignored** (see the warning in `01-usb-transport.md`). A
short read is therefore silently accepted by the vendor driver. We must check
`trans_bytes == len`.

### 2.2 Line size: `m_ImgLineSize` and `dwCISBytesPerLine`

```
iColorBits    = (spImageType == 0 ? 3 : 1) * spPixelDepth           ; ScanCtrl_SetScanParameter.c:77-82
m_ImgLineSize = (iColorBits * spScanLinePixels >> 3) * 3            ; ScanCtrl_SetScanParameter.c:89
```

with `spScanLinePixels = dwCISPixelsPerSeg` (`Scanner_SetScanParameter.c`,
every resolution branch). The trailing `* 3` is the **segment count**, not the
channel count. So:

| Mode | `iColorBits` | `m_ImgLineSize` |
|---|---|---|
| colour 8-bit | 24 | `3 * dwCISPixelsPerSeg * 3` = `3 * W` |
| colour 16-bit | 48 | `6 * W` |
| gray 8-bit | 8 | `W` |
| gray 16-bit | 16 | `2 * W` |

where `W = dwCISPixelsPerLine = 3 * dwCISPixelsPerSeg` (section 4.3). This is
identical to `g_Scan_Parameters.dwCISBytesPerLine` computed in
`MyScanner_SetParameter2.c:83-93` (`W*3`, `W`, `W*6`, `W*2` for modes 0, 1,
3, 4). **At 300 dpi colour: `W = 0xF30 = 3888`, `m_ImgLineSize =
dwCISBytesPerLine = 11664` bytes.**

### 2.3 Lines per read and retry policy (`Scanner_ReadDataFromScanner`)

```
linesPerBlock = g_Block_Buffer_Size / dwCISBytesPerLine
```

`g_Block_Buffer_Size` is initialised to `0x10000` in `.data` (`0x22d6c4`),
but `MyScanner_GetParameter2` (called from `StartScan1` before every scan)
overwrites it with `g_Scan_Parameters.dwCISBytesPerLine`
(`MyScanner_GetParameter2.c:35`). Hence **`linesPerBlock == 1`**: the vendor
reads one raw line per bulk transfer (11664 bytes at 300 dpi colour), and
`*pdwMemBufBaseLines` reported to the shim is likewise `1`.

Loop (`Scanner_ReadDataFromScanner.c`), executed on the cache thread:

```
left = dwYExtent ; g_isScanning = 1
while left != 0 and !isCancelScan:
    n = min(left, linesPerBlock)                       ; == 1
    retry = 0
    while !Api_ReadScanDataWithout3CHPack(buf, n):     ; CMDASIC_ReadRawImage(buf, n * m_ImgLineSize)
        retry++ ; if retry > 10: g_isReadScanDataFromScannerFailed = 1 ; goto done   ; 11 attempts total
        sleep1(100)                                    ; busy-wait, CPU-ms
    retry = 0
    while !Cache_Write(&gCacheHandle, buf, n * dwCISBytesPerLine):
        retry++ ; if retry > 10: goto done             ; no sleep
    left -= n
    if g_isScanning == 1: ReadRegister(0x37) -> g_nScanningCoverStatus
done:
    g_isScanning = 0 ; Thread_SetEndSignal(&gThreadHandle)   ; IsStarted = 0
```

Facts that follow:

- **There is no end-of-scan detection on the host.** The thread reads exactly
  `dwYExtent` lines and stops. `m_ImgRemainSize` is decremented in
  `ScanCtrl_ReadScanData*` but never read anywhere in the corpus. Register
  `0x101` bit 4 is consulted only in the start-of-scan poll (1.2). The device
  side was told the line count via reg `0x25` (1.1) and is expected to
  deliver exactly that many lines.
- The **cover read of reg `0x37`** after each line only updates
  `g_nScanningCoverStatus`; bit 2 is "cover open"
  (`MyScanner_GetCoverStatus.c`). It is reported to the application through
  `MyScanner_GetCoverStatus2` while `g_isScanning == 1` (so the app does not
  issue a competing control transfer during a scan) and is **never acted on
  by the driver**. It adds one control read per line and is not needed for
  correctness.
- A retry re-issues the full `SetupBulkLength` + bulk IN for the same line.
  Whether the device re-sends the same line or the next one on retry is not
  determinable statically (open question).
- ADF-only: `ScanCtrl_ReadScanData*` returns `0xFF` (not 0 or 1) once
  `gReadLine >= gCalADFLine`. On the flatbed path it returns only 0 or 1.

### 2.4 `ReadScanData` vs `ReadScanDataWithout3CHPack`

`ScanCtrl_ReadScanData.c` and `ScanCtrl_ReadScanDataWithout3CHPack.c` are
byte-for-byte identical except that the former, after the bulk read, calls
`ScanCtrl_Pack_3CH_CIS_Color(line, m_ImgLineSize, CCDLMT+1, m_16bitscanflg==0)`
(or `_Gray` when `CCDLMT == 0`) on each line in place. The live scan path
uses the `Without3CHPack` variant and performs the identical pack later on
the consumer side in `Scanner_ReadScanData` (section 4.1) with
`channel = 3` (colour) / `1` (gray) and `ByteMode = (dwColorDepth == 8)`.
The ADF line-count preamble in both functions (`APBWriteAddr(0x1100000)`,
`APBReadData`, `ReadTwiRegister(0x43)`) runs only when `ginADFscan != 0`.

## 3. Raw line format

One raw line of `m_ImgLineSize` bytes is **three planes** of `W` samples
each, plane-sequential, then within each plane a segment-interleaved and
**reversed** sample order.

### 3.1 Plane order: B, R, G

| Plane index `p` | Byte range (8-bit) | Colour |
|---|---|---|
| 0 | `[0, W)` | **Blue** |
| 1 | `[W, 2W)` | **Red** |
| 2 | `[2W, 3W)` | **Green** |

Evidence (two independent paths agree):

1. `Scanner_Calibration_WhiteShading.c:69-71` calls
   `Scanner_Calibration_CalcGetShadingDataColor3(lpBuf, pwWhiteShadingR,
   pwWhiteShadingG, pwWhiteShadingB, dwCISPixelsPerLine, ...)`. Inside
   `Color3` (`Scanner_Calibration_CalcGetShadingDataColor3.c:33-38, 53-55`)
   plane 0 (`lpBuf[(i + j*W*3)*2]`) is averaged into `pwData2` (= **B**),
   plane 1 (`+ W`) into `pwData0` (= **R**), plane 2 (`+ 2W`, written as
   `j*W*6 + W*4` in 16-bit byte units) into `pwData1` (= **G**).
   `CalcGetShadingDataColor2` (8-bit dark shading) has the same mapping.
2. `Scanner_Calibration_WriteShadingToDevice.c` passes
   `(pwWhiteShadingB, pwWhiteShadingR, pwWhiteShadingG, pwDarkB, pwDarkR,
   pwDarkG)` to `CtlImage_WriteShadingColor`, which writes them to
   `shd_bank_address[0..2]` in that order — the ASIC's shading banks are
   indexed by raw plane, so bank 0 = B, 1 = R, 2 = G.
3. The pack function (3.3) emits `(plane 1, plane 2, plane 0)` as the output
   pixel, which is `(R, G, B)` under this assignment and nothing else
   sensible.

Sample width: 8-bit mode → 1 byte per sample; 16-bit mode → `u16`
**little-endian** (Color3 assembles `lpBuf[2i] | lpBuf[2i+1] << 8`; the
16-bit pack copies byte pairs `[idx], [idx+1]` in order).

### 3.2 Within-plane sample order

Let `S = W / 3 = dwCISPixelsPerSeg` (segment length) and `W = 3 S`. The
sensor is read out as three segments `s ∈ {0, 1, 2}` of `S` pixels each, one
pixel from each segment per sample clock, and the whole plane is stored
mirrored. Segment `s`, pixel `k` (`0 ≤ k < S`) lives at plane sample index

```
σ(s, k) = W − 3k − (s + 1) = W − 1 − (3k + s)
```

Equivalently, reading the plane **backwards** from its last sample gives
`seg0 k0, seg1 k0, seg2 k0, seg0 k1, seg1 k1, seg2 k1, …`.

This is exactly what `ScanCtrl_Pack_3CH_CIS_Color.c` (8-bit branch) encodes.
With `N = dwLen / channel / bytes = W` and `M = N / 3 = S`:

For output pixel `i` (`0 ≤ i < N`), with segment `s = i div M` and
in-segment pixel `k = i mod M`, the three bytes written are, in order R, G, B:

```
R = plane1[N − 3k − (s+1)]      i.e. ImgBuf[2N − 3k − (s+1)]
G = plane2[N − 3k − (s+1)]      i.e. ImgBuf[3N − 3k − (s+1)]
B = plane0[N − 3k − (s+1)]      i.e. ImgBuf[ N − 3k − (s+1)]
```

There are **no per-segment offsets** other than the `−(s+1)` term; the three
segments are not shifted against each other inside the pack. The 2-pixel
shift of section 4.2 is applied later, to segment 0 only.

### 3.3 Pack result ("packed line")

`ScanCtrl_Pack_3CH_CIS_Color` rewrites the line in place as `W` pixels of
3 bytes (8-bit) or 6 bytes (16-bit), pixel-interleaved. Define packed pixel
`i = s·S + k`:

```
packed[3i + 0] = raw[1·W + σ(s,k)]      ; R
packed[3i + 1] = raw[2·W + σ(s,k)]      ; G
packed[3i + 2] = raw[0·W + σ(s,k)]      ; B
```

16-bit: the same with `u16` units — `packed_u16[3i + c]` ←
`raw_u16[p·W + σ]`, i.e. byte offsets doubled (`4N − 2 − 6i` in the code is
`2·(2N − 1 − 3i)`), bytes copied in order so LE is preserved.

`ScanCtrl_Pack_3CH_CIS_Gray` is the single-plane version:
`packed[i] = raw[σ(s,k)]` (8-bit) / `packed_u16[i] = raw_u16[σ(s,k)]`.

`ScanCtrl_PackCISImg` (a plain plane-to-pixel interleave with no reversal or
segment handling) exists in the corpus but has **no caller** on this path.

## 4. Host-side pipeline: `Scanner_ReadScanData`

All of this runs in the application thread on data pulled from the cache,
one block (= one line) at a time. Variables, from the function prologue:

```
L  = dwCISBytesPerLine            ; raw/packed line length in bytes (11664)
X0 = g_Scan_Parameters.dwNewXPos  ; crop origin, packed-pixel units
XW = g_Scan_Parameters.dwXExtent  ; output width in pixels
```

### 4.1 Pack (per line)

`ScanCtrl_Pack_3CH_CIS_Color(line, L, 3, dwColorDepth == 8)` for
`dwScanMode == 0`, `..._Gray(line, L, 1, ...)` otherwise — exactly section 3.3.

### 4.2 Segment rotation with 2-pixel overlap (per line)

From `Scanner_ReadScanData` (colour branch):

The packed line of `L` bytes is copied to a scratch buffer `tmp`, then
rebuilt as: for `j < (L/3)·2`, `line[j] = tmp[L/3 + j]`; otherwise
`line[j] = tmp[j − (L/3)·2 + 6]` (the gray branch adds 2 instead of 6).

In byte terms: the new line is `tmp[L/3 .. L)` followed by
`tmp[6 .. L/3 + 6)`. Because `L/3 = W` bytes and 8-bit colour pixels are 3
bytes, in **pixel** terms (`S = W/3` pixels per segment):

```
rotated[x] = packed[x + S]           for 0 ≤ x < 2S          (segments 1 and 2, unchanged)
rotated[x] = packed[x − 2S + 2]      for 2S ≤ x < 3S         (segment 0, shifted by 2 pixels)
```

So segment 0 is moved to the end of the line and advanced by two pixels:
packed pixels `0` and `1` (segment 0, `k = 0, 1`) are **dropped**, and
`rotated[3S−2]`, `rotated[3S−1]` are `packed[S]`, `packed[S+1]` — i.e.
**duplicates** of `rotated[0]`, `rotated[1]` (segment 1, `k = 0, 1`). This is
the "2-pixel overlap". Physically the segments therefore sit on the glass in
the order 1, 2, 0 with segment 0 overlapping the start of segment 1 by two
pixels at 300 dpi.

The `+6` (colour) and `+2` (gray) are **constants in the source**, so the
shift is 2 pixels only in 8-bit modes. In 16-bit colour (6 bytes/pixel) and
16-bit gray (2 bytes/pixel) the same constants are **one** pixel. See
section 7 and Open questions.

### 4.3 Crop: `dwNewXPos` and `dwXExtent`

`g_Scan_Parameters.dwNewXPos = dwXPos + dwStartPixelOffset`
(`MyScanner_SetParameter2.c:166`), with the per-resolution constants:

| `dwXRes` (dpi) | `dwStartPixelOffset` | `dwCISPixelsPerSeg` = S | `dwCISPixelsPerLine` = W |
|---|---|---|---|
| 200 (`200 ≤ dpi < 300`) | `0xAA` = 170 | `0x360` = 864 | `0xA20` = 2592 |
| 300 (`300 ≤ dpi < 600`) | `0x100` = 256 | `0x510` = 1296 | `0xF30` = 3888 |
| 600 (`600 ≤ dpi < 1200`) | `0x1FF` = 511 | `0xA20` = 2592 | `0x1E60` = 7776 |
| 1200 (`1200 ≤ dpi < 2400`) | `0x3FD` = 1021 | `0x1440` = 5184 | `0x3CC0` = 15552 |
| 2400 | `0x7F8` = 2040 | `0x2880` = 10368 | `0x7980` = 31104 |

(`MyScanner_SetParameter2.c:48-82`; `dwXRes < 200` is rejected.)
`Scanner_Calibration2` uses different offsets (`0x10B` at 300 dpi) for its
own one-line calibration scans; those are not part of image readout.

The crop copies `XW` pixels starting at rotated pixel `X0`:

```
out[y][x] = rotated[y][X0 + x]        0 ≤ x < XW
```

Nothing checks `X0 + XW ≤ W`; an implementation must. The shim computes
`dwXPos = (11.98 − fRight_in) * dpi` and `dwXExtent = (fRight − fLeft) * dpi`
(`sane_aviisandrvf3240_start.c`), i.e. the **X origin is measured from the
right-hand edge of an 11.98-inch-wide bed**. Whether that means the rotated
line runs right-to-left across the glass can only be settled on hardware
(Open questions).

### 4.4 Channel order (`nRGBOrder`) and output stride

8-bit colour (`Scanner_ReadScanData.c`, inner `j` loop); output stride is
`XW * 3` bytes per line, lines packed contiguously:

| `nRGBOrder` | `out[3x+0]` | `out[3x+1]` | `out[3x+2]` |
|---|---|---|---|
| 0 | `rotated[X0+x].R` | `.G` | `.B` |
| 1 | `rotated[X0+x].B` | `.G` | `.R` |

If `g_isDoColorMatch == 1` (the shim passes `isColorMatch = 1` to
`StartScan1`), `Scanner_ColorMatrixEnhance(outLine, XW, nRGBOrder)` is then
applied to each 8-bit colour line. That is an image-pipeline step (colour
matrix) and is **out of scope** for this document; our core emits the
unprocessed values.

Any other `nRGBOrder` value leaves the output untouched (no `else`).

16-bit colour (stride `XW * 6`): `nRGBOrder == 0` is a `memcpy` of
`rotated[X0 .. X0+XW)` (so `R lo, R hi, G lo, G hi, B lo, B hi`); otherwise
the R and B **words** are swapped (`B lo, B hi, G, G, R lo, R hi`).

Gray 8-bit: `out[x] = rotated[X0 + x]`, stride `XW`. Gray 16-bit: `memcpy`
of `XW*2` bytes from byte offset `X0*2`, stride `XW*2`.

`*pdwXferLines` is always set to `dwWantedLines` on return, even on failure
(`Scanner_ReadScanData.c`, `LAB_0011d9f0`). The function returns 0 only
when the cache stays empty after the reader thread has failed
(`g_isReadScanDataFromScannerFailed == 1`) and more than
`g_dwMaxRetry_ReadScanData` = **2** (`.data` @ `0x22d6c0`) consecutive
cache misses, each preceded by `sleep1(500)`.

### 4.5 The shim's byte swap (`sane_aviisandrvf3240_read`)

The shim calls `ReadImage(lines, nRGBOrder = 1, buf, &xfer)` and then swaps
`buf[3m]` with `buf[3m+2]` for every `m` in `[0, bytesPerLine * xfer / 3)`.
With `nRGBOrder = 1` the driver produced B,G,R; the swap turns it back into
**R,G,B**, which is what SANE's `SANE_FRAME_RGB` expects. (The swap runs
unconditionally; since the shim only offers `Color24`, that is harmless.)

`ReadImage` additionally applies `g_lpTable{R,G,B}` lookup tables when the
scan was started via `StartScan2` with a custom RGB table; `StartScan1` sets
`g_isStartScan2 = 0`, so this never runs for the shim.

### 4.6 Net mapping — what to implement and unit-test

For an **8-bit colour** scan at resolution with segment size `S` and plane
width `W = 3S`, raw line bytes `raw[0 .. 3W)`, and final RGB output
`out[0 .. XW*3)` (after the shim's swap, i.e. the same as `nRGBOrder = 0`):

```
plane(c)  = { R: 1, G: 2, B: 0 }                      (c = output channel)
seg(x), k(x) for rotated pixel x (0 ≤ x < 3S):
    x <  2S          : s = 1 + x / S,   k = x mod S
    2S ≤ x < 3S−2    : s = 0,           k = x − 2S + 2
    x ∈ {3S−2, 3S−1} : s = 1,           k = x − (3S−2)        (duplicates of x = 0, 1)
σ(s, k)   = W − 1 − (3k + s)

out[3·x_out + c] = raw[ plane(c)·W + σ(seg(X0 + x_out), k(X0 + x_out)) ]
```

Closed form per region of `x = X0 + x_out`:

| Region of `x` | `s` | `σ` |
|---|---|---|
| `0 ≤ x < S` | 1 | `W − 2 − 3x` |
| `S ≤ x < 2S` | 2 | `W − 3 − 3(x − S)` |
| `2S ≤ x < 3S − 2` | 0 | `W − 7 − 3(x − 2S)` |
| `x = 3S − 2`, `3S − 1` | 1 | `W − 2`, `W − 5` |

Every one of the `W` raw samples of a plane is used exactly once except
`σ = W−1` and `σ = W−4` (segment 0, `k = 0, 1`), which are dropped, and
`σ = W−2`, `σ = W−5`, which appear twice.

**Worked example, 300 dpi colour: `W = 3888`, `S = 1296`, `L = 11664`.**
Raw byte index of each channel for selected rotated pixels `x` (the
plane base offsets are `B: +0`, `R: +3888`, `G: +7776`):

| rotated `x` | segment `s`, `k` | `σ` | B raw byte | R raw byte | G raw byte |
|---|---|---|---|---|---|
| 0 | 1, 0 | 3886 | 3886 | 7774 | 11662 |
| 1 | 1, 1 | 3883 | 3883 | 7771 | 11659 |
| 1295 | 1, 1295 | 1 | 1 | 3889 | 7777 |
| 1296 | 2, 0 | 3885 | 3885 | 7773 | 11661 |
| 2591 | 2, 1295 | 0 | 0 | 3888 | 7776 |
| 2592 | 0, 2 | 3881 | 3881 | 7769 | 11657 |
| 3885 | 0, 1295 | 2 | 2 | 3890 | 7778 |
| 3886 | 1, 0 (dup of 0) | 3886 | 3886 | 7774 | 11662 |
| 3887 | 1, 1 (dup of 1) | 3883 | 3883 | 7771 | 11659 |

With the shim's default crop at 300 dpi, output pixel `x_out = 0` is rotated
pixel `x = X0 = 256 + dwXPos`; e.g. for `dwXPos = 0`, `out[0..3)` ←
raw bytes `(R 7774−3·256, G 11662−768, B 3886−768)` = `(7006, 10894, 3118)`.

A unit test should build a synthetic raw line where
`raw[p·W + σ] = f(p, s, k)` with an injective `f`, run the implementation,
and check `out` against the table above for `X0 = 0, XW = 3S` and for the
shim's `X0 = 256`.

## 5. The caching thread (summary; not to be replicated)

`MyScanner_ReadScanData` starts the thread lazily on the **first** `ReadImage`
call after `StartScan1` (`isFiestGetRows`), not at start of scan. Between
`Api_StartScan` and that first read the device buffers lines on its own
SDRAM (reg `0x28` MAXWD, section 1.1); nothing on the host paces it.

- `Scanner_ThreadStart` → `Cache_Create(&gCacheHandle, g_Block_Buffer_Size,
  "XT5750HS.tmp")` opens **`/tmp/XT5750HS.tmp`** (`"w+b"`) and
  `Thread_Create` (pthread) runs `Scanner_WriteDataToCache` =
  `Scanner_ReadDataFromScanner` (section 2.3).
- `Cache_Write` always tries `Cache_WriteToFile` first and falls back to
  `Cache_SaveToRam` (malloc) only if the file write fails; blocks are queued
  in `hList_Busy` and recycled via `hList_Idle`. So the whole scan is spooled
  through a temp file, one line per node.
- `Scanner_ReadDataFromCache` loops on `Cache_Read`; on an empty cache it
  `sleep1(1000)`s and retries while either the thread is alive or
  `g_dwTotalUploadLines < g_dwTotalReadLines` (the latter counter is never
  incremented anywhere in the corpus, so effectively "while the thread is
  alive").
- `Scanner_ThreadEnd` → `Thread_Destroy` → `Thread_WaitForEnd` spins
  (`usleep(100)`) until `IsStarted == 0`, i.e. until the reader thread has
  pulled **every** `dwYExtent` line from the device (or failed, or
  `isCancelScan` was set); then `pthread_cancel`, and `Cache_Destroy`
  closes and `remove()`s the temp file.

Device-affecting consequences:

1. The thread issues the per-line reg `0x37` cover read (section 2.3).
2. `StopScan` (section 6) **cannot send anything to the device until all
   lines have been read**, unless `CancelScaner` set `isCancelScan` first
   (the shim's `sane_cancel` does exactly that: `CancelScaner` then
   `StopScan`). In the cancel case the device is left mid-scan with
   un-read lines in its buffer when `EnableScan(0)` arrives.
3. No lamp-off, no motor command and no register write other than `0x37`
   happens on the thread.

For our core: a single-threaded loop "bulk-read one line, reconstruct,
hand to caller" is equivalent as long as the host keeps up with the device's
buffer (MAXWD); whether the ASIC stalls or overruns when the host is slow is
an open question.

## 6. Stop scan

`StopScan` (export) → `MyScanner_StopScan` then `MyScanner_Close` (a
no-op, returns 1). Exact order inside `MyScanner_StopScan.c`:

| # | Call | Device traffic | Notes |
|---|---|---|---|
| 1 | `Scanner_ThreadEnd()` | none (reg `0x37` reads stop) | blocks until the reader thread has finished or was cancelled (section 5). On failure the chain skips to step 6. |
| 2 | `Api_StopScan()` → `ScanCtrl_StopScan()` | reg `0x01` bit 0 := 0 (`CtlImage_EnableScan(0)`) | flatbed: no motor wait here (`CtlMotor_WaitMotorStop` is ADF-only in this function); `FreeDiffBuf` dead. Returns 0 → `isOK = 0`, skip to step 6. |
| 3 | `Api_WaitMotorStop()` → `CtlMotor_WaitMotorStop()` | poll reg `0x101` until bit 0 == 0; tight loop; 60000 CPU-ms limit | return value ignored |
| 4 | `Scanner_ColorMatrixEnhance_Release()` if colour-match | none | host only |
| 5 | `Scanner_CarriageHomeDontWait()` | read reg `0x101`; if bit 3 == 0 (not home) **and** bit 0 == 0 (motor idle): `Api_MoveMotor2(1900.0 pps, 12000 steps, dir 0, unit 4)` | starts a home move and does **not** wait. `ScanCtrl_MoveMotor2` begins with reg `0x38` bit 4 := 1, reg `0x37` bit 4 := 1, `CtlSensor_SetTBTime(1)`, `CtlMotor_SetMotorDirect(0)`, then loads a PPS curve and starts the motor (details belong to the motor document). |
| 6 | `Api_CloseDevice()` → `CMDIO_CloseDevice()` | `libusb_clear_halt(EP 0x02)`, `libusb_clear_halt(EP 0x81)`, **`libusb_reset_device`**, `release_interface(0)`, `close`, `libusb_exit` | a USB port reset while the carriage may still be homing |
| 7 | `Api_OpenDevice2(0x2e5a, 0xd011)` | re-enumerate and claim | |
| 8 | `CMDASIC_WriteRegisterBit(0x3a, 2, 1, 0)`; `(0x35, 1, 1, 1)`; `(0x32, 1, 1, 0)` | reg `0x3A` bit 2 := 0, reg `0x35` bit 1 := 1, reg `0x32` bit 1 := 0 | identical to `MyScanner_SetScanLED(FALSE)` and the tail of `Scanner_CarriageHome`: stops the "scanning" LED blink that `MyScanner_StartScan.c:29` turned on with `0x3A` bit 2 := 1. These are GPIO/LED bits, not lamp control. |
| 9 | `Api_CloseDevice()` | second `libusb_reset_device` + close | |

Steps 6-9 run unconditionally, even when steps 1-2 failed.

What does **not** happen at stop: the lamp is not switched off
(`Scanner_SwitchLampPower` has no caller on the stop path; it is only called
with `1` from `MyScanner_StartScan` and from the calibration routines), the
image counters are not cleared, and `CMDIO_CancelBulkRead` (opcode `0x8D`)
is **never called** anywhere in the corpus. The `isCancelScan` flag set by
`CancelScaner` is purely host-side; it stops the reader thread and the
consumer loop and is reset to 0 by the next `MyScanner_ReadScanData` first
call.

Minimal stop sequence for our core, in order: `EnableScan(0)`; wait for
`0x101` bit 0 clear; optionally start the home move; LED off bits. The two
port resets are almost certainly vendor hygiene rather than protocol, but
that must be verified on hardware (Open questions).

## 7. Gray and 16-bit differences

Everything above applies with these substitutions:

| | Colour 8-bit | Gray 8-bit | Colour 16-bit | Gray 16-bit |
|---|---|---|---|---|
| `dwScanMode` (SetParameter2 arg / stored) | 0 / 0 | 1 / 1 | 3 / 0, `dwColorDepth 16` | 4 / 1, `dwColorDepth 16` |
| `CCDLMT + 1` (channels), reg `0x25` factor | 3 | 1 | 3 | 1 |
| `m_ImgLineSize` = `dwCISBytesPerLine` | `3W` | `W` | `6W` | `2W` |
| Raw line | 3 planes B,R,G × `W` bytes | 1 plane × `W` bytes | 3 planes × `W` u16 LE | 1 plane × `W` u16 LE |
| Pack | `Pack_3CH_CIS_Color(…,3,1)` | `Pack_3CH_CIS_Gray(…,1,1)` | `Pack_3CH_CIS_Color(…,3,0)` | `Pack_3CH_CIS_Gray(…,1,0)` |
| Within-plane order | `σ(s,k)` | `σ(s,k)` | `σ(s,k)` in u16 units | `σ(s,k)` in u16 units |
| Rotation shift of segment 0 | +2 px (`+6` bytes) | +2 px (`+2` bytes) | **+1 px** (`+6` bytes / 6) | **+1 px** (`+2` bytes / 2) |
| Output pixel | `R,G,B` (or `B,G,R`) | 1 byte | `R16,G16,B16` LE (or `B,G,R`) | u16 LE |
| Output stride | `XW*3` | `XW` | `XW*6` | `XW*2` |

The gray plane is the single channel the ASIC produces when
`CtlImage_SetColorDepth(iColorBits, spMonoChannel = 1)` is programmed
(`ScanCtrl_SetScanParameter.c:112`); which physical colour it is (G, or a
mix) is not determinable from this corpus. The shim never uses gray or
16-bit, so those paths have had no end-to-end exercise in the vendor's own
Linux product.

## Open questions

1. **Reg `0x101` bits 4 (`0x10`) and 6 (`0x40`), reg `0x102` (4 bytes):**
   inferred meanings only. On hardware: log `0x101`/`0x102` from
   `EnableScan(1)` until the first bulk read returns, and again after the
   last line, to pin "scan finished" and the counter's unit.
2. **Bulk-read blocking semantics.** Does the device hold the bulk IN until a
   full line is available (so the host needs no `0x102` pacing), or does it
   return short/NAK? `trans_bytes` is ignored by the vendor; we must check
   it and find out whether short reads occur at the last line.
3. **Retry semantics.** After a failed bulk read, does a repeated
   `SetupBulkLength(0x10000000, len)` deliver the same line again or the
   next one? Determines whether the vendor's 10-retry loop can drop lines.
4. **Host-slower-than-device behaviour.** Flatbed uses `MAXWD` with
   `resv_line = 2`. Does the ASIC pause the motor when the SDRAM buffer
   reaches MAXWD, or overrun? Our single-threaded reader depends on this.
5. **Mirroring / X origin.** The shim measures `dwXPos` from the right edge
   of an 11.98 in bed. Verify on hardware whether the reconstructed line
   (section 4.6) is left-to-right or needs a horizontal flip, and whether
   11.98 in is the true scan width (`W/dpi` = 12.96 in at 300 dpi, of which
   `dwStartPixelOffset` = 256 px are skipped on the left of the rotated line).
6. **The 2-pixel overlap.** Confirm on a ruler scan that segment boundaries at
   rotated `x = S` (seg 1→2) and `x = 2S` (seg 2→0) are seamless with the
   `+2` shift, and that the duplicated pixels at `x = 3S−2, 3S−1` are
   outside any usable crop. Also whether the overlap is really 1 px in 16-bit
   modes or the `+6`/`+2` constants are a vendor bug there.
7. **Meaning of `0x10000000`** in `SetupBulkLength` for image reads versus
   the SDRAM/shading-bank addresses used by `CMDASIC_ReadBulkData`.
8. **Are the two `libusb_reset_device` calls at stop needed?** Try the
   minimal stop sequence of section 6 and check that a second scan still
   starts cleanly.
9. **`0x102` initial `0x200`** and **`ReadFourRegister` byte order** (which
   end is the LSB of the 4-byte value) — needed only if we ever use the
   counter's value, not its zero test.
10. **Gray channel identity** and 16-bit sample endianness on the wire
    (inferred LE from the host code only).
11. **CPU-time clocks.** All vendor timeouts (60 s poll, 30 s is libusb
    wall-clock) and sleeps are in process CPU time; real wall-clock limits
    for a slow-moving carriage at 2400 dpi must be measured.
