# VF3240 motor and lamp control

Status: **partially verified on hardware (2026-10-13):** standalone moves in both directions work as described, and the ASIC stops a toward-home move when the home sensor trips (§3.2, §8.1, open question 2). Scan-driven motion from the §5 slope tables is verified at 200 (CurveData1, 2584 pps), 300, 600, 1200 and 2400 dpi, including the `nTBtime = 4` (code 2) case the 2400 dpi `Final_TG` forces. Everything else is still static inference.

## Source of truth

`libviisanvf3240dr.so`, decompiled corpus at
`re/exports/libviisanvf3240dr.so/decomp/` (one file per function; function
addresses in `functions.txt`, recovered types in `types.h`). Compilation units
(from DWARF): `MSTScanSDK/asiccontrol/gimotor.c` (`CtlMotor_*`),
`gilamp.c` (`CtlLamp_*`), `gisensor.c` (`CtlSensor_*`),
`scancontrol/gscanctrl.c` (`ScanCtrl_*`), `api/asicapi.c` (`Api_*`), and the
vendor's SANE-side wrappers `Scanner_*` / `MyScanner_*` (`MyScanner.cpp`).
Data tables (`CurveData1`, `CurveData2`, `MotorTableSet`, `ASIC_INIT_REG`,
tuning globals) were read from the `.data` section of
`vendor/linux-sane-1.0.2.0/unpacked/usr/local/lib64/sane/aviisandr-adv/VF3240/libviisanvf3240dr.so`
using the symbol table (`objdump -t`) and `struct.unpack` on the raw bytes.

This document uses the transport vocabulary of
[`01-usb-transport.md`](01-usb-transport.md) and the opcode table of
[`02-opcode-map.md`](02-opcode-map.md). Register writes below are
`CMDASIC_WriteRegister` (opcode `0x83`), multi-byte register writes are
`CMDASIC_WriteTwiRegister` / `CMDASIC_WriteTriRegister` (big-endian byte
order into consecutive registers, issued through `CMDASIC_WriteMultiRegister`,
opcode `0x83`), bit writes are `CMDASIC_WriteRegisterBit(addr, startbit,
nbits, value)` (read-modify-write on the host-side shadow
`CMDASIC_RegTable[addr].nValue`, then a full-byte write), and bulk table
uploads are `CMDASIC_WriteBulkData` (opcode `0x82` setup + bulk OUT on
EP `0x02`).

Everything in this file is static inference. Where the vendor code is
internally inconsistent or where units cannot be closed from the binary alone,
the item is listed under **Open questions** at the end and must be settled on
the device.

---

## 1. Status registers

### 1.1 Register `0x101` (8-bit status, read with `CMDASIC_ReadRegister`)

Wire: control IN, `wValue = 0x018E` (opcode `0x8E` with address bit 8 set),
`wIndex = (0x01 << 8) | 0x22 = 0x0122`, 2-byte reply `[value, 0x55]`.

| Bit | Mask | Meaning | Evidence |
|---|---|---|---|
| 0 | `0x01` | **Motor moving** (1 = moving). | `Api_GetMotorStatus`: `*iState = (nVal & 1) != 0`. `CtlMotor_WaitMotorStop` polls until this bit is 0. |
| 2 | `0x04` | **Lamp on** (1 = lamp currently on). | `CtlLamp_GetLampState`: `*iStatus = (nVal & 4) != 0`. |
| 3 | `0x08` | **Carriage at home** (1 = home sensor active). | `Api_CheckMotorinHome`: `*iState = (nVal & 8) != 0`. |
| 4 | `0x10` | **Scan finished** (inferred name). | `ScanCtrl_ChkScanDataReady`: if set, the wait loop exits immediately with `ScanFinish = 1` and skips the `0x102` poll. |
| 6 | `0x40` | **Busy / no scan data yet** (inferred name). | `ScanCtrl_ChkScanDataReady`: loop continues while this bit is 1; when it clears, the function proceeds to poll `0x102`. |
| 1, 5, 7 | | Not referenced anywhere in the corpus. | — |

Only five functions read `0x101`: `Api_GetMotorStatus`, `Api_CheckMotorinHome`,
`CtlLamp_GetLampState`, `CtlMotor_WaitMotorStop`, `ScanCtrl_ChkScanDataReady`
(exhaustive `grep 'ReadRegister(0x101'`).

### 1.2 Register `0x102` (32-bit, read with `CMDASIC_ReadFourRegister`)

Confirmed from `CMDASIC_ReadFourRegister.c`:

```
wValue  = 0x018E                       // 0x8E | (addr bit 8) << 8
wIndex  = (uint16)((0x102 << 8) | 0x22) = 0x0222   // address low byte 0x02 in the high byte of wIndex
len     = 5, device -> host
reply   = [b3, b2, b1, b0, 0x55]       // str[4] must be 'U' (0x55) or the read fails
value   = b3<<24 | b2<<16 | b1<<8 | b0  // reply bytes are big-endian
```

(The decompiler shows `*(uchar*)dwResult = str[3]`, `+1 = str[2]`, `+2 = str[1]`,
`+3 = str[0]` on a little-endian host, i.e. `str[0]` is the most significant byte.)

The only reader is `ScanCtrl_ChkScanDataReady`, which treats **any non-zero
value** as "scan data available" (inferred: a byte or line count of buffered
image data). Semantics of the value itself are not used by the driver.

### 1.3 Wait loops and timeouts

| Function | Polls | Exit condition | Timeout | Sleep between polls |
|---|---|---|---|---|
| `CtlMotor_WaitMotorStop` (= `Api_WaitMotorStop`) | `0x101` | bit 0 == 0 → success | 60 000 ms (`59999 < now - start` → fail) | **none** — tight loop, one control read per iteration |
| `ScanCtrl_ChkScanDataReady` phase 1 | `0x101` | bit 4 set → success (ScanFinish) ; bit 6 clear → go to phase 2 | 60 000 ms | none |
| `ScanCtrl_ChkScanDataReady` phase 2 | `0x102` (4-byte) | value != 0 → success (skipped if ScanFinish) | 60 000 ms | none |

Both timeouts use `gettickcount()`, which is **`clock() * 1000 / CLOCKS_PER_SEC`,
i.e. process CPU time in ms, not wall-clock time**. Because the loop is blocked
in `libusb_control_transfer` most of the time, CPU time advances much more
slowly than wall time, so the vendor's "60 s" is in practice an unbounded wait.
The same is true of `sleep1(ms)`, a busy-wait on `clock()`. A reimplementation
should use a monotonic wall clock and a short sleep (a few ms) between polls.

The libusb transfer timeout (`libusb_timeout`, `.data`) is **30 000 ms** for
every control and bulk transfer.

---

## 2. Register map used by the motor and lamp code

All addresses are 9-bit ASIC register addresses. Bit fields are `startbit:nbits`
as passed to `CMDASIC_WriteRegisterBit`. Power-on values come from
`ASIC_INIT_REG` (117 entries, uploaded by `CtlDevice_LoadInitRegData*` via
`CMDASIC_WriteMultiRegister` — see section 2.1). "Init" = value in that table,
"—" = not in the table (register not initialised by the driver; its default is
whatever the ASIC powers up with).

| Reg | Field | Written by | Meaning | Init |
|---|---|---|---|---|
| `0x01` | bit 0 | `CtlImage_EnableScan(b)` | Scan enable. 1 = start capturing lines when the motor starts. | `0x00` |
| `0x02` | bit 0 | `CtlMotor_EnableLongCurve(b)` | "Long curve" enable. **Never called** in this binary. | `0x78` |
| `0x02` | bit 2 | `CtlMotor_SetMotorDirect(iDir)` | Direction. Written as `iDir == 0`, so **bit2 = 1 ⇒ move toward home**, **bit2 = 0 ⇒ move away from home (scan direction)**. | (bit2 = 0) |
| `0x02` | bit 3 | `CtlMotor_EnableFastMove(b)` | 1 = use the *fast* slope table (`0x1000c000`/`0x10010000`, regs `0xae`/`0xb0`) for the feed phase; 0 = feed with the scan table. | (bit3 = 1) |
| `0x02` | bit 4 | `CtlMotor_SetMotorPowerBit(b)` | Motor driver power / hold-current enable. | (bit4 = 1) |
| `0x02` | bit 5 | `CtlMotor_EnableAutoGoHome(b)` | 1 = ASIC returns the carriage to home automatically when the scan completes. | (bit5 = 1) |
| `0x02` | bit 6 | `CtlMotor_SetStartStop(b)` | Written as `b == 0`. The caller passes `spDisBackTrack != 1`, so bit6 = 1 means "back-tracking disabled". Scan path sets **bit6 = 0** (back-tracking on). | (bit6 = 1) |
| `0x03` | bit 4 | `CtlLamp_SetLampOnOff(b)` | Main lamp on/off. | `0x9f` (bit4 = 1, lamp on) |
| `0x03` | bit 5 | `CtlLamp_SetXPLampOnOff(b)` | "XP" (transparency/second) lamp select. Also read by `Api_GetLampStatus` as `xpasel`. | (bit5 = 0) |
| `0x03` | bits 0..3 | `CtlLamp_SetLampWatchDog` | Lamp watchdog time value (see 7.4). | `0xf` |
| `0x03` | bit 7 | `CtlLamp_SetLampWatchDog` | Lamp watchdog enable. | 1 |
| `0x05` | bits 4..5 | `CtlLamp_SetLampWatchDog` | Lamp watchdog time unit (0 = ×1 min, 1 = ×2 min, 2 = ×4 min). | `0x40` → unit 0 |
| `0x09` | bit 3 | `CtlImage_SetScanForward(f)` | Written as `f == 0`. Image-direction flag; not called from the motor/scan path. | `0x08` |
| `0x0a` | bit 4 | `CtlLamp_EnableLampPWM(b)` | Lamp PWM enable. | `0x40` (bit4 = 0) |
| `0x0c` | bits 0..3 | `ScanCtrl_SetScanParameter` | `CCDLMT`: number of line exposures per output line minus 1 (`2` for colour = R,G,B sequential; `0` for grey). Read back by `CtlMotor_SetZ1Z2Value`. | `0x02` |
| `0x0f` | byte | `CtlMotor_SetMotorStartMove()` | **Motor start trigger**: write `0xff`. | — |
| `0x1c` | bits 0..2 | `CtlSensor_SetTBTime(t)` | Time-base divider code for the slope tables: `t` ∈ {1,2,4,8,16,32} → code {0,1,2,3,4,5}. Host keeps `CtlSensor_nTBtime = t`. | `0x00` |
| `0x1d` | bits 0..4 | `CtlSensor_SetDummyLine(n)` | Dummy lines; host keeps `CtlSensor_nStepPerLine = n + 1`. | `0x20` |
| `0x32` | bit 1 | `Scanner_CarriageHome`, `MyScanner_StopScan`, `MyScanner_SetScanLED` | GPIO: front-panel LED control (inferred from `MyScanner_SetScanLED`). Not motor hardware. | `0x69` |
| `0x35` | bit 1 | same | GPIO: LED control (inferred). | `0x7f` |
| `0x3a` | bit 2 | same | GPIO: LED "flash" (inferred: `MyScanner_SetScanLED(1)` sets it, `(0)` clears it). | — |
| `0x37` | bit 4 | `ScanCtrl_MoveMotor2` | Set to 1 before a standalone move. Meaning unknown. | `0x00` |
| `0x37` | bit 5 | `CtlMotor_SetScanTableUnit`, `CtlMotor_SetFastTableUnit` | 1 only for step unit 16 (extends the 2-bit unit code). | (0) |
| `0x38` | bit 4 | `ScanCtrl_MoveMotor2` | Set to 1 before a standalone move. Meaning unknown. | `0xe0` |
| `0x38` | bit 5 | `CtlMotor_Set*TableUnit` | Always written 1 whenever a table unit is set. Meaning unknown. | (1) |
| `0x3b` | bits 7..6 / 5..4 / 3..2 / 1..0 | `CtlMotor_SetMotorVref(gohome, ffast, ssback, fscan)` | Motor Vref (current) selectors, 2 bits each: `gohome` (go-home move), `ffast` (fast feed), `ssback` (back-track), `fscan` (scan). `-1` = leave field unchanged. | — |
| `0x3c` | same layout | `CtlMotor_SetMotorVref_2(...)` | Second Vref register, identical layout (inferred: second phase/driver). | — |
| `0x3d..0x3f` | 20-bit, big-endian (`0x3d` = bits 19..16, `0x3e` = 15..8, `0x3f` = 7..0) | `CtlMotor_SetFeedSteps(n)` | **Feed step count** for a move (`n & 0xfffff`). | `0x000001` |
| `0x7d..0x7f` | 24-bit BE | `ScanCtrl_SetScanParameter` | TG (line exposure) period in pixel clocks = `CtlSensor_wTGPeriod`. | `0x0055f0` |
| `0xa0` | bits 0..2 | `CtlMotor_SetScanTableUnit` | Scan-table step-unit code (see 4.3). | — |
| `0xa0` | bits 3..5 | `CtlMotor_SetFastTableUnit` | Fast-table step-unit code. | — |
| `0xa2:0xa3` | 16-bit BE | `CtlMotor_CalculateFeedReg` | Written with `m_iScanTableUnit`. Read back by `CalculateFeedReg` and `SetZ1Z2Value` (as "b_value" when fast move is on). Register name unknown. | `0x0001` |
| `0xa4:0xa5` | 16-bit BE | `CtlMotor_WriteScanSlopeTable` | Scan slope table 1 length (entries). | `0x0004` |
| `0xa6:0xa7` | 16-bit BE | (never written by this driver) | Read by `CtlMotor_SetZ1Z2Value` for Z1. | `0x00f0` (240) |
| `0xaa:0xab` | 16-bit BE | `CtlMotor_WriteScanSlopeTable` | Scan slope table 2 length. | `0x0004` |
| `0xac:0xad` | 16-bit BE | `CtlMotor_WriteScanSlopeTable` | Scan slope table 3 length. | `0x0001` |
| `0xae:0xaf` | 16-bit BE | `CtlMotor_WriteFastSlopeTable` | Fast slope table 1 length. | `0x0004` |
| `0xb0:0xb1` | 16-bit BE | `CtlMotor_WriteFastSlopeTable` | Fast slope table 2 length. | `0x0001` |
| `0xb2..0xb4` | 24-bit BE | `CtlMotor_WriteMotorRemainder(z1, z2)` | Z1 remainder (motor/TG phase alignment, see 5.6). | — |
| `0xb5..0xb7` | 24-bit BE | `CtlMotor_WriteMotorRemainder` | Z2 remainder. | — |
| `0xfa:0xfb` | 16-bit values written as two 2-byte registers | `CtlLamp_SetLampPWMDuration(on, off)` | PWM on / off duration. **Note the overlap**: `WriteTwiRegister(0xfa, on)` writes `0xfa,0xfb`; `WriteTwiRegister(0xfb, off)` writes `0xfb,0xfc`. The second write overwrites `0xfb`. Vendor bug or 8-bit registers; unresolved. | — |
| `0x101` | status | read-only | See section 1. | |
| `0x102` | 32-bit status | read-only | See section 1. | |

### 2.1 Power-on register image (`ASIC_INIT_REG`, 117 entries)

Uploaded as one `CMDASIC_WriteMultiRegister` batch (payload = 117 × `RegRec{ushort addr; uchar value}` packed as 4-byte records, `0x1d4` bytes) by `CtlDevice_LoadInitRegData` / `..2` / `..3`. `..3` (used by the real scan path, `Api_InitiateASIC3(xres, yres)`) selects `ASIC_INIT_REG_GPIO111` for 2400×2400 and 1200×2400, `ASIC_INIT_REG_GPIO000` for 1200×1200, else `ASIC_INIT_REG`; the three tables differ **only** in register `0x32` (`0x69` / `0x61` / `0x7d`) — no motor or lamp register differs. Full table (addr=value, hex):

```
001=00 002=78 003=9f 004=02 005=40 006=10 008=00 009=08 00a=40 00b=8a 00c=02
011=00 012=00 013=1f 014=30 016=00 017=00 018=08 019=00 01a=2e 01b=00 01c=00
01d=20 01e=10 01f=00 020=03 021=00 022=56 025=00 026=00 027=00 028=00 029=40
02a=00 02b=f0 02c=04 02d=b0 02e=80 02f=80 030=e0 031=00 033=0f 032=69 034=00
035=7f 036=0f 037=00 038=e0 039=00 03d=00 03e=00 03f=01 052=0b 053=01 054=03
055=05 056=07 057=09 058=09 059=09 05a=1b 05b=00 05c=80 05f=30 060=00 061=01
07d=00 07e=55 07f=f0 080=00 081=3c 082=00 083=00 084=14 085=00 086=27 087=ec
088=00 089=52 08a=00 08b=07 08c=d0 08e=0f 08f=a0 090=00 091=17 092=70 093=00
094=00 095=00 096=00 097=00 098=00 09b=7f 09c=7f 09f=00 0a1=90 0a2=00 0a3=01
0a4=00 0a5=04 0a6=00 0a7=f0 0a8=00 0a9=f0 0aa=00 0ab=04 0ac=00 0ad=01 0ae=00
0af=04 0b0=00 0b1=01 0b8=00 0b9=00 0bd=00 0be=00
```

Motor-relevant consequences: after init, reg `0x02 = 0x78` (fast-move, motor
power, **auto-go-home and back-track-disable all set**, direction = away from
home), reg `0x03 = 0x9f` (**lamp on**, watchdog enabled, value 15), `0x3b`/`0x3c`
(Vref) and `0xa0` (units) are left at ASIC defaults, and `0x0f` is never
initialised. After the batch, `LoadInitRegData*` also writes
`0x34[2:2]=0, 0x34[5:2]=0, 0x35[0]=0, 0x35[1]=1` (GPIO, inferred LED).

### 2.2 Clock constants

`CtlDevice_LoadInitRegData*` calls `CtlDevice_SetScannerClock(Clock_ASIC)` with
`Clock_ASIC = 48` and `CtlDevice_SetScanMOD(Clock_Pixel)` with `Clock_Pixel = 12`
(both `.data` constants). `SetScannerClock` computes the host-side constant

```
CtlDevice_PixelTime = Clock_Pixel / Clock_ASIC = 12 / 48 = 0.25     // microseconds per pixel clock
```

(and writes reg `0x0b = (old & 0x1f) | 0x60` plus USB-local register `0x10 = 0x0d`
for the 48 MHz case). So the **pixel clock is 4 MHz**, and `1e6 / PixelTime =
4 000 000` pixel clocks per second is the constant that appears in every
pps conversion below. `CtlDevice_SetScanMOD(12)` writes reg `0x14[0] = 0` and
reg `0x06[5:3] = 0`.

---

## 3. Homing

### 3.1 Decision logic — `Scanner_CarriageHome`

Called by `MyScanner_CarriageHome` (which wraps it in `Api_OpenDevice2(0x2e5a,
0xd011)` … `Api_CloseDevice()`), which is called from `Initialize`, `StartScan1`
and `StartScan2` before every scan.

```
home   = Api_CheckMotorinHome()        // 0x101 bit 3
if read failed or home == 1: return    // already at home: do nothing
moving = Api_GetMotorStatus()          // 0x101 bit 0
if moving == 0:
    Api_InitiateASIC()                 // re-upload ASIC_INIT_REG, AFE regs, clocks, SDRAM, gamma (full re-init!)
    0x3a[2] = 0 ; 0x35[1] = 1 ; 0x32[1] = 0     // LED GPIO (inferred)
    Scanner_SwitchLampPower(0)         // lamp off if it was on (section 7)
    Api_MoveMotor2(1900.0, 12000, 0, 4)          // move toward home, see 3.2
    Api_WaitMotorStop()                // poll 0x101 bit 0, "60 s"
if moving == 1:                        // (re-evaluated after the above; only reached when the carriage was already moving)
    0x3a[2] = 0 ; 0x35[1] = 1 ; 0x32[1] = 0
    Api_WaitMotorStop()
```

Note the quirk: after the `moving == 0` branch `nMotorStatus` is still 0, so the
`moving == 1` branch never runs in the same call; the two branches are mutually
exclusive. Also note there is **no second home-sensor check after the move**: the
function returns success as soon as the motor-moving bit clears, whether or not
bit 3 is now set.

`Scanner_CarriageHomeDontWait` is the same without re-init, LED, lamp or wait:
`if !home && !moving: Api_MoveMotor2(1900.0, 12000, 0, 4)`. It is called at the
end of `MyScanner_StopScan`.

`Scanner_CarriageHome_Calibration2File` = `if !home && !moving { MoveMotor2(1900,
12000, 0, 4); WaitMotorStop }` (no re-init, no lamp).

`MyScanner_CheckCarriageAtHome` = open device, `Api_CheckMotorinHome`, close,
return bit 3.

### 3.2 The home move — `Api_MoveMotor2(1900.0, 12000, 0, 4)`

Direction `0` = toward home (reg `0x02` bit 2 = 1). Step unit 4, target speed
1900 pps, 12000 steps. The complete register sequence is given in section 4
(`ScanCtrl_MoveMotor2`); with these arguments the concrete values are:

| Step | Wire operation | Address / value | Notes |
|---|---|---|---|
| 1 | bit write | `0x38[4] = 1` | |
| 2 | bit write | `0x37[4] = 1` | |
| 3 | bit write | `0x1c[0:3] = 0` | `CtlSensor_SetTBTime(1)`; `nTBtime = 1` |
| 4 | bit write | `0x02[2] = 1` | direction: toward home |
| 5 | bit writes | `0x38[5] = 1`, `0x37[5] = 0`, `0xa0[3:3] = 2` | `CtlMotor_SetFastTableUnit(4)` → code 2 |
| 6 | reg write | `0x3b = (old & 0x0f) \| 0xa0` then `0x3b = 0x00`, `0x3c = 0x00` | `SetMotorVref(2,2,-1,-1)` immediately overwritten by `SetMotorVref(0,0,0,0)` and `SetMotorVref_2(0,0,0,0)`. Net: both Vref registers = 0. |
| 7 | bulk write | `0x1000c000`, 720 bytes | fast slope table, 360 × u16 LE (CurveData2 entries < 1900 → 359 words, + target word, already a multiple of 4). Words = `1e6 / pps` (unit 4): first `[0xffff, 43478, 24390, 16666, …]`, last `526`. |
| 8 | 2-reg write | `0xae:0xaf = 0x0168` (360) | |
| 9 | bulk write | `0x10010000`, same 720 bytes | |
| 10 | 2-reg write | `0xb0:0xb1 = 0x0168` | |
| 11 | bit write | `0x02[3] = 1` | fast move on |
| 12 | 3-reg write | `0x3d:0x3e:0x3f = 0x00 0xb8 0xb0` | feed = `4 × 12000 − 2 × 360 = 47280` |
| 13 | bit write | `0x02[4] = 1` | motor power on |
| 14 | reg write | `0x0f = 0xff` | **start** |

Then the caller polls `0x101` bit 0 until clear.

**How does this stop at the home position?** 12000 steps × unit 4 is far longer
than the A3 bed (section 8); the driver issues no "stop" command and the move
returns success only when bit 0 clears. The only mechanism consistent with the
code is that the ASIC **terminates a toward-home move itself when the home
sensor (`0x101` bit 3) trips**. This is inferred, not observed, and is the single
most important thing to verify on hardware before any other motor command is
issued (see section 8).

---

## 4. Standalone motor moves — `Api_MoveMotor` / `Api_MoveMotor2`

`Api_MoveMotor(fpps, Steps, iDir)` → `ScanCtrl_MoveMotor` (step unit fixed at 2,
Vref argument 1). `Api_MoveMotor2(fpps, Steps, iDir, nStepUnit)` →
`ScanCtrl_MoveMotor2` (caller-chosen unit, Vref argument 2, plus the `0x37`/`0x38`
bit-4 writes and a short-move fix-up). Neither waits; callers follow with
`Api_WaitMotorStop` (`Scanner_MotorMove` does both).

`iDir`: **0 = toward home, 1 = away from home.** Evidence: homing uses 0;
`MyScanner_MotorMoveTest` homes, then `Api_MoveMotor(5200, 0x4d58, 1)`,
`WaitMotorStop`, `sleep1(150)`, `Api_MoveMotor(5200, 24000, 0)` (a long move back,
again relying on the home sensor to stop it); `ScanCtrl_SetScanParameter` sets
direction 1 for a scan; `Scanner_Calibration2` moves `Scanner_MotorMove(800.0,
0xe1 (300 dpi) or 0xfa, 1)` away from home onto the calibration strip.

### 4.1 `ScanCtrl_MoveMotor2(fpps, Steps, iDir, nStepUnit)` — full sequence

| Step | Function | Wire operation | Address / value | Notes |
|---|---|---|---|---|
| 1 | — | bit write | `0x38[4] = 1` | `MoveMotor2` only |
| 2 | — | bit write | `0x37[4] = 1` | `MoveMotor2` only |
| 3 | `CtlSensor_SetTBTime(1)` | bit write | `0x1c[0:3] = 0` | `CtlSensor_nTBtime = 1` |
| 4 | `CtlMotor_SetMotorDirect(iDir)` | bit write | `0x02[2] = (iDir == 0)` | |
| 5 | select curve | — | `crID = (fpps <= 2095.0) ? 1 : 0` | `MotorTableSet[0]` = CurveData1 (1000 entries, max 9019 pps), `[1]` = CurveData2 (510 entries, max 2096 pps) |
| 6 | `CtlMotor_LoadPPSCurveToFastTable(curve, len, fpps, unit, Vref, &usstep)` | see 4.3 | `0x38[5]`, `0x37[5]`, `0xa0[3:3]`, `0x3b`, `0x3c`, two bulk writes, `0xae`, `0xb0` | `usstep` = number of table words written |
| 6a | `MoveMotor2` only | — | `nMoveStep = nStepUnit*Steps − 2*usstep`; if `< 1`, reload the table with a lower target `curve[(nStepUnit*(Steps−1)) >> 1]` (so the accel+decel ramps fit inside the move) | |
| 7 | `CtlMotor_EnableFastMove(1)` | bit write | `0x02[3] = 1` | |
| 8 | `CtlMotor_SetFeedSteps(unit*Steps − 2*usstep)` | 3-reg write | `0x3d..0x3f = (n & 0xfffff)` big-endian | **Feed count excludes the ramps**: the ASIC runs the ramp table up, feeds this many unit-steps at the final speed, then runs the table down. |
| 9 | `CtlMotor_SetMotorPowerBit(1)` | bit write | `0x02[4] = 1` | |
| 10 | `CtlMotor_SetMotorStartMove()` | reg write | `0x0f = 0xff` | start |

`ScanCtrl_MoveMotor` is steps 3–10 with `unit = 2`, `Vref = 1`, no 6a.
Note that in `ScanCtrl_MoveMotor` (unit 2) `usstep * -2` is subtracted without a
lower bound, so a short move can produce a negative (wrapped, masked to 20 bits)
feed count — do not copy that.

### 4.2 Step unit encoding — `CtlMotor_SetScanTableUnit(n)` / `CtlMotor_SetFastTableUnit(n)`

Both first write `0x38[5] = 1`, then:

| n | Scan table (`0xa0[0:3]`, `0x37[5]`) | Fast table (`0xa0[3:3]`, `0x37[5]`) |
|---|---|---|
| 1 | code 0, `0x37[5]=0` | code 0, `0x37[5]=0` |
| 2 | code 1, `0x37[5]=0` | **code 2**, `0x37[5]=0` (decompiled `case 2:` falls through into `case 4:` — no `break`) |
| 4 | code 2, `0x37[5]=0` | code 2, `0x37[5]=0` |
| 8 | code 3, `0x37[5]=0` | code 3, `0x37[5]=0` |
| 16 | code 3, `0x37[5]=1` | code 3, `0x37[5]=1` |
| other | code 1 (default `tt`), `0x37[5]` unchanged | code 1, `0x37[5]` unchanged |

Host-side `m_iScanTableUnit = n` / `m_iFastTableUnit = n` are stored as given
(so the fast-table unit-2 case stores 2 but programs code 2, which the scan
table would use for unit 4). `FastTBunit` is 2 for every scan, so this
fall-through is exercised on every scan; whether the hardware then runs the
fast table at unit 4 (and the feed distance is double what the host computed)
is an open question (section 9).

The "unit" is the number of table entries (micro-steps) per motor step as the
host counts them: table words are computed per unit-step (section 4.3), feed
counts are multiplied by the unit, and `CalculateFeedReg` divides table lengths
by the unit to get whole steps.

### 4.3 PPS curve → slope table — `CtlMotor_LoadPPSCurveTo{Scan,Fast}Table`

Signature: `(float *curve, int curveLen, float targetPPS, int stepUnit, int Vref,
int *retstep)`. Both variants do:

1. `CtlMotor_Set{Scan,Fast}TableUnit(stepUnit)` (4.2).
2. Vref: **Scan** variant: `SetMotorVref(0,0,0,0)` → `0x3b = 0`; `SetMotorVref_2(0,0,0,0)` → `0x3c = 0`.
   **Fast** variant: `SetMotorVref(Vref, Vref, -1, -1)` → `0x3b = (old & 0x0f) | Vref<<6 | Vref<<4`, then `SetMotorVref(0,0,0,0)` → `0x3b = 0`, then `0x3c = 0`. (The Vref argument is therefore dead; both registers end at 0.)
3. Scan variant only: refuse (`return 0`) if `targetPPS > curve[curveLen-1]`.
4. Build the u16 table:

```
unit = m_iScanTableUnit (scan) or m_iFastTableUnit (fast)   // as stored by step 1
tb   = CtlSensor_nTBtime                                     // 1, 2, 4, 8, 16 or 32
pixel_time = CtlDevice_PixelTime                             // 0.25 µs

word(pps) = trunc( ((1e6 / pps) / pixel_time) / tb / unit )  // pixel clocks per unit-step, divided by tb
          = trunc( 4_000_000 / (pps * tb * unit) )           // with the VF3240 clocks

i = 0
while i < curveLen and curve[i] < targetPPS:
    tbl[i] = min(word(curve[i]), 0xFFFF)      // ramp entries are clamped to 16 bits
    i += 1
tbl[i] = word(targetPPS)                     // NOT clamped (ushort truncation of int)
i += 1
// pad with copies of the last word until the count is a multiple of StepTim (= 4)
for k in 0 .. StepTim-1:
    if (i + k) % StepTim == 0: i += k; break
    tbl[i + k] = tbl[i - 1]
count = min(i, maxMotorStep)                 // maxMotorStep = 8192
```

   The ramp therefore contains every curve entry strictly below the target
   speed, in curve order (ascending pps = descending period), followed by the
   target period, padded to a multiple of 4 entries. Because the curve is
   ascending, the table is a deceleration-ordered list of periods; the ASIC runs
   it forward to accelerate and backward to decelerate (inferred from the
   `2*usstep` subtraction in the feed count).

5. Scan variant only: `CtlMotor_CalculateTableSum(tbl, count)`:
   `m_iTableSum = tb * Σ tbl[0..count-1]`, `m_iTableHigh = tb * tbl[count-1]`
   (used by Z1/Z2, section 5.6).
6. `CtlMotor_Write{Scan,Fast}SlopeTable(tbl, count)` (4.4).
7. `*retstep = count`.

`StepTim = 4` and `maxMotorStep = 8192` are `.data` constants.

### 4.4 Table upload — `CtlMotor_WriteScanSlopeTable` / `CtlMotor_WriteFastSlopeTable`

Each bulk write is `CMDASIC_WriteBulkData(addr, bytes, len)` =
`CMDASIC_SetupBulkLength(addr, len, bmode = 1)` — control OUT, `wValue = 0x0082`,
`wIndex = 0x0001`, 8-byte payload `{u32 addr, u32 len}` little-endian — followed by
`CMDIO_BulkWrite(bytes, len)` = `libusb_bulk_transfer(EP 0x02)`. Confirmed from
`CMDASIC_WriteBulkData.c`, `CMDASIC_SetupBulkLength.c`, `CMDIO_BulkWrite.c`.
The table payload is the host `ushort[]` array as-is, i.e. **u16 little-endian**
(x86-64 host), `count * 2` bytes.

| Table | Function | Bulk target (`m_w0?SlopeTblAddr`) | Length register (16-bit BE, value = `count`) |
|---|---|---|---|
| Scan 1 | `WriteScanSlopeTable` | `0x10000000` | `0xa4:0xa5` |
| Scan 2 | `WriteScanSlopeTable` | `0x10004000` | `0xaa:0xab` |
| Scan 3 | `WriteScanSlopeTable` | `0x10008000` | `0xac:0xad` |
| Fast 1 | `WriteFastSlopeTable` | `0x1000c000` | `0xae:0xaf` |
| Fast 2 | `WriteFastSlopeTable` | `0x10010000` | `0xb0:0xb1` |

Order within each function: bulk, length reg, bulk, length reg, (bulk, length
reg). The **same** table is written to all three scan slots (and both fast slots);
the three scan slots are presumably accel / const / decel or R/G/B-phase tables in
the ASIC, but the driver never differentiates them. Slot spacing (`0x4000` bytes =
8192 u16) matches `maxMotorStep = 8192`.

`CtlMotor_WriteMotorPhaseTable` (64 × u16 phase pattern to `0x01000a00`, 128
bytes) and `CtlMotor_EnableLongCurve` exist but have **no callers** in this
binary; they are not part of the protocol as exercised.

### 4.5 Speed tables

`MotorTableSet` (`.data`, `struct MotorTableCfg { int TbLength; float *fPPSTable;
int Vref; int UseStepUnit; }`, 2 entries):

| Index | TbLength | fPPSTable | Vref | UseStepUnit | Selected when |
|---|---|---|---|---|---|
| 0 | 1000 | `CurveData1` | 1 | 2 | target pps > 2095.0 |
| 1 | 510 | `CurveData2` | 1 | 2 | target pps ≤ 2095.0 |

`Vref` and `UseStepUnit` fields are never read by the code. Both curves are
strictly increasing and every value is an integer stored as `float`.

#### `CurveData2` — 510 floats (pps), used for all scans and for homing (`.data` at `0x22cb40`)

| index | +0 | +1 | +2 | +3 | +4 | +5 | +6 | +7 | +8 | +9 |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 5 | 23 | 41 | 60 | 78 | 96 | 115 | 132 | 150 | 168 |
| 10 | 187 | 205 | 223 | 242 | 260 | 281 | 302 | 323 | 345 | 366 |
| 20 | 388 | 409 | 432 | 457 | 483 | 508 | 526 | 544 | 560 | 576 |
| 30 | 592 | 607 | 621 | 635 | 649 | 662 | 675 | 688 | 700 | 713 |
| 40 | 724 | 736 | 747 | 758 | 769 | 780 | 791 | 801 | 811 | 821 |
| 50 | 831 | 840 | 850 | 859 | 868 | 878 | 886 | 895 | 904 | 912 |
| 60 | 921 | 929 | 937 | 945 | 953 | 961 | 969 | 977 | 984 | 992 |
| 70 | 999 | 1006 | 1013 | 1020 | 1026 | 1033 | 1039 | 1046 | 1052 | 1058 |
| 80 | 1065 | 1071 | 1077 | 1083 | 1089 | 1095 | 1101 | 1106 | 1112 | 1118 |
| 90 | 1124 | 1129 | 1135 | 1140 | 1146 | 1151 | 1157 | 1162 | 1167 | 1173 |
| 100 | 1178 | 1183 | 1188 | 1193 | 1198 | 1203 | 1208 | 1213 | 1218 | 1223 |
| 110 | 1228 | 1233 | 1237 | 1242 | 1247 | 1252 | 1256 | 1261 | 1265 | 1270 |
| 120 | 1274 | 1279 | 1283 | 1288 | 1292 | 1296 | 1301 | 1305 | 1309 | 1314 |
| 130 | 1318 | 1322 | 1326 | 1330 | 1334 | 1338 | 1342 | 1346 | 1350 | 1354 |
| 140 | 1358 | 1362 | 1366 | 1370 | 1374 | 1378 | 1382 | 1385 | 1389 | 1393 |
| 150 | 1397 | 1400 | 1404 | 1408 | 1411 | 1415 | 1419 | 1422 | 1426 | 1429 |
| 160 | 1433 | 1436 | 1440 | 1443 | 1447 | 1450 | 1454 | 1457 | 1460 | 1464 |
| 170 | 1467 | 1471 | 1474 | 1477 | 1480 | 1484 | 1487 | 1490 | 1493 | 1497 |
| 180 | 1500 | 1503 | 1506 | 1509 | 1512 | 1515 | 1519 | 1522 | 1525 | 1528 |
| 190 | 1531 | 1534 | 1537 | 1540 | 1543 | 1546 | 1549 | 1552 | 1555 | 1557 |
| 200 | 1560 | 1563 | 1566 | 1569 | 1572 | 1575 | 1577 | 1580 | 1583 | 1586 |
| 210 | 1589 | 1591 | 1594 | 1597 | 1600 | 1602 | 1605 | 1608 | 1610 | 1613 |
| 220 | 1616 | 1618 | 1621 | 1624 | 1626 | 1629 | 1631 | 1634 | 1636 | 1639 |
| 230 | 1642 | 1644 | 1647 | 1649 | 1652 | 1654 | 1657 | 1659 | 1661 | 1664 |
| 240 | 1666 | 1669 | 1671 | 1674 | 1676 | 1678 | 1681 | 1683 | 1685 | 1688 |
| 250 | 1690 | 1693 | 1695 | 1697 | 1699 | 1702 | 1704 | 1706 | 1709 | 1711 |
| 260 | 1713 | 1715 | 1718 | 1720 | 1722 | 1724 | 1727 | 1729 | 1731 | 1733 |
| 270 | 1735 | 1737 | 1740 | 1742 | 1744 | 1746 | 1748 | 1750 | 1752 | 1754 |
| 280 | 1757 | 1759 | 1761 | 1763 | 1765 | 1767 | 1769 | 1771 | 1773 | 1775 |
| 290 | 1777 | 1779 | 1781 | 1783 | 1785 | 1787 | 1789 | 1791 | 1793 | 1795 |
| 300 | 1797 | 1799 | 1801 | 1803 | 1805 | 1807 | 1808 | 1810 | 1812 | 1814 |
| 310 | 1816 | 1818 | 1820 | 1822 | 1823 | 1825 | 1827 | 1829 | 1831 | 1833 |
| 320 | 1834 | 1836 | 1838 | 1840 | 1842 | 1843 | 1845 | 1847 | 1849 | 1851 |
| 330 | 1852 | 1854 | 1856 | 1858 | 1859 | 1861 | 1863 | 1864 | 1866 | 1868 |
| 340 | 1870 | 1871 | 1873 | 1875 | 1876 | 1878 | 1880 | 1881 | 1883 | 1885 |
| 350 | 1886 | 1888 | 1890 | 1891 | 1893 | 1895 | 1896 | 1898 | 1899 | 1901 |
| 360 | 1903 | 1904 | 1906 | 1907 | 1909 | 1911 | 1912 | 1914 | 1915 | 1917 |
| 370 | 1918 | 1920 | 1921 | 1923 | 1925 | 1926 | 1928 | 1929 | 1931 | 1932 |
| 380 | 1934 | 1935 | 1937 | 1938 | 1940 | 1941 | 1943 | 1944 | 1946 | 1947 |
| 390 | 1948 | 1950 | 1951 | 1953 | 1954 | 1956 | 1957 | 1958 | 1960 | 1961 |
| 400 | 1963 | 1964 | 1966 | 1967 | 1968 | 1970 | 1971 | 1973 | 1974 | 1975 |
| 410 | 1977 | 1978 | 1979 | 1981 | 1982 | 1984 | 1985 | 1986 | 1988 | 1989 |
| 420 | 1990 | 1992 | 1993 | 1994 | 1996 | 1997 | 1998 | 2000 | 2001 | 2002 |
| 430 | 2003 | 2005 | 2006 | 2007 | 2009 | 2010 | 2011 | 2013 | 2014 | 2015 |
| 440 | 2016 | 2018 | 2019 | 2020 | 2021 | 2023 | 2024 | 2025 | 2026 | 2027 |
| 450 | 2029 | 2030 | 2031 | 2032 | 2034 | 2035 | 2036 | 2037 | 2039 | 2040 |
| 460 | 2041 | 2042 | 2043 | 2045 | 2046 | 2047 | 2048 | 2049 | 2050 | 2052 |
| 470 | 2053 | 2054 | 2055 | 2056 | 2057 | 2058 | 2060 | 2061 | 2062 | 2063 |
| 480 | 2064 | 2066 | 2067 | 2068 | 2069 | 2070 | 2071 | 2072 | 2073 | 2074 |
| 490 | 2075 | 2077 | 2078 | 2079 | 2080 | 2081 | 2082 | 2083 | 2084 | 2085 |
| 500 | 2086 | 2087 | 2088 | 2090 | 2091 | 2092 | 2093 | 2094 | 2095 | 2096 |

#### `CurveData1` — 1000 floats (pps), used only when the target speed exceeds 2095 pps (`.data` at `0x22bba0`)

| index | +0 | +1 | +2 | +3 | +4 | +5 | +6 | +7 | +8 | +9 |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 15 | 31 | 46 | 67 | 83 | 103 | 123 | 144 | 164 | 185 |
| 10 | 206 | 226 | 247 | 266 | 287 | 307 | 328 | 348 | 368 | 389 |
| 20 | 409 | 428 | 448 | 468 | 488 | 508 | 528 | 547 | 567 | 587 |
| 30 | 607 | 627 | 646 | 666 | 685 | 704 | 723 | 743 | 762 | 782 |
| 40 | 801 | 820 | 839 | 858 | 877 | 897 | 916 | 935 | 954 | 973 |
| 50 | 992 | 1010 | 1029 | 1048 | 1067 | 1085 | 1104 | 1123 | 1142 | 1161 |
| 60 | 1179 | 1198 | 1216 | 1235 | 1253 | 1271 | 1289 | 1307 | 1326 | 1344 |
| 70 | 1362 | 1380 | 1399 | 1417 | 1435 | 1452 | 1470 | 1488 | 1506 | 1524 |
| 80 | 1542 | 1560 | 1577 | 1595 | 1612 | 1630 | 1647 | 1666 | 1683 | 1700 |
| 90 | 1718 | 1735 | 1752 | 1771 | 1788 | 1805 | 1821 | 1839 | 1857 | 1873 |
| 100 | 1890 | 1907 | 1924 | 1942 | 1959 | 1975 | 1992 | 2009 | 2026 | 2043 |
| 110 | 2059 | 2076 | 2093 | 2110 | 2126 | 2142 | 2160 | 2176 | 2192 | 2209 |
| 120 | 2226 | 2242 | 2258 | 2275 | 2291 | 2307 | 2323 | 2340 | 2355 | 2371 |
| 130 | 2388 | 2404 | 2420 | 2436 | 2452 | 2468 | 2484 | 2500 | 2515 | 2532 |
| 140 | 2547 | 2563 | 2579 | 2594 | 2610 | 2626 | 2641 | 2657 | 2672 | 2687 |
| 150 | 2704 | 2719 | 2734 | 2750 | 2765 | 2780 | 2796 | 2811 | 2827 | 2841 |
| 160 | 2857 | 2871 | 2886 | 2902 | 2917 | 2932 | 2947 | 2961 | 2977 | 2991 |
| 170 | 3007 | 3021 | 3036 | 3051 | 3066 | 3080 | 3095 | 3110 | 3124 | 3139 |
| 180 | 3153 | 3168 | 3182 | 3197 | 3211 | 3226 | 3240 | 3255 | 3269 | 3284 |
| 190 | 3297 | 3312 | 3326 | 3341 | 3354 | 3369 | 3383 | 3397 | 3411 | 3425 |
| 200 | 3439 | 3453 | 3466 | 3481 | 3495 | 3508 | 3523 | 3536 | 3551 | 3564 |
| 210 | 3578 | 3591 | 3605 | 3619 | 3632 | 3646 | 3659 | 3673 | 3686 | 3700 |
| 220 | 3714 | 3727 | 3740 | 3753 | 3767 | 3780 | 3793 | 3807 | 3819 | 3833 |
| 230 | 3847 | 3860 | 3873 | 3885 | 3898 | 3912 | 3924 | 3938 | 3951 | 3964 |
| 240 | 3977 | 3990 | 4002 | 4015 | 4028 | 4041 | 4054 | 4066 | 4079 | 4092 |
| 250 | 4104 | 4117 | 4130 | 4142 | 4155 | 4167 | 4180 | 4192 | 4205 | 4218 |
| 260 | 4230 | 4242 | 4255 | 4266 | 4279 | 4292 | 4304 | 4316 | 4329 | 4341 |
| 270 | 4352 | 4365 | 4377 | 4389 | 4401 | 4413 | 4426 | 4438 | 4449 | 4461 |
| 280 | 4473 | 4485 | 4497 | 4509 | 4521 | 4533 | 4544 | 4556 | 4568 | 4580 |
| 290 | 4590 | 4602 | 4614 | 4626 | 4638 | 4649 | 4661 | 4673 | 4684 | 4695 |
| 300 | 4706 | 4718 | 4730 | 4741 | 4752 | 4763 | 4775 | 4786 | 4798 | 4809 |
| 310 | 4820 | 4831 | 4842 | 4854 | 4865 | 4876 | 4887 | 4898 | 4909 | 4920 |
| 320 | 4932 | 4942 | 4953 | 4964 | 4975 | 4986 | 4997 | 5008 | 5019 | 5029 |
| 330 | 5040 | 5051 | 5062 | 5072 | 5083 | 5094 | 5104 | 5115 | 5126 | 5136 |
| 340 | 5147 | 5158 | 5168 | 5179 | 5190 | 5200 | 5210 | 5220 | 5231 | 5241 |
| 350 | 5251 | 5262 | 5273 | 5283 | 5293 | 5304 | 5314 | 5323 | 5334 | 5344 |
| 360 | 5354 | 5364 | 5374 | 5385 | 5395 | 5405 | 5415 | 5425 | 5435 | 5445 |
| 370 | 5455 | 5465 | 5475 | 5485 | 5495 | 5504 | 5514 | 5524 | 5534 | 5544 |
| 380 | 5553 | 5563 | 5573 | 5582 | 5592 | 5602 | 5611 | 5621 | 5631 | 5640 |
| 390 | 5650 | 5659 | 5669 | 5678 | 5688 | 5698 | 5707 | 5716 | 5726 | 5735 |
| 400 | 5745 | 5754 | 5763 | 5772 | 5782 | 5791 | 5800 | 5809 | 5819 | 5828 |
| 410 | 5838 | 5847 | 5856 | 5866 | 5875 | 5884 | 5893 | 5902 | 5911 | 5920 |
| 420 | 5929 | 5938 | 5947 | 5956 | 5965 | 5973 | 5982 | 5991 | 6000 | 6009 |
| 430 | 6018 | 6027 | 6035 | 6044 | 6053 | 6062 | 6070 | 6079 | 6088 | 6096 |
| 440 | 6105 | 6114 | 6123 | 6131 | 6140 | 6149 | 6157 | 6165 | 6174 | 6182 |
| 450 | 6191 | 6200 | 6208 | 6216 | 6225 | 6233 | 6241 | 6249 | 6258 | 6266 |
| 460 | 6275 | 6283 | 6292 | 6300 | 6308 | 6316 | 6324 | 6333 | 6341 | 6349 |
| 470 | 6357 | 6365 | 6373 | 6381 | 6390 | 6398 | 6406 | 6414 | 6422 | 6429 |
| 480 | 6438 | 6446 | 6454 | 6462 | 6470 | 6478 | 6485 | 6494 | 6501 | 6509 |
| 490 | 6517 | 6525 | 6533 | 6541 | 6548 | 6556 | 6564 | 6571 | 6580 | 6587 |
| 500 | 6595 | 6602 | 6609 | 6618 | 6626 | 6633 | 6640 | 6648 | 6656 | 6664 |
| 510 | 6671 | 6678 | 6685 | 6694 | 6701 | 6708 | 6715 | 6723 | 6731 | 6738 |
| 520 | 6745 | 6752 | 6760 | 6767 | 6775 | 6782 | 6789 | 6796 | 6803 | 6811 |
| 530 | 6819 | 6826 | 6833 | 6840 | 6847 | 6854 | 6861 | 6868 | 6876 | 6883 |
| 540 | 6890 | 6897 | 6904 | 6911 | 6918 | 6925 | 6932 | 6939 | 6946 | 6953 |
| 550 | 6960 | 6967 | 6973 | 6980 | 6988 | 6994 | 7001 | 7008 | 7015 | 7022 |
| 560 | 7028 | 7036 | 7042 | 7049 | 7057 | 7063 | 7069 | 7076 | 7083 | 7089 |
| 570 | 7096 | 7103 | 7109 | 7116 | 7123 | 7130 | 7136 | 7142 | 7149 | 7156 |
| 580 | 7162 | 7169 | 7175 | 7181 | 7188 | 7195 | 7201 | 7208 | 7214 | 7220 |
| 590 | 7227 | 7233 | 7239 | 7246 | 7253 | 7259 | 7266 | 7271 | 7277 | 7284 |
| 600 | 7290 | 7296 | 7303 | 7309 | 7315 | 7322 | 7328 | 7334 | 7341 | 7347 |
| 610 | 7353 | 7359 | 7365 | 7371 | 7378 | 7383 | 7390 | 7395 | 7401 | 7408 |
| 620 | 7414 | 7420 | 7426 | 7432 | 7438 | 7444 | 7450 | 7456 | 7462 | 7467 |
| 630 | 7474 | 7480 | 7485 | 7491 | 7497 | 7503 | 7509 | 7514 | 7520 | 7526 |
| 640 | 7533 | 7538 | 7543 | 7550 | 7555 | 7561 | 7567 | 7572 | 7578 | 7584 |
| 650 | 7590 | 7595 | 7601 | 7607 | 7612 | 7618 | 7624 | 7629 | 7635 | 7640 |
| 660 | 7646 | 7651 | 7657 | 7662 | 7667 | 7674 | 7679 | 7685 | 7690 | 7695 |
| 670 | 7701 | 7706 | 7712 | 7717 | 7723 | 7728 | 7733 | 7739 | 7744 | 7750 |
| 680 | 7755 | 7761 | 7766 | 7771 | 7777 | 7782 | 7787 | 7792 | 7798 | 7803 |
| 690 | 7809 | 7813 | 7819 | 7824 | 7829 | 7834 | 7839 | 7845 | 7850 | 7855 |
| 700 | 7860 | 7866 | 7870 | 7876 | 7880 | 7885 | 7891 | 7895 | 7901 | 7905 |
| 710 | 7911 | 7916 | 7921 | 7926 | 7932 | 7936 | 7942 | 7946 | 7951 | 7956 |
| 720 | 7961 | 7966 | 7971 | 7976 | 7980 | 7986 | 7990 | 7995 | 8000 | 8005 |
| 730 | 8010 | 8015 | 8019 | 8024 | 8029 | 8034 | 8038 | 8044 | 8048 | 8053 |
| 740 | 8057 | 8063 | 8067 | 8072 | 8077 | 8082 | 8086 | 8091 | 8095 | 8100 |
| 750 | 8105 | 8110 | 8114 | 8119 | 8123 | 8128 | 8133 | 8137 | 8142 | 8146 |
| 760 | 8151 | 8155 | 8160 | 8165 | 8170 | 8174 | 8179 | 8183 | 8188 | 8192 |
| 770 | 8197 | 8201 | 8205 | 8209 | 8214 | 8219 | 8223 | 8228 | 8232 | 8236 |
| 780 | 8240 | 8245 | 8249 | 8254 | 8258 | 8263 | 8266 | 8271 | 8276 | 8280 |
| 790 | 8285 | 8288 | 8293 | 8296 | 8301 | 8305 | 8309 | 8314 | 8318 | 8323 |
| 800 | 8326 | 8331 | 8335 | 8339 | 8343 | 8348 | 8352 | 8356 | 8360 | 8364 |
| 810 | 8369 | 8372 | 8376 | 8380 | 8385 | 8389 | 8393 | 8397 | 8401 | 8405 |
| 820 | 8409 | 8413 | 8418 | 8421 | 8425 | 8429 | 8434 | 8438 | 8441 | 8446 |
| 830 | 8449 | 8453 | 8457 | 8461 | 8466 | 8469 | 8473 | 8476 | 8481 | 8485 |
| 840 | 8488 | 8493 | 8496 | 8500 | 8504 | 8508 | 8512 | 8515 | 8520 | 8523 |
| 850 | 8527 | 8531 | 8534 | 8538 | 8542 | 8546 | 8550 | 8553 | 8557 | 8561 |
| 860 | 8565 | 8569 | 8572 | 8576 | 8580 | 8583 | 8587 | 8590 | 8594 | 8598 |
| 870 | 8602 | 8606 | 8609 | 8613 | 8617 | 8620 | 8624 | 8627 | 8630 | 8634 |
| 880 | 8638 | 8641 | 8645 | 8648 | 8652 | 8656 | 8659 | 8663 | 8666 | 8670 |
| 890 | 8674 | 8677 | 8681 | 8685 | 8687 | 8691 | 8695 | 8698 | 8702 | 8705 |
| 900 | 8708 | 8712 | 8715 | 8719 | 8722 | 8725 | 8729 | 8733 | 8736 | 8739 |
| 910 | 8742 | 8746 | 8750 | 8752 | 8756 | 8760 | 8762 | 8766 | 8770 | 8772 |
| 920 | 8776 | 8779 | 8782 | 8786 | 8789 | 8792 | 8796 | 8799 | 8802 | 8806 |
| 930 | 8809 | 8812 | 8815 | 8819 | 8822 | 8825 | 8828 | 8831 | 8835 | 8838 |
| 940 | 8841 | 8844 | 8847 | 8850 | 8854 | 8857 | 8860 | 8863 | 8866 | 8869 |
| 950 | 8873 | 8876 | 8878 | 8882 | 8885 | 8888 | 8891 | 8895 | 8897 | 8900 |
| 960 | 8904 | 8906 | 8910 | 8913 | 8915 | 8919 | 8922 | 8924 | 8928 | 8931 |
| 970 | 8933 | 8936 | 8940 | 8942 | 8945 | 8949 | 8952 | 8954 | 8958 | 8961 |
| 980 | 8963 | 8966 | 8970 | 8972 | 8975 | 8978 | 8980 | 8984 | 8987 | 8990 |
| 990 | 8992 | 8995 | 8998 | 9001 | 9004 | 9007 | 9009 | 9012 | 9016 | 9019 |

On the flatbed path `CurveData1` is selected only for 200 dpi scans (2584 pps,
section 5.4) and by the factory `MyScanner_MotorMoveTest` (5200 pps). The fast
feed (`FastTBPPS = 1500`), homing (1900), calibration moves (800) and every
other scan speed use `CurveData2`.

---

## 5. Motor set-up for a scan — `ScanCtrl_SetScanParameter`

`Scanner_SetScanParameter(isMotorStillMode, isAutoGoHome, isHWShading, p)` fills
a `ScanParameter` and calls `Api_SetScanParameter` = `ScanCtrl_SetScanParameter`.
The real scan (`MyScanner_StartScan`) calls it with **`(0, 0, 1, …)`**: motor
moving, **auto-go-home off**, hardware shading on. Calibration passes
`(1, 0, 0)` (still) for dark/offset/exposure and `(0, 0, 0)` for white shading
(moving, fixed 464.9 pps — `spMotorFixedSpeed = 1`).

### 5.1 Inputs at 300 dpi colour flatbed

From `MyScanner_SetParameter2` / `MyScanner_StartScan` / `Scanner_SetScanParameter`
for `dwScanMode = 0` (colour), `dwXRes = dwYRes = 300`:

| Host variable | Value | Source |
|---|---|---|
| `CCD_DPIHW` → `CtlSensor_iOptRes` | 1200 (`0x4b0`; 2400 when resolution ≥ 301) | `MyScanner_StartScan` |
| `ScanTBunit` | **4** at 300 dpi (`dwResolution < 0x12d`); 4 at 200, 8 at 600, 2 at 1200, 8 at 2400 — see 5.4 | `MyScanner_StartScan` |
| `FastTBunit` | 2 | `.data` |
| `FastTBPPS` | 1500.0 | `.data` |
| `FastTBVref`, `ScanTBVref` | 1, 1 | `.data` |
| `MotorGear` | 600 | `.data` (`ADFMotorGear` = 300, ADF only) |
| `spMotorResolution` | 300 (= `dwYResolution`) | `Scanner_SetScanParameter` |
| `spExposureTime` (`fTGTime`) | 0.387 (ms) for colour at 300/200 dpi; grey 1.161. (600: 0.717 / 2.151; 1200: 1.683 / 5.049; 2400: 3.367 / 10.101) | `MyScanner_SetParameter2` |
| `spDummyLines` | 0 → `CtlSensor_nStepPerLine = 1` | |
| `spScanBeginLine` | `trunc(dwYPos / dwYRes * 2400) + 0x32a` (810) at 300 dpi. Other offsets: <300 dpi `2400·y + 0x35c`; 600 dpi `4800·y + 0x582`; 1200 `4800·y + 0x51e`; 2400 `4800·y + 0x4a6` | `Scanner_SetScanParameter` |
| `CCDLMT` | 2 (colour), 0 (grey) → reg `0x0c[0:4]` | `ScanCtrl_SetScanParameter` |
| `spDisBackTrack`, `spMotorFixedSpeed`, `spRepeatParams`, `spADFscan` | 0 | |

### 5.2 Motor-related call order (flatbed, `spStillMode = 0`)

Non-motor steps are listed only where they feed a motor computation.

| Step | Function | Wire operation | Address / value (300 dpi colour) | Notes |
|---|---|---|---|---|
| 1 | — | bit write | `0x0c[0:4] = 0` | CCDLMT temporarily 0 |
| 2 | — | 3-reg write | `0x7d..0x7f = 1` | TG period temporarily 1 |
| 3 | `CtlSensor_SetDummyLine(0)` | bit write | `0x1d[0:5] = 0` | `nStepPerLine = 1` |
| 4 | `CtlMotor_SetStartStop(spDisBackTrack != 1)` = `(1)` | bit write | `0x02[6] = 0` | back-tracking enabled |
| 5 | … sensor/AFE set-up … | | | |
| 6 | TG period | 3-reg write | `0x7d..0x7f = wTGPeriod = trunc(0.387 × 1000 / 0.25) = 1548` | `CtlSensor_wTGPeriod = 1548` |
| 7 | `CtlMotor_SetMotorVref(FastTBVref, FastTBVref, ScanTBVref, ScanTBVref)` = `(1,1,1,1)` | reg write | `0x3b = 0x55` | overwritten in step 10 |
| 8 | `CtlMotor_SetScanTableUnit(ScanTBunit = 4)` | bit writes | `0x38[5]=1`, `0x37[5]=0`, `0xa0[0:3] = 2` | `m_iScanTableUnit = 4` |
| 9 | `CtlMotor_SetFastTableUnit(FastTBunit = 2)` | bit writes | `0x38[5]=1`, `0x37[5]=0`, `0xa0[3:3] = 2` | fall-through, see 4.2; `m_iFastTableUnit = 2` |
| 10 | speed derivation | host only | `Final_TG = trunc(nStepPerLine × wTGPeriod / (MotorGear / spMotorResolution)) × (CCDLMT+1)` = `trunc(1 × 1548 / 2) × 3 = 2322` | pixel clocks per motor step |
| 11 | `CtlSensor_CalculateTBTIME(Final_TG)` | bit write | `tb = (Final_TG >> 16) + 1` rounded up to {1,2,4,8,16,32} → 1 → `0x1c[0:3] = 0` | `nTBtime = 1`; tb grows only when a step exceeds 65535 clocks |
| 12 | `Final_PPS` | host only | `spMotorFixedSpeed ? 464.9 : (1e6 / PixelTime) / Final_TG = 4 000 000 / 2322 = 1722.65 pps` | **scan speed is derived from the TG period**: one motor step = line time / steps-per-line |
| 13 | `CtlMotor_LoadPPSCurveToScanTable(CurveData2, 510, 1722.65, 4, 1, &n)` | see 4.3 | `0x38[5]=1, 0x37[5]=0, 0xa0[0:3]=2`; `0x3b = 0`, `0x3c = 0`; table of **268 words** (265 curve entries < 1722.65, + target, padded to 268): `word = 4e6/(pps·1·4) = 1e6/pps` → `[0xffff, 43478, 24390, 16666, 12820, 10416, …, 581, 580, 580, 580, 580]`; bulk 536 bytes to `0x10000000`, `0xa4:0xa5 = 0x010c`; same to `0x10004000`, `0xaa:0xab = 0x010c`; same to `0x10008000`, `0xac:0xad = 0x010c` | `m_iTableSum = 452918`, `m_iTableHigh = 580` |
| 14 | `CtlMotor_LoadPPSCurveToFastTable(CurveData2, 510, 1500.0, 2, 1, &n)` | see 4.3 | `0x38[5]=1, 0x37[5]=0, 0xa0[3:3]=2`; `0x3b = 0x50` then `0x3b = 0`, `0x3c = 0`; **184 words** (180 entries < 1500, + target, padded): `word = 2e6/pps` → `[0xffff, 0xffff, 48780, 33333, 25641, …, 1336, 1333, 1333, 1333, 1333]`; bulk 368 bytes to `0x1000c000`, `0xae:0xaf = 0x00b8`; same to `0x10010000`, `0xb0:0xb1 = 0x00b8` | |
| 15 | `CtlMotor_CalculateFeedReg(trunc(spScanBeginLine / (iOptRes / MotorGear)))` | 2-reg write, 3-reg write, bit write | arg = `(2400·y + 810) / 2` = 405 for `y = 0`. `0xa2:0xa3 = 0x0004` (= `m_iScanTableUnit`). Then (see 5.5) `tmpline = 405 − 2·(184/2) − 268/4 − 4/4 = 153` ≥ 1 → fast: `0x3d..0x3f = 2 × 153 = 306 = 0x000132`, `0x02[3] = 1` | feed before the scan window |
| 16 | `CtlMotor_SetMotorPowerBit(1)` | bit write | `0x02[4] = 1` | |
| 17 | `CtlMotor_SetZ1Z2Value()` | two 3-reg writes | `0xb2..0xb4 = Z1`, `0xb5..0xb7 = Z2` (5.6): with the values above `Z1 = 1750 = 0x0006d6`, `Z2 = 4190 = 0x00105e` | |
| 18 | … SDRAM / MAXWD … | | | |
| 19 | `CtlMotor_EnableAutoGoHome(spAutoGoHome)` = `(0)` | bit write | `0x02[5] = 0` | **auto-go-home disabled** for the real scan |
| 20 | … gamma / shading enables … | | | |
| 21 | `CtlMotor_SetMotorDirect(1)` | bit write | `0x02[2] = 0` | away from home |
| 22 | — | bit writes | `0x19[0]=1`, `0x0c[0:4] = CCDLMT = 2`, `0x01[7]=1`, `0x08[4]=1`; then after the ADF block: `0x01[7]=1`, `0x1e[0]=1`, `0x19[0]=1`, `0x1e[2]=1`, `0x08[4]=0`, `0x1b[7]=1`, `0x19[2]=1` | image-path bits, listed for ordering only |
| 23 | `CtlImage_EnablePattern(0)`, `CtlImage_ClrCounter(1,1)`, `sleep1((CCDLMT+1) × spExposureTime)` | | `sleep1(trunc(3 × 0.387)) = sleep1(1)` ms | |

After `Api_SetScanParameter` returns, `Scanner_SetScanParameter` (SANE side)
writes a block of sensor/AFE registers and then, for **every** resolution
(the four branches of its resolution ladder are identical):

| Step | Function | Wire operation | Address / value |
|---|---|---|---|
| 24 | `CtlMotor_SetMotorVref(0, 0, 2, 2)` | reg write | `0x3b = 0x0a` (gohome 0, ffast 0, ssback 2, fscan 2) |
| 25 | `CtlMotor_SetMotorVref_2(0, 0, 2, 2)` | reg write | `0x3c = 0x0a` |

So the Vref values that are actually in force when the scan starts are
`0x3b = 0x3c = 0x0a`; everything `LoadPPSCurve*` did to them is dead.
`Scanner_Calibration_Shading_SetScanParameter` does the same `(0,0,2,2)` pair.

### 5.3 Still mode (`spStillMode = 1`, calibration only)

Replaces steps 7–17 with: `wSlope[0..3] = wTGPeriod / nTBtime` (4 identical
words), `CtlMotor_WriteScanSlopeTable(wSlope, 4)` (three bulk writes of 8 bytes,
`0xa4 = 0xaa = 0xac = 4`); `CtlMotor_SetFeedSteps(1)`; `CtlMotor_EnableFastMove(0)`;
`CtlMotor_SetMotorPowerBit(0)`; `CtlMotor_WriteMotorRemainder(0, 0)`. The motor is
unpowered and the ASIC's line clock is driven by a 4-entry constant table.

### 5.4 Scan speed by resolution (colour, `nStepPerLine = 1`)

| Resolution | `fTGTime` ms | `wTGPeriod` | `MotorGear/res` | `Final_TG` | `Final_PPS` | Curve | `ScanTBunit` |
|---|---|---|---|---|---|---|---|
| 200 | 0.387 | 1548 | 3 | 1548 | 2583.98 | **CurveData1** | 4 |
| 300 | 0.387 | 1548 | 2 | 2322 | 1722.65 | CurveData2 | 4 |
| 600 | 0.717 | 2868 | 1 | 8604 | 464.90 | CurveData2 | 8 |
| 1200 | 1.683 | 6732 | 0.5 | 40392 | 99.03 | CurveData2 | 2 |
| 2400 | 3.367 | 13468 | 0.25 | 161616 | 24.75 | CurveData2 | 8 |

`ScanTBunit` comes from the `if` ladder in `MyScanner_StartScan` (`res`, `xres`,
`yres` all equal for a normal scan): `res < 301` → 4; `res < 601` → 8; `res < 1201`
→ 2 (the `2` branch is reached because `yres < 0x4b1` holds only via the outer
`|| (dwYResolution < 0x4b1)` term — a 600×1200 request also lands there); else 8.
Note that calibration runs *before* this assignment with `ScanTBunit = 8`
(set at the top of `MyScanner_StartScan`).

(`Final_TG` for 1200/2400 exceeds 16 bits only at 2400 → `tb = 4` there.) 200 dpi
colour is the one flatbed case that selects `CurveData1`. Grey mode uses the
three-times-longer `fTGTime` with `CCDLMT + 1 = 1`, giving the same `Final_TG`.
`spMotorFixedSpeed = 1` (white-shading calibration) forces 464.9 pps regardless.

### 5.5 Feed computation — `CtlMotor_CalculateFeedReg(ScanBeginLine)`

```
write16(0xa2, m_iScanTableUnit & 0xff)
scanLen = read16(0xa4) / m_iScanTableUnit          // ramp length in whole steps
fastLen = read16(0xae) / m_iFastTableUnit
a2      = read16(0xa2) / m_iScanTableUnit          // = 1 with the values above
tmpline = ScanBeginLine - 2*fastLen - scanLen - a2 // (reads come from the host shadow, not the device)
if tmpline >= 1:
    feed = m_iFastTableUnit * tmpline ; fast = 1    // fast move up, feed, ramp down, then scan ramp
else:
    tmpline = max(ScanBeginLine - scanLen, 0)
    feed = m_iScanTableUnit * tmpline ; fast = 0    // too close: approach with the scan table only
CtlMotor_SetFeedSteps(feed)          // 0x3d..0x3f
CtlMotor_EnableFastMove(fast)        // 0x02[3]
```

The ASIC therefore executes, on `0x0f = 0xff`: fast ramp up (184 entries), `feed`
unit-steps at 1500 pps, fast ramp down, scan ramp up (268 entries), then scan at
1722.65 pps while `0x01[0]` captures `CtlImage_SetScanLength` lines, then (if
`0x02[5]`) return home. This phase structure is inferred from the arithmetic;
the ASIC's actual sequencing is not visible in the driver.

### 5.6 Z1 / Z2 — `CtlMotor_SetZ1Z2Value`

```
L        = (read8(0x0c) & 0x0f) + 1                        // CCDLMT + 1 (3 colour, 1 grey)
b_value  = (read8(0x02) & 0x08) ? read16(0xa2)             // fast move on: the 0xa2 value (= unit)
                                : read20(0x3d)             // fast move off: the feed count
Z1 = (m_iTableSum + (read16(0xa6) - 1) * m_iTableHigh) % (L * wTGPeriod)   // 0xa6 = 240 from init
Z2 = (m_iTableSum + (b_value   - 1) * m_iTableHigh) % (L * wTGPeriod)
write24(0xb2, Z1); write24(0xb5, Z2)
```

(`m_iTableSum` = tb × sum of scan-table words, `m_iTableHigh` = tb × last word, from
`CalculateTableSum`.) These are the phase remainders of the motor step clock
relative to the colour line period after the ramp — the ASIC presumably uses
them to keep the R/G/B exposure boundaries aligned to motor steps. All reads are
from the host shadow table.

---

## 6. Start, stop and the motor afterwards

### 6.1 `ScanCtrl_StartScan` (= `Api_StartScan`), flatbed path

| Step | Function | Wire operation | Address / value |
|---|---|---|---|
| 1 | `CtlImage_EnableScan(1)` | bit write | `0x01[0] = 1` |
| 2 | `CtlMotor_SetMotorStartMove()` | reg write | `0x0f = 0xff` |
| 3 | `ScanCtrl_ChkScanDataReady()` | reads | poll `0x101` (bit 4 / bit 6), then `0x102` != 0 (section 1.3) |

(For `m_ccdpackingflg != 0` a first `CMDASIC_ReadRawImage` of the line-difference
buffer follows; that flag is always 0 in this driver.) The scan is a single
hardware-sequenced move; the host only reads bulk data afterwards.

### 6.2 `ScanCtrl_StopScan` (= `Api_StopScan`), flatbed

`CtlImage_EnableScan(0)` → `0x01[0] = 0`. Nothing is written to the motor; for the
ADF case only it first waits for motor stop. The motor is **not** stopped by the
host — a running feed/scan continues until the ASIC finishes it.

### 6.3 `MyScanner_StopScan` (SANE-side end of scan)

```
Scanner_ThreadEnd()                 // reader thread + cache
Api_StopScan()                      // 0x01[0] = 0
Api_WaitMotorStop()                 // poll 0x101 bit 0 until the ASIC has finished the programmed travel
(colour-matrix release)
Scanner_CarriageHomeDontWait()      // if !home && !moving: Api_MoveMotor2(1900.0, 12000, 0, 4)  — returns immediately
Api_CloseDevice()
Api_OpenDevice2(0x2e5a, 0xd011)     // re-open just to write the LED GPIO:
0x3a[2] = 0 ; 0x35[1] = 1 ; 0x32[1] = 0
Api_CloseDevice()
```

Because the scan was programmed with auto-go-home **off** (5.2 step 19), the
carriage is returned by this explicit non-blocking home move, and the device is
closed while it is still travelling. The next `MyScanner_CarriageHome` (start of
the next scan) finds bit 3 set and does nothing, or finds bit 0 set and waits.
`Scanner_Calibration_StopScan` = `Api_StopScan` + `Api_WaitMotorStop` (no homing;
calibration continues from where the carriage stopped).

---

## 7. Lamp

The VF3240 is a CIS scanner; the "lamp" is the CIS LED bar. Only one lamp bit
(`0x03[4]`) is used on the flatbed path.

### 7.1 Registers

| Reg / field | Function | Meaning |
|---|---|---|
| `0x03[4]` | `CtlLamp_SetLampOnOff(b)` | Lamp on (1) / off (0) |
| `0x03[5]` | `CtlLamp_SetXPLampOnOff(b)` | XP (second) lamp select; `Api_GetLampStatus` reads it back as `xpasel` |
| `0x03[7]`, `0x03[0:4]`, `0x05[4:2]` | `CtlLamp_SetLampWatchDog(min)` | Lamp timeout, see 7.4 |
| `0x0a[4]` | `CtlLamp_EnableLampPWM(b)` | PWM enable |
| `0xfa`, `0xfb` (+`0xfc`) | `CtlLamp_SetLampPWMDuration(on, off)` | PWM on/off counts (see the overlap note in section 2) |
| `0x101[2]` | `CtlLamp_GetLampState` | Lamp currently on |

### 7.2 `Api_SwitchLampPower(iDevice, iLampState)`

```
if iLampState == 1:
    iDevice == 0: 0x03[5] = 0      // flatbed lamp
    iDevice == 1: 0x03[5] = 1      // XP lamp
    0x03[4] = 0                    // off ...
    0x03[4] = 1                    // ... then on (pulse)
else:
    0x03[4] = 0
```

### 7.3 `Api_GetLampStatus(iDevice, *state)`

Reads reg `0x03`; `xpasel = bit 5`. For `iDevice == 0`: state = `0x101[2]` if
`xpasel == 0`, else 0. For `iDevice == 1`: state = `0x101[2]` if `xpasel == 1`, else
0. I.e. the status bit is attributed to whichever lamp is currently selected.

### 7.4 `Api_SetLampTimeOut(min)` = `CtlLamp_SetLampWatchDog(min)`

```
nVal = reg3_shadow with bit 4 := current 0x101[2]          // preserve the actual lamp state
if min == 0:  write8(0x03, nVal & 0x7f)                     // watchdog off
else:
    min <  16: val = min      ; unit = 0
    min <  32: val = min / 2  ; unit = 1
    else     : val = min(min,60) / 4 ; unit = 2
    write8(0x03, (nVal & 0x70) | val)                        // clears bit 7 and sets the 4-bit value
    0x05[4:2] = unit
    write8(0x03, shadow | 0x80)                              // watchdog on
```

`Api_SetLampPWM(pwm)`: `-1` → `0x0a[4] = 0`; else `0x0a[4] = 1`, `on = min(pwm, 400)`,
`SetLampPWMDuration(on, 400 - on)`. **Neither `Api_SetLampPWM` nor
`Api_SetLampTimeOut` has a caller** in this binary (the watchdog stays at its init
value `0x03 = 0x9f`: enabled, value 15, unit 0 → 15 min, inferred).

### 7.5 What the flatbed path does — `Scanner_SwitchLampPower(on)`

```
state = Api_GetLampStatus(0)
if state == 0 && on == 1: Api_SwitchLampPower(0, 1)
if state == 1 && on == 0: Api_SwitchLampPower(0, 0)
else: nothing
```

Call sites: `Scanner_CarriageHome` → off (before the home move);
`Scanner_Calibration2` → on, `sleep1(100)`, off (then moves 0xe1/0xfa steps);
`Scanner_Calibration_WhiteShading` → on before the shading scan;
`MyScanner_StartScan` → on, immediately before `Scanner_SetScanParameter`;
`MyScanner_MotorTest` → off. **No call turns the lamp off after a scan**; the
watchdog (init `0x9f`) is relied on. Since init itself sets `0x03 = 0x9f` (lamp
on), the lamp is on from `Api_InitiateASIC*` onward until `Scanner_CarriageHome`
turns it off for the home move.

---

## 8. Safety notes for a reimplementation

1. **No software end-stop.** The driver never bounds a move by bed length. Both
   homing (`12000 × 4` unit-steps) and the factory test's return move (`24000 × 2`)
   are far longer than the bed. At 600 steps/in (`MotorGear`, section 9 Q1) an
   A3 bed is ≈ 10 100 steps; the home move is 48 000 unit-steps at unit 4 =
   12 000 steps ≈ 20 in. **The only thing that can stop a toward-home move is the
   ASIC acting on the home sensor (`0x101` bit 3)** — inferred, never observed.
   Verify on hardware with a *short* toward-home move first (e.g. 200 steps,
   dir 0), confirm bit 0 clears and bit 3 behaves, before issuing the vendor's
   12000-step move.
2. **Away-from-home moves have no protection at all.** The vendor's biggest
   away move is `0x4d58` = 19 800 steps at unit 2 (`MotorMoveTest`, dir 1) — that
   is ≈ 16.5 in if `MotorGear` is steps/in, within A3 (16.5 in) only barely.
   Never issue dir = 1 moves without having verified the step-to-distance
   ratio (section 9 Q1) and the current position.
3. **Scan length is the end-stop for scans.** A scan moves `feed` + ramps +
   `CtlImage_SetScanLength(lines × (CCDLMT+1))` lines and then stops (or returns
   home if `0x02[5]`). Keep `spScanBeginLine + spTotalScanLines` within the
   vendor's own maxima.
4. **Always home first.** `StartScan1/2` and `Initialize` call
   `MyScanner_CarriageHome` before anything else, and it performs a full ASIC
   re-init (`Api_InitiateASIC`) when the carriage is away from home and idle.
5. **Vendor timeouts**: 60 s "motor stop" and 60 s "data ready" (CPU-time based,
   section 1.3 — treat as "effectively infinite" in the vendor driver, use a
   real 60 s wall clock yourself); 30 s libusb transfer timeout. The vendor never
   sleeps between status polls.
6. **Shadow-register hazard.** Every bit write and every value read by
   `CalculateFeedReg` / `SetZ1Z2Value` comes from the host shadow
   (`CMDASIC_RegTable`), which is **only valid after the init batch has been
   written in the same process**. A reimplementation that opens a device with a
   stale ASIC state must re-upload `ASIC_INIT_REG` (as `MyScanner_CarriageHome`
   does) before relying on read-modify-write bit operations.
7. **Vref.** The effective motor-current selectors during a scan are
   `0x3b = 0x3c = 0x0a`; during a standalone move both are `0x00` (section 4.3).
   Whether 0 means "lowest current" or "default" is unknown; do not run long
   moves with values that differ from the vendor's until the thermal behaviour
   is confirmed.
8. **Lamp.** The CIS LED is on from init until the home move turns it off, and is
   left on after a scan. There is no evidence that this is harmful (CIS LEDs),
   but turn it off explicitly in a reimplementation.

---

## 9. Open questions

1. **Step-to-distance ratio.** `MotorGear = 600` combined with
   `Final_TG = TG × nStepPerLine / (MotorGear / spMotorResolution)` implies
   600 motor steps per inch during a scan (2 steps per 1/300 in line). But
   `spScanBeginLine` is computed as `yPos_in × 2400 + const` at 300 dpi and divided
   by `iOptRes / MotorGear = 2`, giving 1200 steps per inch of requested
   Y-offset — a factor-2 mismatch, unless the fast-table unit-code fall-through
   (4.2: unit 2 programmed as code 2) makes the hardware run the fast feed at
   half the host's step count. Needs a ruler: measure carriage travel for
   `Api_MoveMotor2(800, N, 1, unit)` with unit 2 and 4, and the offset produced
   by `dwYPos`.
2. **Does the ASIC stop a toward-home move on the home sensor?** (Sections 3.2,
   8.1.) The driver's correctness depends on it; nothing in the driver proves it.
3. **`0x37[4]`, `0x38[4]`, `0x38[5]`** — written to 1 before standalone moves /
   whenever a unit is set; never cleared; meaning unknown. **Hardware
   (2026-10-13):** with `0x37[4]`/`0x38[4]` left set (by a `MoveMotor2`-style
   move not followed by a re-init), a subsequent scan captures lines but the
   carriage does not move. So they select a standalone-move mode that
   overrides scan-driven motion; only the init table clears them. Use
   `ScanCtrl_MoveMotor` for moves between init and a scan, as the vendor does.
4. **`0xa2:0xa3`** — written with the scan unit, read as "b_value" for Z2 and as a
   subtractive term in `CalculateFeedReg`; register name unknown.
5. **`0xa6:0xa7 = 240`** (from init, never written) — used in Z1; meaning unknown.
6. **Three scan slots / two fast slots** at `0x10000000 + n×0x4000` always receive
   identical tables. Which slot the ASIC uses for accelerate / cruise /
   decelerate (or per colour phase) is unknown; a reimplementation must write all
   of them.
7. **`0x101` bit 4 / bit 6** semantic names ("scan finished", "busy") are inferred
   from `ScanCtrl_ChkScanDataReady`'s control flow only.
8. **`0x102`** — which counter it is (bytes? lines?) is unknown; only `!= 0` is used.
9. **Vref encoding** (`0x3b`/`0x3c`, four 2-bit fields) — physical current levels
   unknown; vendor uses 0 during moves and 2 (`ssback`, `fscan`) during scans.
10. **`CtlLamp_SetLampPWMDuration` register overlap** (`0xfa:0xfb` then `0xfb:0xfc`)
    — unused on the flatbed path; unresolved whether the registers are 8-bit.
11. **Lamp watchdog unit** (reg `0x05[4:2]`: 1 / 2 / 4 min per count) is inferred
    from the divisor ladder in `CtlLamp_SetLampWatchDog`; the init value 15 would
    be 15 min.
12. **`CtlMotor_WriteMotorPhaseTable` / `EnableLongCurve`** — present, uncalled;
    the ASIC presumably has a built-in phase table that this driver never
    replaces.
13. `gettickcount` uses `clock()` (CPU time). Whether the vendor's effective
    unbounded waits ever matter in practice (e.g. a stalled carriage) is
    unknown; a real 60 s wall-clock timeout plus an explicit `0x01[0] = 0` /
    `0x02[4] = 0` (motor power off) is the proposed recovery, but **the vendor
    never performs a motor abort**, so the abort behaviour is untested.
