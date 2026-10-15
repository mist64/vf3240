# VF3240 ASIC initialisation

Status: **verified on hardware (2026-10-13):** the full 300 dpi init sequence is accepted and spot-checked registers read back as documented; register meanings remain inferred.

Source of truth: `libviisanvf3240dr.so` decompiled corpus at
`re/exports/libviisanvf3240dr.so/decomp/`, specifically
`Api_OpenDevice2.c`, `CtlDevice_OpenDevice2.c`, `CMDIO_OpenDevice2.c`,
`CMDIO_GetDeviceFeatures.c`, `CMDIO_CloseDevice.c`, `Api_InitiateASIC3.c`,
`CtlDevice_LoadInitRegData3.c`, `CtlDevice_LoadInitRegData.c`,
`CMDASIC_WriteMultiRegister.c`, `CMDASIC_WriteRegister.c`,
`CMDASIC_WriteRegisterBit.c`, `CMDASIC_WriteTwiRegister.c`,
`CMDASIC_WriteUsbRegister.c`, `CMDASIC_WriteBulkData.c`,
`CMDASIC_SetupBulkLength.c`, `CMDIO_BulkWrite.c`, `CtlAfe_WriteAFERegister.c`,
`CtlAfe_ReadAFERegister.c`, `CtlDevice_SetSDRAMType.c`,
`CtlDevice_SetScannerClock.c`, `CtlDevice_SetAMBAClock.c`,
`CtlDevice_SetScanMOD.c`, `CtlADF_ADFInit.c`, `CtlImage_SDRamSet_ShadingBank.c`,
`CtlImage_SetDefaultGamma.c`, `MyScanner_StartScan.c`,
`MyScanner_CarriageHome.c`, `Scanner_CarriageHome.c`, `Initialize.c`,
`MyScanner_OpenDeviceForUnknowStatus.c`, `MyScanner_CheckCarriageAtHome.c`,
`Api_CheckMotorinHome.c`, `Api_GetMotorStatus.c`; types from
`re/exports/libviisanvf3240dr.so/types.h` (`RegRec`, `tagGammaData`,
`SetupCmd`); and the **data tables and initialised globals read directly from
the ELF** `vendor/linux-sane-1.0.2.0/unpacked/usr/local/lib64/sane/aviisandr-adv/VF3240/libviisanvf3240dr.so`
(symbol addresses from `objdump -t`, bytes extracted with a throw-away
python script that maps VMA to file offset via the section headers).

This document consumes the transport in [`01-usb-transport.md`](01-usb-transport.md),
the opcodes in [`02-opcode-map.md`](02-opcode-map.md) and the open sequence in
[`03-device-open.md`](03-device-open.md). Wire notation used below:

- `CTRL OUT 0x83 idx=0 len=N [..]` = `libusb_control_transfer(0x40, 0x0C, wValue=0x0083, wIndex=0, payload, N)`
- `CTRL OUT 0x8C idx=A len=1 [v]` = same with `wValue=0x008C, wIndex=A`
- `CTRL IN 0x18E idx=0x0122 len=2` = `libusb_control_transfer(0xC0, 0x0C, wValue=0x018E, wIndex=0x0122, buf, 2)`
- `BULK OUT 0x02 len=N` = `libusb_bulk_transfer(hdl, 0x02, buf, N, &n, 30000)`

The libusb timeout for every transfer is the global `libusb_timeout`, which
the ELF `.data` section initialises to **30000 ms** (symbol at `0x22b2a8`).

## 0. Where init sits in the scan path

The SANE shim (`libsane-aviisandrvf3240.so.1`) does not link the core; it
`dlopen`s it and `dlsym`s a dozen entry points (`strings` on the shim:
`Initialize`, `StartScan1`, `StopScan`, `ReadImage`, `GetKeyStatus`,
`GetCoverStatus`, `SetPowerSaving`, `CancelScaner`, `GetParameters`,
`GetDeviceInfo`, `LoadDriver`, `FreeDriver`). The scan entry is
`StartScan1(dwScanMode, dwDPI, ...)` (`StartScan1.c:29-67`), which only
accepts `dwDPI` in {200, 300, 600, 1200, 2400}, forces `dwGamma = 16`, and runs:

```
MyScanner_Open()                       // CCD_DPIHW = 1200; ScanTBunit = 2; no USB traffic
MyScanner_CarriageHome()               // open, home the carriage if needed (section 5.2), close (USB reset)
MyScanner_SetParameter2(mode, dwDPI, dwDPI, x, y, w, h, 16)   // X and Y resolution are always equal
MyScanner_GetParameter2(...)
MyScanner_StartScan(NULL)
```

So before every scan the device may already have been through one
`Api_InitiateASIC()` + motor move + USB reset cycle inside
`MyScanner_CarriageHome`; `MyScanner_StartScan` then re-opens and
re-initialises from scratch.

`MyScanner_StartScan(pCallback)` (`MyScanner_StartScan.c`): its first three
actions are:

```
CCD_DPIHW = (g_Scan_Parameters.dwResolution < 301) ? 1200 : 2400;   // line 11-16
ScanTBunit = 8;
Api_OpenDevice2(0x2e5a, 0xd011)                                     // section 1
Api_InitiateASIC3(g_Scan_Parameters.dwXResolution,
                  g_Scan_Parameters.dwYResolution)                  // section 2
CMDASIC_WriteRegisterBit(0x3a, 2, 1, 1)                             // section 5
... Scanner_CheckCalibration / Scanner_Calibration2 / lamp / gamma / Api_StartScan (later documents)
```

`g_Scan_Parameters.dwResolution`, `dwXResolution` and `dwYResolution` are
all set from the same `dwXRes`/`dwYRes` arguments in
`MyScanner_SetParameter2.c:14-16` (`dwResolution = dwXRes`). For the
**300 dpi, colour, flatbed** reference case: `dwResolution = dwXResolution =
dwYResolution = 300`, hence `CCD_DPIHW = 1200`.

`Initialize()` (`Initialize.c`), the probe path the shim runs before any scan,
does *not* run `Api_InitiateASIC3` itself; it opens/closes the device up to
three times and only reaches ASIC init indirectly through
`MyScanner_CarriageHome` when the carriage is not at home (section 5.2).

## 1. Open: `Api_OpenDevice2` → `CtlDevice_OpenDevice2` → `CMDIO_OpenDevice2`

`Api_OpenDevice2(wVID, wPID)` and `CtlDevice_OpenDevice2(wVID, wPID)` are
one-line pass-throughs (`Api_OpenDevice2.c:9`, `CtlDevice_OpenDevice2.c:9`).
Every caller in the corpus passes the literals `0x2e5a, 0xd011`
(`MyScanner_StartScan.c:20`, `MyScanner_CarriageHome.c:9`,
`MyScanner_CheckCarriageAtHome.c`, `MyScanner_GetKeyStatus.c`,
`MyScanner_SetScanLED.c`, `MyScanner_StopScan.c`,
`MyScanner_SetPowerSaverStatus.c`, `MyScanner_MotorTest.c`, ...).
`MyScanner_OpenDeviceForUnknowStatus` calls `CMDIO_OpenDevice2(0x2e5a,0xd011)`
directly, bypassing the Api/Ctl layers.

### 1.1 Sequence (`CMDIO_OpenDevice2.c`)

| Step | Call | Notes |
|---|---|---|
| 1 | `libusb_init(&sanei_usb_ctx)` | failure → return 0 |
| 2 | `libusb_get_device_list(sanei_usb_ctx, &devlist)` | failure → **falls through with the result = return value of `printf`** (vendor bug: the function returns the printf byte count, a positive number, i.e. "success"). Never freed (`libusb_free_device_list` is absent from both open variants). |
| 3 | per device: `libusb_get_bus_number`, `libusb_get_device_address`, `libusb_get_device_descriptor` | bus/address only used in log strings |
| 4 | match `desc.idVendor == wVID && desc.idProduct == wPID` | |
| 5 | `libusb_open(dev, &hdl)` | failure → return 0 |
| 6 | `libusb_get_configuration(hdl, &config)` | result unused |
| 7 | `libusb_get_device_descriptor(dev, &desc)` | re-fetched, unused |
| 8 | `libusb_get_config_descriptor(dev, 0, &config0)` | |
| 9 | `libusb_set_configuration(hdl, config0->bConfigurationValue)` | `-6` (BUSY) tolerated; `-3` and any other error → `libusb_close`, `libusb_free_config_descriptor`, return 0 |
| 10 | `libusb_claim_interface(hdl, config0->interface->altsetting->bInterfaceNumber)` | any error (incl. `-6`) → `libusb_close`, return 0 |
| 11 | `bIsFindMysherScanner = 1; break` | |
| 12 | after loop: `if (bIsFindMysherScanner == 0) return 0` | **explicit not-found check** |
| 13 | `CMDIO_GetDeviceFeatures()` | on failure: `CMDIO_CloseDevice()` is called **but the function still returns 1** (`CMDIO_OpenDevice2.c:122-127`). Same defect exists in `CMDIO_OpenDevice.c:124-128`. |
| 14 | return 1 | |

No control or bulk traffic other than the single `0x8E/idx=0` probe inside
`CMDIO_GetDeviceFeatures` is issued by the open path.

### 1.2 `CMDIO_GetDeviceFeatures` (`CMDIO_GetDeviceFeatures.c`)

```
CMDIO_GetConnectionType(&CMDIO_iConnectType)      // CTRL IN 0x8E idx=0 len=1; CMDIO_iConnectType = (byte & 0x08) == 0
if fail → return 0
CMDIO_BulkFiFoSize = (CMDIO_iConnectType == 1) ? 0x200 : 0x40
CMDIO_VID = 0x2e5a; CMDIO_PID = 0xd011              // hard-coded, not taken from the descriptor
return 1
```

This is exactly as described in `03-device-open.md`; nothing differs between
the two open variants here.

### 1.3 Close: `Api_CloseDevice` → `CtlDevice_CloseDevice` → `CMDIO_CloseDevice`

Relevant because every `MyScanner_*` entry point is a full open/close cycle
(`CMDIO_CloseDevice.c:8-13`):

```
libusb_clear_halt(hdl, 0x02)
libusb_clear_halt(hdl, 0x81)
libusb_reset_device(hdl)          // <-- the device is USB-reset on every close
libusb_release_interface(hdl, 0)  // literal 0, not the claimed bInterfaceNumber
libusb_close(hdl)
libusb_exit(sanei_usb_ctx)
```

Consequence for section 6: the ASIC is reset (or at least re-enumerated)
between `Initialize()` and `MyScanner_StartScan()`, while the host-side
register shadow is a process global that survives. The vendor driver copes
only because `Api_InitiateASIC3` rewrites 117 registers in bulk before
anything relies on the shadow.

### 1.4 Amendment to 03-device-open.md

`03-device-open.md` states that `CMDIO_OpenDevice` (via `Api_OpenDevice`) is
"the" open path and that `CMDIO_OpenDevice2` is reached only from
"unrelated entry points". That is inverted for the scan path: **`Api_OpenDevice`
has no callers in the corpus** (grep for `Api_OpenDevice(` matches only its own
file and `Api_OpenDevice2`), whereas `MyScanner_StartScan`,
`MyScanner_CarriageHome`, `MyScanner_StopScan`, `MyScanner_CheckCarriageAtHome`
and every other `MyScanner_*` front-door function use `Api_OpenDevice2`.
The sequence a Rust port must mirror is therefore `CMDIO_OpenDevice2`.

Differences between `CMDIO_OpenDevice2` and the documented `CMDIO_OpenDevice`:

| Aspect | `CMDIO_OpenDevice` (03) | `CMDIO_OpenDevice2` (this doc) |
|---|---|---|
| VID/PID | literals `0x2e5a`/`0xd011` | parameters `wVID`/`wPID` (callers always pass the same literals) |
| not-found handling | `ufind` set but never tested; returns 1 and frees an uninitialised `config0` | `bIsFindMysherScanner` tested; returns 0 when no match |
| `libusb_free_config_descriptor` on success | called once after the loop | **never called on the success path** (only in the `set_configuration` failure branch) — descriptor leak, harmless |
| `CMDIO_GetDeviceFeatures` failure | closes device, returns 1 (bug) | closes device, returns 1 (same bug) |
| `libusb_get_device_list` failure | (same) | returns `printf`'s byte count as the status (bug) |
| error tolerances for `set_configuration`/`claim_interface` | as documented in 03 | identical |

Nothing in 03's step list 1–7 ("Summary for Task 7's `Device::open`") needs
to change; step 6 (explicit not-found failure) is in fact what `OpenDevice2`
already does. Add: **do not report success when the feature probe fails**, and
expect the vendor close to perform `libusb_reset_device`.

## 2. `Api_InitiateASIC3(xres, yres)` → `CtlDevice_LoadInitRegData3`

`Api_InitiateASIC3` is a pass-through returning `LoadInitRegData3(...) != 0`
(`Api_InitiateASIC3.c:10`). `Api_InitiateASIC()` →
`CtlDevice_LoadInitRegData()` is the older variant used by
`Scanner_CarriageHome` and `MyScanner_StartScan_Calibration2File`; it is
**byte-for-byte the same sequence** except that it always uses the base
`ASIC_INIT_REG` table (`CtlDevice_LoadInitRegData.c:19-22`). `Api_InitiateASIC2`
(used by the motor test entry points only) is not on the scan path and is not
documented here.

### 2.1 Table selection (`CtlDevice_LoadInitRegData3.c:19-36`)

| `nXRes` | `nYRes` | Table |
|---|---|---|
| 2400 | 2400 | `ASIC_INIT_REG_GPIO111` |
| 1200 | 2400 | `ASIC_INIT_REG_GPIO111` |
| 1200 | 1200 | `ASIC_INIT_REG_GPIO000` |
| anything else (incl. **300/300**, 600/600, 2400/1200) | | `ASIC_INIT_REG` |

The test is on the exact `(x, y)` pair; there is no `<=` comparison.
Because `StartScan1` always passes `dwDPI` for both axes, the reachable
cases are: 2400 → `GPIO111`, 1200 → `GPIO000`, 200/300/600 → `ASIC_INIT_REG`;
the 1200×2400 row is dead code for the SANE front-end.
**For the 300 dpi reference case the base `ASIC_INIT_REG` table is used.**

### 2.2 Table layout and wire encoding

All three tables are `RegRec[117]` (`0x75` entries, `0x1d4` bytes). A
`RegRec` (DWARF `types.h`) is 4 bytes: a 16-bit register address, an 8-bit
value, and one byte of padding.

`LoadInitRegData3` copies the selected table into a `malloc(0x1d4)` buffer
(`u16 addr` at `+0`, `u8 value` at `+2`, byte `+3` untouched) and hands it to
`CMDASIC_WriteMultiRegister(pBuf, 0x1d4)`, which (`CMDASIC_WriteMultiRegister.c:22-66`):

1. for each of `dwLen >> 2` entries: sets the host shadow
   `CMDASIC_RegTable[addr].nValue = value`, then appends `[addr & 0xFF, value]`
   to buffer `addr1` if `addr & 0x100 == 0`, else to `addr2`;
2. whenever a buffer reaches 64 bytes it is flushed:
   `CTRL OUT 0x83 idx=0 len=64 [addr1]` or `CTRL OUT 0x183 idx=0 len=64 [addr2]`;
3. after the loop any remainder is flushed with its actual length.

Verified against the ELF: all 117 entries in all three tables have `addr <
0x100` and padding byte `0x00`, no address appears twice, and the entries
are *not* sorted (e.g. `0x33` precedes `0x32`). The wire sequence for one
table is therefore exactly **4 control transfers on opcode `0x83`**:

| Transfer | Entries | `wValue` | `wIndex` | `wLength` |
|---|---|---|---|---|
| 1 | 0–31 | `0x0083` | 0 | 64 |
| 2 | 32–63 | `0x0083` | 0 | 64 |
| 3 | 64–95 | `0x0083` | 0 | 64 |
| 4 | 96–116 | `0x0083` | 0 | 42 |

Payload of each transfer is the concatenation of `[addr, value]` pairs in
table order.

### 2.3 `ASIC_INIT_REG` — full contents (ELF `.data` at `0x22b2e0`, 117 rows, wire order)

The three tables differ in **exactly one entry**: index 42, register `0x32`.
Everything else is identical byte-for-byte (`ASIC_INIT_REG_GPIO111` at
`0x22b4c0`, `ASIC_INIT_REG_GPIO000` at `0x22b6a0`).

| # | Transfer | Addr | Value | Notes |
|---|---|---|---|---|
| 0 | 1 | `0x001` | `0x00` |  |
| 1 | 1 | `0x002` | `0x78` |  |
| 2 | 1 | `0x003` | `0x9F` |  |
| 3 | 1 | `0x004` | `0x02` |  |
| 4 | 1 | `0x005` | `0x40` |  |
| 5 | 1 | `0x006` | `0x10` |  |
| 6 | 1 | `0x008` | `0x00` |  |
| 7 | 1 | `0x009` | `0x08` |  |
| 8 | 1 | `0x00A` | `0x40` |  |
| 9 | 1 | `0x00B` | `0x8A` |  |
| 10 | 1 | `0x00C` | `0x02` |  |
| 11 | 1 | `0x011` | `0x00` |  |
| 12 | 1 | `0x012` | `0x00` |  |
| 13 | 1 | `0x013` | `0x1F` |  |
| 14 | 1 | `0x014` | `0x30` |  |
| 15 | 1 | `0x016` | `0x00` |  |
| 16 | 1 | `0x017` | `0x00` |  |
| 17 | 1 | `0x018` | `0x08` |  |
| 18 | 1 | `0x019` | `0x00` |  |
| 19 | 1 | `0x01A` | `0x2E` |  |
| 20 | 1 | `0x01B` | `0x00` |  |
| 21 | 1 | `0x01C` | `0x00` |  |
| 22 | 1 | `0x01D` | `0x20` |  |
| 23 | 1 | `0x01E` | `0x10` |  |
| 24 | 1 | `0x01F` | `0x00` |  |
| 25 | 1 | `0x020` | `0x03` |  |
| 26 | 1 | `0x021` | `0x00` |  |
| 27 | 1 | `0x022` | `0x56` |  |
| 28 | 1 | `0x025` | `0x00` |  |
| 29 | 1 | `0x026` | `0x00` |  |
| 30 | 1 | `0x027` | `0x00` |  |
| 31 | 1 | `0x028` | `0x00` |  |
| 32 | 2 | `0x029` | `0x40` |  |
| 33 | 2 | `0x02A` | `0x00` |  |
| 34 | 2 | `0x02B` | `0xF0` |  |
| 35 | 2 | `0x02C` | `0x04` |  |
| 36 | 2 | `0x02D` | `0xB0` |  |
| 37 | 2 | `0x02E` | `0x80` |  |
| 38 | 2 | `0x02F` | `0x80` |  |
| 39 | 2 | `0x030` | `0xE0` |  |
| 40 | 2 | `0x031` | `0x00` |  |
| 41 | 2 | `0x033` | `0x0F` |  |
| 42 | 2 | `0x032` | `0x69` | GPIO000 table: `0x61`; GPIO111 table: `0x7d` (only differing entry) |
| 43 | 2 | `0x034` | `0x00` |  |
| 44 | 2 | `0x035` | `0x7F` |  |
| 45 | 2 | `0x036` | `0x0F` |  |
| 46 | 2 | `0x037` | `0x00` |  |
| 47 | 2 | `0x038` | `0xE0` |  |
| 48 | 2 | `0x039` | `0x00` |  |
| 49 | 2 | `0x03D` | `0x00` |  |
| 50 | 2 | `0x03E` | `0x00` |  |
| 51 | 2 | `0x03F` | `0x01` |  |
| 52 | 2 | `0x052` | `0x0B` |  |
| 53 | 2 | `0x053` | `0x01` |  |
| 54 | 2 | `0x054` | `0x03` |  |
| 55 | 2 | `0x055` | `0x05` |  |
| 56 | 2 | `0x056` | `0x07` |  |
| 57 | 2 | `0x057` | `0x09` |  |
| 58 | 2 | `0x058` | `0x09` |  |
| 59 | 2 | `0x059` | `0x09` |  |
| 60 | 2 | `0x05A` | `0x1B` |  |
| 61 | 2 | `0x05B` | `0x00` |  |
| 62 | 2 | `0x05C` | `0x80` |  |
| 63 | 2 | `0x05F` | `0x30` |  |
| 64 | 3 | `0x060` | `0x00` |  |
| 65 | 3 | `0x061` | `0x01` |  |
| 66 | 3 | `0x07D` | `0x00` |  |
| 67 | 3 | `0x07E` | `0x55` |  |
| 68 | 3 | `0x07F` | `0xF0` |  |
| 69 | 3 | `0x080` | `0x00` |  |
| 70 | 3 | `0x081` | `0x3C` |  |
| 71 | 3 | `0x082` | `0x00` |  |
| 72 | 3 | `0x083` | `0x00` |  |
| 73 | 3 | `0x084` | `0x14` |  |
| 74 | 3 | `0x085` | `0x00` |  |
| 75 | 3 | `0x086` | `0x27` |  |
| 76 | 3 | `0x087` | `0xEC` |  |
| 77 | 3 | `0x088` | `0x00` |  |
| 78 | 3 | `0x089` | `0x52` |  |
| 79 | 3 | `0x08A` | `0x00` |  |
| 80 | 3 | `0x08B` | `0x07` |  |
| 81 | 3 | `0x08C` | `0xD0` |  |
| 82 | 3 | `0x08E` | `0x0F` |  |
| 83 | 3 | `0x08F` | `0xA0` |  |
| 84 | 3 | `0x090` | `0x00` |  |
| 85 | 3 | `0x091` | `0x17` |  |
| 86 | 3 | `0x092` | `0x70` |  |
| 87 | 3 | `0x093` | `0x00` |  |
| 88 | 3 | `0x094` | `0x00` |  |
| 89 | 3 | `0x095` | `0x00` |  |
| 90 | 3 | `0x096` | `0x00` |  |
| 91 | 3 | `0x097` | `0x00` |  |
| 92 | 3 | `0x098` | `0x00` |  |
| 93 | 3 | `0x09B` | `0x7F` |  |
| 94 | 3 | `0x09C` | `0x7F` |  |
| 95 | 3 | `0x09F` | `0x00` |  |
| 96 | 4 | `0x0A1` | `0x90` |  |
| 97 | 4 | `0x0A2` | `0x00` |  |
| 98 | 4 | `0x0A3` | `0x01` |  |
| 99 | 4 | `0x0A4` | `0x00` |  |
| 100 | 4 | `0x0A5` | `0x04` |  |
| 101 | 4 | `0x0A6` | `0x00` |  |
| 102 | 4 | `0x0A7` | `0xF0` |  |
| 103 | 4 | `0x0A8` | `0x00` |  |
| 104 | 4 | `0x0A9` | `0xF0` |  |
| 105 | 4 | `0x0AA` | `0x00` |  |
| 106 | 4 | `0x0AB` | `0x04` |  |
| 107 | 4 | `0x0AC` | `0x00` |  |
| 108 | 4 | `0x0AD` | `0x01` |  |
| 109 | 4 | `0x0AE` | `0x00` |  |
| 110 | 4 | `0x0AF` | `0x04` |  |
| 111 | 4 | `0x0B0` | `0x00` |  |
| 112 | 4 | `0x0B1` | `0x01` |  |
| 113 | 4 | `0x0B8` | `0x00` |  |
| 114 | 4 | `0x0B9` | `0x00` |  |
| 115 | 4 | `0x0BD` | `0x00` |  |
| 116 | 4 | `0x0BE` | `0x00` |  |

#### Diff between the three tables

| Index | Addr | `ASIC_INIT_REG` | `ASIC_INIT_REG_GPIO000` | `ASIC_INIT_REG_GPIO111` |
|---|---|---|---|---|
| 42 | `0x032` | `0x69` = `0110 1001` | `0x61` = `0110 0001` | `0x7D` = `0111 1101` |

Bits 4..2 of register `0x32` are `010`, `000` and `111` respectively, which
is evidently what the `GPIO000` / `GPIO111` suffixes name (**inferred** from
the bit pattern; the function names are the only evidence). Bits 0, 5 and 6
are set and bit 1 is clear in all three. Note that `Scanner_CarriageHome`,
`MyScanner_StopScan` and `MyScanner_SetScanLED` later clear bit 1 of `0x32`
and `MyScanner_SetPowerSaverStatus` toggles bit 5, and the calibration /
scan-parameter code toggles bit 0 — so `0x32` is a GPIO/output-control
register whose bits 4..2 select a sensor-side mode by resolution.

## 3. AFE initialisation

### 3.1 `AFE_INIT_REG` (ELF `.data` at `0x22b880`, 16 bytes)

Raw bytes: `00 68 01 08 02 2f 03 2f 04 2f 05 00 06 00 07 00`.
`LoadInitRegData3.c:43-47` iterates `i = 0..7` and calls
`CtlAfe_WriteAFERegister(AFE_INIT_REG[2i], AFE_INIT_REG[2i+1])`, i.e. the
array is `(afe_addr, afe_value)` byte pairs and the 16-bit data word always has
a zero high byte:

| i | AFE addr | AFE data (16-bit) |
|---|---|---|
| 0 | `0x00` | `0x0068` |
| 1 | `0x01` | `0x0008` |
| 2 | `0x02` | `0x002F` |
| 3 | `0x03` | `0x002F` |
| 4 | `0x04` | `0x002F` |
| 5 | `0x05` | `0x0000` |
| 6 | `0x06` | `0x0000` |
| 7 | `0x07` | `0x0000` |

The loop aborts on the first failed write (returns 0 from `LoadInitRegData3`).

### 3.2 Wire encoding of an AFE register write (`CtlAfe_WriteAFERegister.c`)

There is no dedicated AFE opcode. The function builds three `RegRec`s and
calls `CMDASIC_WriteMultiRegister(RegBuf, 0xc)`:

```
RegBuf[0] = { 0x51, afe_addr      }
RegBuf[1] = { 0x5d, afe_data >> 8 }
RegBuf[2] = { 0x5e, afe_data & 0xff }
```

which becomes **one** control transfer:

```
CTRL OUT 0x83 idx=0 len=6 [0x51, afe_addr, 0x5D, data_hi, 0x5E, data_lo]
```

So ASIC registers `0x51` (AFE write address), `0x5D` (data high) and `0x5E`
(data low) form the AFE write port; the serial transaction to the AFE is
presumably triggered by the write to `0x5E` (**inferred**: the ordering is
fixed by the code and the low byte is always last). The three shadow entries
`CMDASIC_RegTable[0x51/0x5d/0x5e]` are updated as a side effect.

For completeness, the read port (`CtlAfe_ReadAFERegister.c`, not used during
init) is: `CMDASIC_WriteRegister(0x50, afe_addr)` then
`CMDASIC_ReadRegister(0x106)` → high byte, `CMDASIC_ReadRegister(0x107)` →
low byte (`CTRL IN 0x18E idx=0x0622 len=2` and `CTRL IN 0x18E idx=0x0722 len=2`).

For the eight init writes the concrete payloads are:

```
51 00 5D 00 5E 68
51 01 5D 00 5E 08
51 02 5D 00 5E 2F
51 03 5D 00 5E 2F
51 04 5D 00 5E 2F
51 05 5D 00 5E 00
51 06 5D 00 5E 00
51 07 5D 00 5E 00
```

## 4. The remaining init steps

### 4.1 Constants

All inputs to steps 4.2–4.6 are initialised `.data` globals that **no function
in the corpus ever assigns** (grep for `DRAM_SEL`, `DRAM_ChipN`, `Clock_ASIC`,
`Clock_AMBA`, `Clock_Pixel`, `ShadingGain` finds only the reads in
`LoadInitRegData*` and `CtlDevice_SetScannerClock`), so they are effectively
compile-time constants:

| Symbol | ELF address | Value |
|---|---|---|
| `DRAM_SEL` | `0x22b2d4` | 4 |
| `DRAM_ChipN` | `0x22b2d8` | 1 |
| `Clock_ASIC` | `0x22b2c0` | 48 (`0x30`) |
| `Clock_AMBA` | `0x22b2c4` | 48 (`0x30`) |
| `Clock_Pixel` | `0x22b2c8` | 12 (`0x0c`) |
| `ShadingGain` | `0x22b2d0` | 4 |
| `CCD_DPIHW` | `0x22b2cc` | 1200 initially; set by `MyScanner_StartScan` to 1200 (`dwResolution <= 300`) or 2400 before open |

### 4.2 `CtlDevice_SetSDRAMType(DRAM_SEL=4, ChipN=1)` (`CtlDevice_SetSDRAMType.c`)

```
WriteRegisterBit(0x0b, start=0, nbits=3, 4)      // bits[2:0] = DRAM_SEL
if ChipN == 1: WriteRegisterBit(0x08, 7, 1, 0); WriteRegisterBit(0x08, 1, 1, 0)
if ChipN == 2: WriteRegisterBit(0x08, 7, 1, 1); WriteRegisterBit(0x08, 1, 1, 0)
if ChipN == 3: WriteRegisterBit(0x08, 7, 1, 0); WriteRegisterBit(0x08, 1, 1, 1)
```

With the shadow fresh from the init table (`0x0b = 0x8A`, `0x08 = 0x00`) the
wire is `0x0b ← 0x8C`, `0x08 ← 0x00`, `0x08 ← 0x00`.

### 4.3 `CtlDevice_SetScannerClock(Clock_ASIC=48)` (`CtlDevice_SetScannerClock.c`)

Side effect: `CtlDevice_PixelTime = Clock_Pixel / iScanClk = 12/48 = 0.25f`.

Register `0x0b` is rebuilt from the **shadow** (`CMDASIC_RegTable[0xb].nValue
& 0x1f`) OR-ed with a clock code in bits 7..5, and written with a full
`CMDASIC_WriteRegister`. Then USB-controller register `0x10` is written via
`CMDASIC_WriteUsbRegister`:

| `iScanClk` (MHz?) | reg `0x0b` bits[7:5] | USB reg `0x10` |
|---|---|---|
| 24 (`0x18`) | `000` | `0x0A` |
| 30 (`0x1e`) | `001` (`0x20`) | `0x0B` |
| 40 (`0x28`) | `010` (`0x40`) | `0x0C` |
| **48 (`0x30`)** | **`011` (`0x60`)** | **`0x0D`** |
| 60 (`0x3c`) | `100` (`0x80`) | `0x0E` |
| 80 (`0x50`) | `101` (`0xA0`) | `0x0F` |
| other | `0x20` | garbage (`bVal \| 0x0b`, vendor bug; unreachable) |

Concrete wire for 48:

```
CTRL OUT 0x83 idx=0      len=2 [0x0B, 0x6C]     // (0x8C & 0x1F) | 0x60
CTRL OUT 0x8C idx=0x0010 len=1 [0x0D]
```

**Note:** the `& 0x1f` mask drops bit 7 that the init table set in `0x0b`
(`0x8A`) and that `SetSDRAMType` preserved (`0x8C`). After this step
`0x0b = 0x6C` and the shadow agrees. Whether bit 7 is meaningful is unknown
(open question).

### 4.4 `CtlDevice_SetAMBAClock(Clock_AMBA=48)` (`CtlDevice_SetAMBAClock.c`)

Same code table as the USB-register column above, written to USB register
`0x13`: `CTRL OUT 0x8C idx=0x0013 len=1 [0x0D]`. (The `default` branch again
ORs `0x0b` into an uninitialised local — unreachable with the constant 48.)

`CMDASIC_WriteUsbRegister(nAddr, nVal)` wire encoding (`CMDASIC_WriteUsbRegister.c:13`):
`CMDIO_WriteCommand(0x8C, nAddr, 1, &nVal)` →
`bmRequestType=0x40, bRequest=0x0C, wValue=0x008C, wIndex=nAddr, wLength=1, data=[nVal]`.
No bit-8 folding, no shadow update.

### 4.5 `CtlDevice_SetScanMOD(Clock_Pixel=12)` (`CtlDevice_SetScanMOD.c`)

```
WriteRegisterBit(0x14, start=0, nbits=1, (iclkpl == 16))     // 12 → 0
code = { 12: 0, 8: 2, 16: 7, 18: 6, other: 6 }[iclkpl]       // 12 → 0
WriteRegisterBit(0x06, start=5, nbits=3, code)
```

Shadow `0x14 = 0x30`, `0x06 = 0x10` → wire `0x14 ← 0x30`, `0x06 ← 0x10`
(both writes are no-ops on the value, but the transfers are still issued).

### 4.6 `CtlADF_ADFInit()` (`CtlADF_ADFInit.c`)

```
CMDASIC_WriteRegister(0x0d, 0x02)
CMDASIC_WriteRegisterBit(0x08, start=6, nbits=1, 1)      // shadow 0x00 → 0x40
```

Wire: `0x0d ← 0x02`, `0x08 ← 0x40`. Return value is unconditionally 1 (the
write results are ignored).

### 4.7 Shading base and `CtlImage_SDRamSet_ShadingBank()`

Before calling it, `LoadInitRegData3.c:81-87` sets:

```
CtlSensor_iOptRes   = CCD_DPIHW;                           // 1200 for <=300 dpi, 2400 otherwise
CtlImage_ShadingBase = (ShadingGain == 8) ? 0x2000 : 0x4000;   // ShadingGain == 4 → 0x4000
```

`CtlImage_SDRamSet_ShadingBank` (`CtlImage_SDRamSet_ShadingBank.c:18-33`) computes
(single-precision float, then truncation):

```
x = ((res*13.0 + res*13.0) * 1.0 * 256.0) / 252.0 / 1024.0
n = (((long)x >> 2) & 0x3fffffff) + 1
shd_full_range = n * 12
shd_bank_reg[0] = 0x0a
shd_bank_reg[1] = shd_bank_reg[0] + n       (uchar arithmetic)
shd_bank_reg[2] = shd_bank_reg[1] + n
for i in 0..2:
    CMDASIC_WriteRegister(0xd0 + i, shd_bank_reg[i])
    shd_bank_address[i] = (shd_bank_reg[i] + 0x8000) * 0x2000
```

Resulting values (computed with float32 emulation; `x` is nowhere near an
integer boundary so rounding mode is irrelevant):

| `CtlSensor_iOptRes` | `x` | `n` | `shd_full_range` | `0xD0` | `0xD1` | `0xD2` | `shd_bank_address[0..2]` |
|---|---|---|---|---|---|---|---|
| **1200** (≤300 dpi) | 30.95 | 8 | 96 | `0x0A` | `0x12` | `0x1A` | `0x10014000`, `0x10024000`, `0x10034000` |
| 2400 (≥301 dpi) | 61.90 | 16 | 192 | `0x0A` | `0x1A` | `0x2A` | `0x10014000`, `0x10034000`, `0x10054000` |

`shd_bank_address` is a host-side table (`.bss` `0x22e740`, `uint[10]`) used
later by the shading upload (`CtlImage_WriteShadingColor/Gray`) as the bulk
target address; it is not sent to the device here. Only the three
`0xD0..0xD2` register writes go on the wire. Note: `(reg + 0x8000) * 0x2000`
means the bank registers hold the SDRAM address in units of 8 KiB with an
implicit `0x10000000` base — the gamma tables in 4.8 live at `0x01000000`,
a different region.

### 4.8 `CtlImage_SetDefaultGamma(1)` (`CtlImage_SetDefaultGamma.c`)

Data (`tagGammaData` = `ushort[3][256]`, `types.h:2115`), `flag == 1` branch:

```
for i in 0..254:  g[c][i] = (i + 1) << 8        // 0x0100, 0x0200, ..., 0xFF00
g[c][255] = 0xFFFF
```

identically for all three channels `c = 0,1,2` (an identity ramp, 16-bit,
little-endian in memory; the bytes are `00 01 00 02 00 03 ... 00 FF FF FF`).
(`flag != 1` produces the inverted ramp and inverted register values; not
used on the scan path.)

Register writes, in order:

| Step | Call | Wire |
|---|---|---|
| 1 | `CMDASIC_WriteTwiRegister(0xc5, 0x0000)` | `CTRL OUT 0x83 idx=0 len=4 [C5 00 C6 00]` |
| 2 | `CMDASIC_WriteTwiRegister(0xc7, 0x0000)` | `CTRL OUT 0x83 idx=0 len=4 [C7 00 C8 00]` |
| 3 | `CMDASIC_WriteTwiRegister(0xc9, 0x0000)` | `CTRL OUT 0x83 idx=0 len=4 [C9 00 CA 00]` |
| 4 | `CMDASIC_WriteRegister(0xbd, 0x00)` | `CTRL OUT 0x83 idx=0 len=2 [BD 00]` |
| 5 | `CMDASIC_WriteRegister(0xbe, 0x07)` | `CTRL OUT 0x83 idx=0 len=2 [BE 07]` |

(`CMDASIC_WriteTwiRegister(nAddr, w)` emits `{nAddr & 0xff, w >> 8}, {(nAddr
& 0xff) + 1, w & 0xff}` through `WriteMultiRegister` — high byte at `nAddr`,
low byte at `nAddr + 1`; `CMDASIC_WriteTwiRegister.c:14-18`. Note it masks the
address to 8 bits, so it cannot address the `0x1xx` bank.)

Then three bulk uploads via `CMDASIC_WriteBulkData(addr, data, 0x200)`
(`CMDASIC_WriteBulkData.c`: `CMDASIC_SetupBulkLength(addr, len, bmode=1)` then
`CMDIO_BulkWrite`):

| Step | Channel | Setup control transfer | Bulk |
|---|---|---|---|
| 6 | 0 | `CTRL OUT 0x82 idx=1 len=8 [00 00 00 01  00 02 00 00]` | `BULK OUT 0x02 len=512` = `g[0]` |
| 7 | 1 | `CTRL OUT 0x82 idx=1 len=8 [00 02 00 01  00 02 00 00]` | `BULK OUT 0x02 len=512` = `g[1]` |
| 8 | 2 | `CTRL OUT 0x82 idx=1 len=8 [00 04 00 01  00 02 00 00]` | `BULK OUT 0x02 len=512` = `g[2]` |

The setup payload is `SetupCmd { u32 addr; u32 len; }` little-endian:
addresses `0x01000000`, `0x01000200`, `0x01000400`, length `0x200`.
`CMDIO_BulkWrite` (`CMDIO_BulkWrite.c:13`) is
`libusb_bulk_transfer(hdl, 0x02, buf, 512, &n, 30000)` and, as noted in 01,
ignores the transferred-byte count.

### 4.9 Post-init bit writes inside `LoadInitRegData3` (`CtlDevice_LoadInitRegData3.c:101-104`)

Return values ignored.

| Call | Shadow before | Written |
|---|---|---|
| `WriteRegisterBit(0x34, start=2, nbits=2, 0)` | `0x00` | `0x34 ← 0x00` |
| `WriteRegisterBit(0x34, start=5, nbits=2, 0)` | `0x00` | `0x34 ← 0x00` |
| `WriteRegisterBit(0x35, start=0, nbits=1, 0)` | `0x7F` | `0x35 ← 0x7E` |
| `WriteRegisterBit(0x35, start=1, nbits=1, 1)` | `0x7E` | `0x35 ← 0x7E` |

### 4.10 Complete ordered wire sequence for `Api_InitiateASIC3(300, 300)`

All control transfers are `bmRequestType=0x40, bRequest=0x0C`; "reg a ← v"
means `CTRL OUT 0x83 idx=0 len=2 [a, v]`.

| Step | Function | Wire operation | Address/value | Notes |
|---|---|---|---|---|
| 1 | `CMDASIC_WriteMultiRegister` | `CTRL OUT 0x83 idx=0 len=64` | `ASIC_INIT_REG[0..31]` | 2.3 |
| 2 | " | `CTRL OUT 0x83 idx=0 len=64` | `ASIC_INIT_REG[32..63]` | |
| 3 | " | `CTRL OUT 0x83 idx=0 len=64` | `ASIC_INIT_REG[64..95]` | |
| 4 | " | `CTRL OUT 0x83 idx=0 len=42` | `ASIC_INIT_REG[96..116]` | |
| 5–12 | `CtlAfe_WriteAFERegister` ×8 | `CTRL OUT 0x83 idx=0 len=6` | `[51 aa 5D 00 5E vv]` for `(aa,vv)` = (00,68) (01,08) (02,2F) (03,2F) (04,2F) (05,00) (06,00) (07,00) | 3.2 |
| 13 | `CtlDevice_SetSDRAMType` | reg write | `0x0B ← 0x8C` | bits[2:0]=4 |
| 14 | " | reg write | `0x08 ← 0x00` | bit7=0 |
| 15 | " | reg write | `0x08 ← 0x00` | bit1=0 |
| 16 | `CtlDevice_SetScannerClock` | reg write | `0x0B ← 0x6C` | bits[7:5]=011, bit7 of init value lost |
| 17 | " | `CTRL OUT 0x8C idx=0x10 len=1` | `[0x0D]` | USB reg 0x10 |
| 18 | `CtlDevice_SetAMBAClock` | `CTRL OUT 0x8C idx=0x13 len=1` | `[0x0D]` | USB reg 0x13 |
| 19 | `CtlDevice_SetScanMOD` | reg write | `0x14 ← 0x30` | bit0=0 (no change) |
| 20 | " | reg write | `0x06 ← 0x10` | bits[7:5]=0 (no change) |
| 21 | `CtlADF_ADFInit` | reg write | `0x0D ← 0x02` | |
| 22 | " | reg write | `0x08 ← 0x40` | bit6=1 |
| 23 | `CtlImage_SDRamSet_ShadingBank` | reg write | `0xD0 ← 0x0A` | 2400: same |
| 24 | " | reg write | `0xD1 ← 0x12` | 2400: `0x1A` |
| 25 | " | reg write | `0xD2 ← 0x1A` | 2400: `0x2A` |
| 26 | `CtlImage_SetDefaultGamma` | `CTRL OUT 0x83 idx=0 len=4` | `[C5 00 C6 00]` | |
| 27 | " | `CTRL OUT 0x83 idx=0 len=4` | `[C7 00 C8 00]` | |
| 28 | " | `CTRL OUT 0x83 idx=0 len=4` | `[C9 00 CA 00]` | |
| 29 | " | reg write | `0xBD ← 0x00` | |
| 30 | " | reg write | `0xBE ← 0x07` | |
| 31 | " | `CTRL OUT 0x82 idx=1 len=8` | `addr=0x01000000 len=0x200` | bulk setup, write mode |
| 32 | " | `BULK OUT 0x02 len=512` | identity ramp | channel 0 |
| 33 | " | `CTRL OUT 0x82 idx=1 len=8` | `addr=0x01000200 len=0x200` | |
| 34 | " | `BULK OUT 0x02 len=512` | identity ramp | channel 1 |
| 35 | " | `CTRL OUT 0x82 idx=1 len=8` | `addr=0x01000400 len=0x200` | |
| 36 | " | `BULK OUT 0x02 len=512` | identity ramp | channel 2 |
| 37 | `LoadInitRegData3` | reg write | `0x34 ← 0x00` | bits[3:2]=0 |
| 38 | " | reg write | `0x34 ← 0x00` | bits[6:5]=0 |
| 39 | " | reg write | `0x35 ← 0x7E` | bit0=0 |
| 40 | " | reg write | `0x35 ← 0x7E` | bit1=1 |

Any failure in steps 1–36 aborts with return 0 (except step 21–22 whose
results are not checked); steps 37–40 cannot fail the call. When
`CMDIO_iConnectType == 1` every `CTRL OUT` above is additionally followed by
the `0x8E/idx=0x20` status poll described in 01/03.

For other resolutions the only changes are: step 1–4 payload entry 42
(`0x32`: `0x61` for 1200/1200, `0x7D` for 2400/2400 and 1200/2400) and
steps 24–25 (`0x1A`/`0x2A` when `CCD_DPIHW == 2400`, i.e. `dwResolution > 300`).

## 5. Register-bit writes around init in the `MyScanner_*` layer

### 5.1 `MyScanner_StartScan` (`MyScanner_StartScan.c:29`)

Immediately after `Api_InitiateASIC3` succeeds:

```
CMDASIC_WriteRegisterBit(0x3a, start=2, nbits=1, 1)      // set bit 2 of 0x3A
```

Register `0x3A` is **not** in any init table, so its shadow is whatever was
last written in this process: `0x00` in a fresh process (`.bss`), or `0x00`
after a preceding `MyScanner_StopScan`/`Scanner_CarriageHome`/`SetScanLED(off)`
(all of which clear the same bit). The wire is therefore `0x3A ← 0x04`.
Register `0x3A` bit 2 is only ever touched by these four functions (grep of
`WriteRegisterBit(0x3a` over the corpus), always paired with "lamp/LED/scan
on-off" style operations; its physical meaning is **unknown** (likely a scan
or LED enable — inferred from `MyScanner_SetScanLED` setting it to 1 for LED
on and 0 for off).

The next register traffic after this is calibration (`Scanner_CheckCalibration`,
`Scanner_Calibration2` / `Scanner_CalibrationFromFile`), out of scope here.

### 5.2 `Initialize` → `MyScanner_CarriageHome` → `Scanner_CarriageHome`

`Initialize()` (`Initialize.c`) does, in order:

1. `MyScanner_OpenDeviceForUnknowStatus()` = `CMDIO_OpenDevice2(0x2e5a,0xd011)`
   immediately followed by `CMDIO_CloseDevice()` (open, feature probe,
   USB reset, close). Failure → `g_DeviceStatus = PowerOff`, return 0.
2. `MyScanner_CheckCarriageAtHome()` = `Api_OpenDevice2`, `Api_CheckMotorinHome`
   (`CTRL IN 0x18E idx=0x0122 len=2` → reg `0x101`; home = bit 3 set),
   `Api_CloseDevice`.
3. If not at home: `MyScanner_CarriageHome()` = `Api_OpenDevice2`,
   `Scanner_CarriageHome()`, `Api_CloseDevice`.

`StartScan1` also calls `MyScanner_CarriageHome()` unconditionally before
every scan (section 0), so the sequence below runs whenever the carriage is
not already at home.

`Scanner_CarriageHome` (`Scanner_CarriageHome.c`):

```
Api_CheckMotorinHome(&home)          // reg 0x101 bit 3
if home: return
Api_GetMotorStatus(&moving)          // reg 0x101 bit 0
if !moving:
    Api_InitiateASIC()               // == section 4.10 with the base table, regardless of dpi
    WriteRegisterBit(0x3a, 2, 1, 0)  // 0x3A bit2 = 0   → wire 0x3A ← 0x00 (shadow 0x00 or 0x04)
    WriteRegisterBit(0x35, 1, 1, 1)  // 0x35 bit1 = 1   → wire 0x35 ← 0x7E (already set by 4.9)
    WriteRegisterBit(0x32, 1, 1, 0)  // 0x32 bit1 = 0   → wire 0x32 ← 0x69 (bit already clear)
    Scanner_SwitchLampPower(0)
    Api_MoveMotor2(1900.0, 12000, 0, 4)
    Api_WaitMotorStop()
if moving:
    WriteRegisterBit(0x3a, 2, 1, 0)  // same three writes, no re-init
    WriteRegisterBit(0x35, 1, 1, 1)
    WriteRegisterBit(0x32, 1, 1, 0)
    Api_WaitMotorStop()
```

The same `0x3A.2=0, 0x35.1=1, 0x32.1=0` triplet is issued by
`MyScanner_StopScan.c:30-32` and `MyScanner_SetScanLED.c:19-21` (LED off);
`MyScanner_SetScanLED` (LED on) issues only `0x3A.2=1`. The motor move and
lamp calls are documented separately.

**Caveat for the `moving` branch:** because `Api_CloseDevice` performs
`libusb_reset_device` and no init is run in that branch, the shadow values
used for the three RMW writes (`0x35 = 0x7E`, `0x32 = 0x69`) are only correct
if an earlier `Api_InitiateASIC*` ran in the same process; in a fresh process
the shadow is zero and the wire would be `0x35 ← 0x02`, `0x32 ← 0x00`.

## 6. The register shadow `CMDASIC_RegTable`

- Symbol: `CMDASIC_RegTable`, `.bss`, address `0x22df00`, size `0x800` =
  **512 × `RegRec`** (4 bytes each) — one entry per 9-bit register address.
  Being `.bss`, it is **all zeros at process start**; there is no
  initialiser and no function that fills it from the device.
- The only writers are `CMDASIC_WriteRegister` (`CMDASIC_WriteRegister.c:14`,
  `RegTable[nAddr].nValue = nData` *before* the transfer, unconditionally,
  even if the transfer then fails) and `CMDASIC_WriteMultiRegister`
  (`CMDASIC_WriteMultiRegister.c:24`, same, per entry). Grep for
  `CMDASIC_RegTable` across the corpus finds no other assignment.
- `CMDASIC_WriteRegisterBit(nAddr, StartBit, Bitn, iData)` (`CMDASIC_WriteRegisterBit.c:17-23`)
  builds a mask of `Bitn` bits starting at `StartBit`, clears those bits in
  the shadow value, ORs in `iData << StartBit`, and writes the result with
  `CMDASIC_WriteRegister`. It **never reads the device**; the read side of the read-modify-write is
  the host shadow. `iData` is not masked, so a caller passing a value wider
  than `Bitn` bits would corrupt neighbouring bits (no init-path caller does).
- Readers of the shadow besides `WriteRegisterBit`: `CtlDevice_SetScannerClock`
  (reg `0x0b`), `CtlImage_SDRamSet_ImageBuffer`, `CtlImage_WriteShadingColor/Gray`,
  `CtlImage_SetDefaultShading`, `CtlImage_SetMAXWD`, `CtlLamp_SetLampWatchDog`,
  `CtlMotor_CalculateFeedReg`, `CtlMotor_SetMotorVref(_2)`, `CtlMotor_SetZ1Z2Value`,
  `CtlSensor_SetDummyPixel` (file list from grep; contents belong to later
  documents).
- `LoadInitRegData3` populates 117 of the 512 entries (those in the table)
  via `WriteMultiRegister`; the subsequent init steps populate `0x51, 0x5d,
  0x5e, 0xd0-0xd2, 0xc5-0xca, 0xbd, 0xbe`. **Registers never written in the
  process read back as `0x00` from the shadow** — relevant for `0x3A` (5.1)
  and for the `0x1xx` bank, none of which is ever written through the shadow
  on the init path.
- The shadow is never cleared: it persists across `Api_CloseDevice` /
  `Api_OpenDevice2` cycles even though `CMDIO_CloseDevice` resets the USB
  device. A Rust port should either mirror this exactly (process-lifetime
  shadow, 512 bytes, zero-initialised, updated before each write) or, more
  defensibly, re-run the full init after every open so the shadow is always
  derived from a known device state.

## 7. Open questions

1. **Hardware verification** of every value above — in particular whether the
   four 64/64/64/42-byte `0x83` batches are accepted as-is, and whether the
   `0x8C` USB-register writes (`0x10 ← 0x0D`, `0x13 ← 0x0D`) are required
   or merely tolerated.
2. Register `0x0b` bit 7: set to 1 by the init table (`0x8A`), then dropped
   by `SetScannerClock`'s `& 0x1f` mask (`0x6C`). Intentional, or a vendor
   bug the hardware happens to tolerate?
3. Meaning of `0x32` bits 4..2 ("GPIO000"/"GPIO111"): which sensor/AFE
   configuration do the three codes select, and why does 1200×2400 share the
   2400×2400 code while 2400×1200 falls back to the default?
4. Meaning of `0x3A` bit 2 (set after init in `StartScan`, cleared in
   `StopScan`/`CarriageHome`/`SetScanLED(off)`).
5. The units of `Clock_ASIC`/`Clock_AMBA` (48) and `Clock_Pixel` (12) —
   presumably MHz, and the encoding of the USB-register clock codes
   `0x0A..0x0F`; unverifiable statically.
6. Whether the AFE serial transaction is triggered by the write to `0x5E`
   (assumed) and whether the AFE register map (`0x00..0x07`) corresponds to a
   known AFE part; the gain/offset setters (`CtlAfe_SetAFEGain/Offset`) will
   need the same mechanism.
7. The `moving` branch of `Scanner_CarriageHome` relies on a stale shadow
   after a USB reset (5.2 caveat) — the real wire values depend on process
   history, which static analysis cannot resolve.
8. `CMDIO_iConnectType` is still the unresolved speed/retry flag from 01/03;
   it decides whether each of the ~40 control writes above is followed by a
   status poll.
