# VF3240 power saving

Status: **verified on hardware (2026-10-13):** clearing `0x32` bit 5 reads back clear and keeps the scanner up. **But every `Api_InitiateASIC3` sets it again** (init table entry for `0x32` is `0x61`/`0x7D`, bit 5 = 1; after a scan `0x32` reads `0xe0`), so the disable must be re-issued after the last init of a session or the unit powers down 5–10 min after the last scan — which is what happened twice during button-tool use before the fix in `job::scan`.

Source of truth: `re/exports/libviisanvf3240dr.so/decomp/SetPowerSaving.c`,
`MyScanner_SetPowerSaverStatus.c`, `MyScanner_GetPowerSaverStatus.c`.

## Why it matters

The scanner powers itself down after several minutes idle (observed by the
user as 5–10 minutes). The SDK export `SetPowerSaving(BOOL)` is what the SANE
shim calls from `sane_close`.

## Mechanism

`MyScanner_SetPowerSaverStatus(bPowerSaver)` opens the device with
`CMDIO_OpenDevice2(0x2e5a, 0xd011)`, then does two shadow-based bit writes
(`CMDASIC_WriteRegisterBit(addr, start_bit, width, value)`), then closes:

| `bPowerSaver` | Write 1 | Write 2 |
|---|---|---|
| 1 (enable)  | `0x35` bit 5 := 1 | `0x32` bit 5 := 1 |
| 0 (disable) | `0x35` bit 5 := 1 | `0x32` bit 5 := 0 |

So `0x35` bit 5 is always set (presumably an enable/arm bit for the feature or
the clock it depends on) and **`0x32` bit 5 selects power saving on/off**.

`MyScanner_GetPowerSaverStatus` builds an INI path but always reports
"enabled" without touching the device.

## Power-on state (hardware, 2026-10-13)

From `docs/hardware/2026-10-13-register-dump-poweron.txt`: `0x32 = 0xe0`
(bit 5 set, i.e. power saving enabled), `0x35 = 0x7f` (bit 5 set). The
homing and init paths (see `04-asic-init.md`) also touch bits on `0x32`,
`0x35` and `0x3a`.

## Caution for reimplementation

`CMDASIC_WriteRegisterBit` composes the new byte from the **host shadow**, not
a device read. The vendor's shadow is populated by the init table write; a
fresh process that has not run init would compute from zeros. Our `Registers`
layer has the same shadow semantics, so seed the shadow with a real read of
`0x32`/`0x35` before the bit write when init has not run.
