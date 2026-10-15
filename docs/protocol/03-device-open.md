# VF3240 device open and claim sequence

Status: **derived from static analysis, not yet verified against hardware.**

Source of truth: `re/exports/libviisanvf3240dr.so/decomp/CMDIO_OpenDevice.c`,
`CMDIO_GetDeviceFeatures.c`, `CMDIO_GetConnectionType.c`. This document
extends [`01-usb-transport.md`](01-usb-transport.md) and consumes the opcode
table in [`02-opcode-map.md`](02-opcode-map.md) for `CMDIO_GetConnectionType`'s
opcode (`0x8E`, `wIndex = 0`).

## Which function is "the" open path

There are three open-device variants in the corpus:
`CMDIO_OpenDevice`, `CMDIO_OpenDevice2`, and `CMDIO_OpenDevice_ReadEEPROM`.
The one actually reachable from the public API is `CMDIO_OpenDevice`, via a
one-line pass-through:

```
Api_OpenDevice  ->  CtlDevice_OpenDevice  ->  CMDIO_OpenDevice
```

(`CMDIO_OpenDevice2` and `CMDIO_OpenDevice_ReadEEPROM` are called from other,
unrelated entry points — `CtlDevice_OpenDevice2`/`MyScanner_*` helpers and
`MyScanner_ReadNVRAM` respectively — and are not part of the documented open
sequence below, though they share the same libusb call skeleton.)

## The open sequence

Transcribed from `CMDIO_OpenDevice.c`:

```
libusb_init(&sanei_usb_ctx)
libusb_get_device_list(sanei_usb_ctx, &devlist)
for each device:
  libusb_get_bus_number / libusb_get_device_address   (used only for log messages)
  libusb_get_device_descriptor
    match desc.idVendor == 0x2e5a && desc.idProduct == 0xd011
  libusb_open(dev, &hdl)
  libusb_get_configuration(hdl, &config)               (result unused beyond error check)
  libusb_get_device_descriptor(dev, &desc)              (re-fetched, unused beyond error check)
  libusb_get_config_descriptor(dev, 0, &config0)
  libusb_set_configuration(hdl, config0->bConfigurationValue)
  libusb_claim_interface(hdl, config0->interface->altsetting->bInterfaceNumber)
  break out of the device-list loop
libusb_free_config_descriptor(config0)
CMDIO_GetDeviceFeatures()
```

This matches the already-verified sequence in the task brief. One detail the
brief's skeleton doesn't show: `libusb_free_config_descriptor(config0)` is
called **once, after the device-list loop**, not immediately after
`libusb_claim_interface`. Because the loop `break`s as soon as a matching
device is claimed, the practical effect is the same — free happens right
after claim, then `CMDIO_GetDeviceFeatures()` runs. Source:
`CMDIO_OpenDevice.c:102-125`.

### A found-but-unchecked defect worth flagging for the Rust port

`CMDIO_OpenDevice` declares a `ufind` flag, sets it to `1` when a matching
device is claimed (`CMDIO_OpenDevice.c:119`), but **never reads it again**.
After the device-list loop — regardless of whether a match was found —
execution falls through to `libusb_free_config_descriptor(config0)` and then
unconditionally sets its result to 1 (success) at line 129. If no VID/PID match
exists in `devlist`, `config0` is never assigned by the loop body, and this
function frees an **uninitialized pointer** and reports success anyway.
`CMDIO_OpenDevice2` (a sibling, not the path used by the public API) does
not have this bug — it checks its equivalent flag
(`bIsFindMysherScanner`) before deciding success/failure. This is a genuine
vendor-driver defect, not a protocol behavior; **Task 7's `Device::open`
must check for a found device explicitly and must not copy this bug** by
returning success when no device matched.

## Error tolerances

From `libusb_set_configuration`'s error handling (`CMDIO_OpenDevice.c:80-101`):

- Return `-6` (`LIBUSB_ERROR_BUSY`) is **ignored** — logged
  ("Maybe the kernel scanner driver or usblp claims the interface? Ignoring
  this error...") and treated as success (`status = 1`).
- Return `-3` (`LIBUSB_ERROR_ACCESS`) is **fatal** — logged ("Make sure you
  run as root or set appropriate permissions"), `status` stays `0`, and the
  device handle is closed and the function returns failure.
- Any other negative return also falls through to `status != 1` and is
  fatal in the same way as `-3` (no explicit further error-code branching).

From `libusb_claim_interface`'s error handling (`CMDIO_OpenDevice.c:102-118`):

- Return `-6` is **fatal** here (unlike `set_configuration`) — logged
  ("Maybe the kernel scanner driver claims the scanner's interface?"), the
  handle is closed, and the function returns failure.
- Return `-3` is **fatal** — logged ("Make sure you run as root or set
  appropriate permissions"), same failure path.
- Note the asymmetry with `set_configuration`: only `set_configuration`
  tolerates `-6`; `claim_interface` never does. Both hints point at the same
  real-world cause on Linux — a kernel driver (e.g. `usblp` or a scanner
  class driver) holding the interface — but the driver's behavior differs
  depending on which call fails.

## The connection-type probe

`CMDIO_GetConnectionType` (called from `CMDIO_GetDeviceFeatures`, the last
step of the open sequence):

```
wValue = 0x8E, wIndex = 0x0000, 1-byte control read   (see 02-opcode-map.md)
type = (byte & 0x08) == 0
```

Source: `CMDIO_GetConnectionType.c:12-14`.

`CMDIO_GetDeviceFeatures` then uses the *raw* connection-type byte (not the
derived `type`) to pick a bulk FIFO size: `CMDIO_iConnectType == 1` selects
`CMDIO_BulkFiFoSize = 0x200`, otherwise `0x40` (`CMDIO_GetDeviceFeatures.c`).

Separately, `CMDIO_WriteCommand` branches on `CMDIO_iConnectType == 1`. This
is the very same global that `CMDIO_GetConnectionType` wrote via its output
parameter (`CMDIO_GetDeviceFeatures` calls it as
`CMDIO_GetConnectionType(&CMDIO_iConnectType)`), so `CMDIO_iConnectType` and
the `type` value derived above are one and the same: when true, it retries
the write up to 3 times, polling
`CMDIO_GetCtrlWriteStatus()` (opcode `0x8E`, `wIndex = 0x20`, success iff the
returned byte is `0x55`) between attempts and failing only after all 3
attempts fail to produce a success from `CMDIO_GetCtrlWriteStatus`; when
`CMDIO_iConnectType != 1`, a single control write is issued and its result is
trusted from the libusb return code alone, with no read-back verification.
Source: `CMDIO_WriteCommand.c:16-39`.

**The physical meaning of bit 3 (`0x08`) in the connection-type byte is
unknown.** It plausibly reflects a link-speed or bus-type distinction (e.g.
USB 2.0 vs USB 3.0, or full-speed vs high-speed), given that it also drives
the bulk FIFO size selection, but nothing in the decompiled corpus states
this explicitly, and `01-usb-transport.md` already flags
`CMDIO_iConnectType`'s meaning as an open question. We model it in later
tasks as an opaque flag named `needs_write_retry`, derived exactly as
`(byte & 0x08) == 0`, without attaching any USB-speed interpretation to it.

## Summary for Task 7's `Device::open`

1. Enumerate devices, match VID `0x2E5A` / PID `0xD011`.
2. `open` -> `get_configuration` -> `get_device_descriptor` ->
   `get_config_descriptor(0)`.
3. `set_configuration(config0.bConfigurationValue)`; tolerate
   `LIBUSB_ERROR_BUSY`, fail on anything else including
   `LIBUSB_ERROR_ACCESS`.
4. `claim_interface(config0.interface[0].altsetting[0].bInterfaceNumber)`;
   fail on any error, including `LIBUSB_ERROR_BUSY` (no tolerance here,
   unlike step 3).
5. Free the config descriptor.
6. **Explicitly fail if no matching device was found** — do not copy the
   vendor driver's defect where a not-found device is still reported as an
   open success.
7. Run the connection-type probe (`0x8E`/`wIndex=0`), derive
   `needs_write_retry = (byte & 0x08) == 0`, and record it for use by the
   control-write path (`02-opcode-map.md`'s retry behavior, also described
   in `01-usb-transport.md`'s "Write retry behaviour" section).
