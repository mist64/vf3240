# VF3240 USB transport layer

Status: **verified on hardware (2026-10-13):** register reads/writes, 4-byte reads, batched writes and bulk setup + bulk IN/OUT all work as described.

Source of truth: `libviisanvf3240dr.so`, compilation unit
`../sane-A3/MSTScanSDK/asiccmd/usbio.c` and `asiccmd.c`, recovered with DWARF.

## Device identity

| | |
|---|---|
| VID:PID | `2e5a:d011` |
| Bulk IN endpoint | `0x81` |
| Bulk OUT endpoint | `0x02` |
| Interrupt IN endpoint | `0x83` (max packet 1 byte, interval 8) — button events |

Endpoints are hardcoded in `CMDIO_BulkRead` / `CMDIO_BulkWrite`, not discovered
from the descriptors. The real descriptors (hardware, 2026-10-13,
`examples/descriptors`): one configuration, one vendor-class interface
(`0xff/0xff/0xff`), the three endpoints above, 10 mA declared. The vendor reads
`0x83` with `libusb_bulk_transfer` in `CMDIO_InterruptIoCtl` (called only by
`CMDIO_GetButtonStatus`, which has no callers); the SANE shim's button options
instead poll the control request `0x8E` with `wIndex = 0x0021`
(`MyScanner_GetKeyStatus`, `02-opcode-map.md`), which works: it latches one bit per button (`0x04/0x08/0x20/0x40`) until read. On hardware (2026-10-13, macOS/libusb) the interrupt endpoint delivered **no packets** during presses while it was being read with 20–50 ms timeouts; whether it ever reports is unknown.

## Control channel

Every non-bulk operation is a **vendor control transfer with `bRequest = 0x0C`**.
The operation is selected by `wValue`; `wIndex` carries an address or subcommand.

| Direction | `bmRequestType` | `bRequest` |
|---|---|---|
| Host → device | `0x40` | `0x0C` |
| Device → host | `0xC0` | `0x0C` |

`CMDIO_WriteCommand(nCmd, nIdx, nCmdLen, pData)` →
`libusb_control_transfer(h, 0x40, 0x0C, nCmd, nIdx, pData, nCmdLen, timeout)`

`CMDIO_ReadCommand(nCmd, nIdx, nCmdLen, pData)` →
`libusb_control_transfer(h, 0xC0, 0x0C, nCmd, nIdx, pData, nCmdLen, timeout)`

### Write retry behaviour

When `CMDIO_iConnectType == 1` (evidence suggests this distinguishes USB speeds;
**unconfirmed**), writes are retried up to 3 times, each followed by
`CMDIO_GetCtrlWriteStatus()` until it reports success. Otherwise a single
transfer is issued and success is assumed from the libusb return code alone.

## ASIC register access

Registers are 9-bit addressed. Bit 8 of the address is folded into the **high
byte of `wValue`**, bits 7..0 travel in the payload or `wIndex`.

### Write — `CMDASIC_WriteRegister(nAddr, nData)`

```
wValue  = 0x0083 | ((nAddr >> 8) & 1) << 8      // 0x0083 or 0x0183
wIndex  = 0x0000
payload = [ nAddr & 0xFF, nData ]               // 2 bytes, host -> device
```

The driver also maintains a host-side shadow copy of every register in
`CMDASIC_RegTable[nAddr].nValue`, updated before the transfer. Read-modify-write
helpers such as `CMDASIC_WriteRegisterBit` operate on this shadow rather than
reading the device back.

### Read — `CMDASIC_ReadRegister(nAddr, *out)`

```
wValue  = 0x008E | ((nAddr >> 8) & 1) << 8      // 0x008E or 0x018E
wIndex  = ((nAddr & 0xFF) << 8) | 0x22
payload = 2 bytes, device -> host
```

Response validation: **`payload[1]` must equal `0x55` (ASCII `'U'`)**, acting as
a validity magic. `payload[0]` is the register value. Any other value in
`payload[1]` is treated as a failed read.

### Wider accesses

`CMDASIC_ReadFourRegister(nAddr, &dwResult)` reads a 32-bit value spanning four
consecutive registers starting at `nAddr`. Same framing as a single-register
read but with a 5-byte reply:

```
wValue  = 0x008E | ((nAddr >> 8) & 1) << 8
wIndex  = ((nAddr & 0xFF) << 8) | 0x22
payload = 5 bytes, device -> host
```

`payload[4]` must be `0x55`; the value is assembled **big-endian** from
`payload[0..4]`: `result = p[0] << 24 | p[1] << 16 | p[2] << 8 | p[3]`
(the decompiled code stores `str[3]` into the result's low byte and `str[0]`
into the high byte). Used on `0x102` by `ScanCtrl_ChkScanDataReady`.

`CMDASIC_ReadTwiRegister` / `ReadTriRegister` and the write variants exist and
follow the same pattern with 3/4-byte replies; not needed by the scan path and
not extracted.

### Batched register writes

`CMDASIC_WriteMultiRegister(pBuf, dwLen)` takes an array of 4-byte records
`{u16 addr, u8 value, u8 pad}` (`dwLen` in bytes, `dwLen / 4` records). It:

1. Updates the host shadow `CMDASIC_RegTable[addr].nValue = value` for every
   record, before anything is sent.
2. Sorts records into two buffers by address bit 8: addresses `< 0x100` go to
   the `0x83` buffer, addresses `>= 0x100` to the `0x183` buffer. Each record
   contributes two payload bytes `[addr & 0xFF, value]`.
3. Whenever a buffer reaches 64 bytes (32 registers) it is flushed as one
   `CMDIO_WriteCommand(opcode, 0, 64, buf)`; leftovers are flushed at the end,
   low-address buffer first, then the `0x183` buffer.

So one control transfer carries up to 32 `[addr, value]` pairs with
`wIndex = 0`; the single-register write is the one-pair case. Order within an
address class is preserved; order *between* classes is not (all `< 0x100`
leftovers go before all `>= 0x100` leftovers).

## Bulk memory path

Every bulk transfer is preceded by a length/address setup on the control
channel, `CMDASIC_SetupBulkLength(addr, len, bmode)`:

```
wValue  = 0x0082
wIndex  = bmode          (0 = device -> host read, 1 = host -> device write)
payload = 8 bytes: u32 addr (little-endian), u32 len (little-endian)
```

(`SetupCmd` is a union of `uchar buf[8]` / `uint val[2]` filled as
`val[0] = addr; val[1] = len` on x86-64, hence little-endian.)

Then exactly one bulk transfer of `len` bytes follows:

| Helper | Setup | Transfer |
|---|---|---|
| `CMDASIC_ReadRawImage(buf, len)` | `SetupBulkLength(0x10000000, len, 0)` | `CMDIO_BulkRead` on EP `0x81` |
| `CMDASIC_ReadBulkData(addr, buf, len)` | `SetupBulkLength(addr, len, 0)` | `CMDIO_BulkRead` on EP `0x81` (no callers) |
| `CMDASIC_WriteBulkData(addr, data, len)` | `SetupBulkLength(addr, len, 1)` | `CMDIO_BulkWrite` on EP `0x02` |

`CMDIO_BulkRead` is a single `libusb_bulk_transfer(hdl, 0x81, buf, len,
&trans_bytes, libusb_timeout)` and returns success iff the libusb return code
is non-negative. Neither bulk helper inspects `trans_bytes`, so short transfers
are silently ignored by the vendor driver. Our implementation must not copy
that bug.

Image data is always read from address `0x10000000`; the same address is the
first motor slope-table *write* target, so the address space is presumably
direction-dependent (see `06-motor.md`).

## Secondary control paths

`CMDASIC_AHBWriteAddr`, `CMDASIC_APBWriteAddr`, `CMDASIC_APBWriteData`,
`CMDASIC_APBReadData` and `CMDASIC_ReadAsicBaseAddr` expose the ASIC's internal
AHB/APB buses — an ARM-style SoC bus fabric behind the register file. Not yet
decoded.

## Open questions

- Meaning of `CMDIO_iConnectType` (USB 2.0 vs 3.0? set by `CMDIO_GetConnectionType`).
- Opcode map beyond `0x83` (reg write) and `0x8E` (reg read): see `02-opcode-map.md`.
- Whether `0x22` in the read `wIndex` is a fixed magic or a sub-opcode.
- EEPROM layout read by `CMDIO_OpenDevice_ReadEEPROM`.
