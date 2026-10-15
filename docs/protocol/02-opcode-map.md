# VF3240 control opcode map

Status: **derived from static analysis, not yet verified against hardware.**

Source of truth: `libviisanvf3240dr.so` decompiled corpus at
`re/exports/libviisanvf3240dr.so/decomp/`. This document extends
[`01-usb-transport.md`](01-usb-transport.md) and does not contradict it; it
fills in the "opcode map beyond `0x83`/`0x8E`" gap that document's Open
Questions section calls out.

## Method

Every control transfer in this driver goes through exactly two wrapper
functions: `CMDIO_WriteCommand(nCmd, nIdx, nCmdLen, pCmdData)` and
`CMDIO_ReadCommand(nCmd, nIdx, nCmdLen, pCmdData)`. Confirmed by grepping the
full decompiled corpus for `libusb_control_transfer`: it appears only inside
`CMDIO_WriteCommand.c` and `CMDIO_ReadCommand.c` themselves (plus the
`libusb_control_transfer.c` stub for the imported symbol). No call site
bypasses these wrappers, so enumerating all callers of the two wrappers
enumerates all control traffic.

```
grep -n 'CMDIO_WriteCommand(\|CMDIO_ReadCommand(' *.c | grep -v '^CMDIO_WriteCommand.c\|^CMDIO_ReadCommand.c'
```

This yields **24 call sites** across **20 files**, using **8 distinct
`wValue` low-byte opcodes**: `0x82, 0x83, 0x84, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E`.
(The task brief estimated 40–80 call sites; the actual, exhaustively verified
count is 24 — there are simply fewer distinct command-issuing functions in
this driver than that estimate assumed. Every file that matched the grep
appears in the Source function column below; see the Coverage section.)

The `0x83` (register write) and `0x8E` (register read) opcodes are the only
ones observed with the address-bit-8 folding described in
`01-usb-transport.md` (`CONCAT71(((nAddr>>8)&1), 0x8_)`, i.e. wValue high
byte becomes `0x01` when address bit 8 is set). None of the other opcodes
below are ever folded this way in the corpus — they always appear as flat
8-bit literals in `wValue`'s low byte, wValue high byte `0x00`.

## Opcode table

| Opcode (wValue low) | Dir | wIndex | Len | Payload | Source function |
|---|---|---|---|---|---|
| `0x82` | out | `bmode` (`0x00`=read setup, `0x01`=write setup) | 8 | `[addr0..addr3, len0..len3]` (two little-endian `uint32`: target address, transfer length) | `CMDASIC_SetupBulkLength` |
| `0x83` | out | `0x0000` | 2 | `[addr_lo, data]` | `CMDASIC_WriteRegister` |
| `0x83` / `0x183` | out | `0x0000` | up to `0x40` | packed `[addr_lo, data]` pairs, batched per address-bit-8 bank (`0x83` for bit8=0 addresses, `0x183` for bit8=1) | `CMDASIC_WriteMultiRegister` |
| `0x84` | in | `And_Factor & ((usAddr + length) \| Or_Factor)` — see NVRAM note below | up to `0x40` | raw NVRAM bytes | `ReadNVRAM` |
| `0x8A` | in | `0x0001` | 4 | ASIC base address (4 bytes) | `CMDASIC_ReadAsicBaseAddr` |
| `0x8A` | in | `0x0005` | 4 | APB data register read | `CMDASIC_APBReadData` |
| `0x8A` | in | `0x26FE` | 1 | single status/key byte **(inferred: "EEPROM key" — see below)** | `CMDIO_OpenDevice_ReadEEPROM` |
| `0x8B` | out | `0x0003` | 4 | AHB write-address register | `CMDASIC_AHBWriteAddr` |
| `0x8B` | out | `0x0005` | 4 | APB write-data register | `CMDASIC_APBWriteData` |
| `0x8B` | out | `0x0006` | 4 | APB write-address register | `CMDASIC_APBWriteAddr` |
| `0x8B` | out | `0x26FE` | 1 | `[0x00]` fixed — clears the same key/status byte read via `0x8A`/`0x26FE` **(inferred meaning)** | `CMDIO_OpenDevice_ReadEEPROM` |
| `0x8C` | out | `nAddr` (raw address, direct passthrough, no bit-8 folding) | 1 | `[value]` **(inferred: "USB-controller-local register", distinct from the 9-bit ASIC register file — no corresponding read opcode was found to confirm this)** | `CMDASIC_WriteUsbRegister` |
| `0x8D` | out | `0x0000` | 1 | `[0x01]` fixed, independent of caller's `flush` argument **(inferred: cancels/flushes an in-flight bulk read; exact meaning of the byte value is unknown)** | `CMDIO_CancelBulkRead` |
| `0x8E` | in | `(addr_lo << 8) \| 0x22` | 2 | `[value, 0x55]` | `CMDASIC_ReadRegister` |
| `0x8E` | in | `(addr_lo << 8) \| 0x22` | 3 | `[value_hi, value_lo, 0x55]` (2-byte register) | `CMDASIC_ReadTwiRegister` |
| `0x8E` | in | `(addr_lo << 8) \| 0x22` | 4 | `[b2, b1, b0, 0x55]` (3-byte register) | `CMDASIC_ReadTriRegister` |
| `0x8E` | in | `(addr_lo << 8) \| 0x22` | 5 | `[b3, b2, b1, b0, 0x55]` (4-byte register) | `CMDASIC_ReadFourRegister` |
| `0x8E` | in | `0x0000` | 1 | `[flags]`; `type = (flags & 0x08) == 0` **(inferred: physical meaning of bit 3 is unknown — see `03-device-open.md`)** | `CMDIO_GetConnectionType` |
| `0x8E` | in | `0x0020` | 1 | `[status]`; success iff `status == 0x55` | `CMDIO_GetCtrlWriteStatus` |
| `0x8E` | in | `0x0021` | 1 | `[key]` — button **event** byte, **verified on hardware (2026-10-13)**: `0x00` idle; a press latches `0x04`, `0x08`, `0x20` or `0x40` (one bit per button, same bits as register `0x31`), returned once and cleared by the read; a held button is reported once. The power button's short press latches **nothing** here (it only pulses register `0x33` bit 1 low for ~0.3 s). Live (level) state of the four buttons is register `0x31`, active low (`0xff` idle, bit cleared while held). See `docs/hardware/2026-10-13-buttons-regwatch.txt`. | `MyScanner_GetKeyStatus`, `MyScanner_GetKeyStatus2` |

All 20 opcode-issuing files above appear in this table (see Coverage below);
`CMDASIC_WriteRegisterBit` and `CMDASIC_WriteTwiRegister`/`WriteTriRegister`
are not listed because they issue no control transfers of their own — they
delegate to `CMDASIC_WriteRegister` / `CMDASIC_WriteMultiRegister`
respectively and so carry no new opcode.

## Notes on specific rows

### `CMDASIC_SetupBulkLength` (`0x82`) — resolves a prior open question

`01-usb-transport.md` lists `CMDASIC_SetupBulkLength(...)` with "opcode TBD".
It is `0x82`. The payload is the vendor's 8-byte `SetupCmd` union (DWARF
`types.h`), viewed as two 32-bit words: word 0 = address, word 1 = length,
both little-endian. The `bmode` parameter (passed as `wIndex`) is
directly observed to be `0` from `CMDASIC_ReadBulkData` and
`CMDASIC_ReadRawImage`, and `1` from `CMDASIC_WriteBulkData` — this is
**proven**, not inferred, since both call sites are visible in the same
corpus (`CMDASIC_ReadBulkData.c:11`, `CMDASIC_WriteBulkData.c:11`,
`CMDASIC_ReadRawImage.c:10`).

### `CMDASIC_WriteMultiRegister` (`0x83`/`0x183`) — batch register write

Confirms rather than contradicts the existing spec's single-register write
row: it is the same opcode, issued repeatedly with a payload of interleaved
`[addr_lo, data]` pairs, flushed every 64 bytes (`rlen == 0x40`) or at the
end of the buffer. Addresses with bit 8 set are batched separately from
addresses with bit 8 clear (two parallel accumulation buffers, `addr1` for
bit8=0, `addr2` for bit8=1), each flushed with its own folded opcode (`0x83`
vs `0x183`). Source: `CMDASIC_WriteMultiRegister.c` lines 22–66.

### `CMDASIC_ReadTwiRegister`/`ReadTriRegister`/`ReadFourRegister` (`0x8E`) — resolves a prior open question

`01-usb-transport.md` states the opcodes for the 2/3/4-byte register reads
are "not yet extracted." They reuse the exact same opcode as
`CMDASIC_ReadRegister` (`0x8E`, same `wIndex` formula) with a payload one
byte longer per additional register byte, and the same `0x55` validity magic
in the final payload byte. This is **proven** directly from source, not
inferred — see `CMDASIC_ReadTwiRegister.c:17`, `CMDASIC_ReadTriRegister.c:18`,
`CMDASIC_ReadFourRegister.c:19`. No corresponding "WriteTwiRegister" opcode
exists as a raw control transfer — `CMDASIC_WriteTwiRegister` and
`CMDASIC_WriteTriRegister` instead decompose into per-byte `RegRec` entries
and call `CMDASIC_WriteMultiRegister`, so wide register writes reuse the
`0x83` opcode rather than getting one of their own.

### `0x8A` / `0x8B` — AHB/APB bus and the EEPROM key byte

`01-usb-transport.md` lists `CMDASIC_AHBWriteAddr`, `APBWriteAddr`,
`APBWriteData`, `APBReadData`, `ReadAsicBaseAddr` as "not yet decoded." The
opcodes are `0x8A` (read direction) and `0x8B` (write direction); the
specific sub-register is selected by a small literal `wIndex` (`1`, `3`,
`5`, `6`), one per named function — this mapping is **proven** by direct
citation, since each function hardcodes exactly one `wIndex` value matching
its own name.

The `0x26FE`/`0x8A`+`0x8B` pair in `CMDIO_OpenDevice_ReadEEPROM` is different
in shape from the other `0x8A`/`0x8B` uses: it moves a 1-byte payload (not
4), and its `wIndex` is a full 16-bit value rather than a small integer
selector. Whether this is the same "AHB/APB bus register" address space or a
distinct addressing mode reused under the same opcode number is
**unconfirmed**; the code reads the byte, and if bit 0 is set, immediately
writes `0x00` back to the same index. We describe this only as "a status/key
byte gating something at open time" — its physical meaning (a write-protect
latch, an EEPROM busy/lock flag, or something else) is **(inferred)** at
best and should be treated as **unknown** for firmware-safety purposes until
confirmed against hardware.

### `0x84` — `ReadNVRAM`

`ReadNVRAM(lpDataBuf, usAddr, usDataLength, nNVRAMType)` computes `wIndex` as
`And_Factor & ((usAddr + running_offset) | Or_Factor)`, where `And_Factor`
and `Or_Factor` are looked up from `nNVRAMType` (1..7):

| nNVRAMType | Or_Factor | And_Factor |
|---|---|---|
| 1 | `0xE200` | `0xE27F` |
| 2 | `0xD000` | `0xD07F` |
| 3 | `0xD000` | `0xD0FF` |
| 4 | `0xD000` | `0xD1FF` |
| 5 | `0xD000` | `0xD3FF` |
| 6 | `0xD000` | `0xD7FF` |
| 7 | `0xC000` | `0xC7FF` |

Transfers are chunked at 0x40 bytes per control read. The bit-masking
mechanics above are **proven** (transcribed directly from
`ReadNVRAM.c:17-45`); which physical memory region or device each
`nNVRAMType` value selects is **not determined** from this corpus alone and
is marked **unknown**. No `WriteNVRAM` counterpart exists anywhere in the
decompiled corpus — `0x84` is read-only as far as this driver is concerned.
Likewise no read-side counterpart to `CMDASIC_WriteUsbRegister`'s `0x8C` was
found. Both asymmetries are noted rather than filled in with a guessed
opcode, per the no-guessing ruling.

### `CMDIO_CancelBulkRead` (`0x8D`)

The function takes a `flush` parameter but never uses it — the payload byte
sent is unconditionally `0x01` regardless of the argument's value
(`CMDIO_CancelBulkRead.c:12-13`). This looks like dead vendor code (an
unused parameter), not a documented protocol subtlety; noted here so a
future implementer does not go looking for a `flush=0` behavior that does
not exist in this binary.

## Coverage check

All 20 files with a `CMDIO_WriteCommand`/`CMDIO_ReadCommand` call site are
cited above:

`CMDASIC_AHBWriteAddr`, `CMDASIC_APBReadData`, `CMDASIC_APBWriteAddr`,
`CMDASIC_APBWriteData`, `CMDASIC_ReadAsicBaseAddr`, `CMDASIC_ReadFourRegister`,
`CMDASIC_ReadRegister`, `CMDASIC_ReadTriRegister`, `CMDASIC_ReadTwiRegister`,
`CMDASIC_SetupBulkLength`, `CMDASIC_WriteMultiRegister`,
`CMDASIC_WriteRegister`, `CMDASIC_WriteUsbRegister`, `CMDIO_CancelBulkRead`,
`CMDIO_GetConnectionType`, `CMDIO_GetCtrlWriteStatus`,
`CMDIO_OpenDevice_ReadEEPROM`, `MyScanner_GetKeyStatus`,
`MyScanner_GetKeyStatus2`, `ReadNVRAM`.

## Out of scope for this document

`CMDIO_InterruptIoCtl` (used by `CMDIO_GetButtonStatus`) is **not** a control
transfer at all — despite its name, it issues `libusb_bulk_transfer(hdl,
0x83, ...)`, a bulk-shaped transfer on a **third endpoint, `0x83`**, not
listed in `01-usb-transport.md`'s endpoint table (`0x81` bulk IN, `0x02` bulk
OUT). This is worth flagging for the transport document but is out of scope
for the control-opcode map, since it never goes through `bRequest = 0x0C`.
