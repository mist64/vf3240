# Driver provenance

The protocol was recovered from the vendor's Linux SANE driver package,
downloaded 2026-08-26 from the official product page
<https://www.viisan.com/en/product/VF3240.html>:

| Package | Version | Bytes | SHA-256 |
|---|---|---|---|
| `https://viisan.oss-us-west-1.aliyuncs.com/en/file/t91/VF3240 Linux sane driver 1.0.2.0.zip` | 1.0.2.0 | 1283596 | `680d4c583400f935a2671f1c9a3be9425b9d0becb5b6beebf4918558aa369edc` |

Inside it, `libviisanvf3240dr.so` (420 KB ELF, 368 named functions) carries
full DWARF debug information and calls libusb directly; it is the protocol.
`libsane-aviisandrvf3240.so` is a thin SANE shim over it. The vendor's
macOS and Windows drivers wrap the same USB protocol and were not used.
