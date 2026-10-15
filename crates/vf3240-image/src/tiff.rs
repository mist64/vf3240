//! Streaming 8-bit RGB classic TIFF writer (little-endian), uncompressed or
//! Deflate (compression 8) with the horizontal predictor (tag 317 = 2).
//!
//! Rows are appended as 64-row strips as they arrive; the image directory is
//! written at the end with the number of rows actually received, then the
//! header's directory pointer is patched. Classic TIFF offsets are 32-bit, so
//! a file must stay under 4 GiB; images above `BIG_THRESHOLD` of pixel data
//! (a full-bed 2400 dpi scan at 8 bit is 3.48 GB, at 16 bit 7 GB) are written
//! as BigTIFF (version 43, 64-bit offsets, 20-byte directory entries).

use flate2::write::ZlibEncoder;
use std::io::{self, Seek, SeekFrom, Write};

/// Rows per strip; 64 rows of a 2400 dpi line is a 5.5 MB strip.
const ROWS_PER_STRIP: usize = 64;
const HEADER_LEN: u64 = 8;
const BIG_HEADER_LEN: u64 = 16;
const LIMIT: u64 = u32::MAX as u64;
/// Above this much uncompressed pixel data the file is written as BigTIFF
/// (64-bit offsets); compressed strips of a full 16-bit bed exceed 4 GiB too.
const BIG_THRESHOLD: u64 = 7 << 29; // 3.5 GiB: an 8-bit 2400 dpi bed (3.24 GiB) stays classic

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Compression {
    None,
    /// zlib stream per strip, horizontal differencing per row, level 1
    /// (measured on real scans: ~60% of raw at >100 MB/s; level 6 gains 2–3
    /// points at a third of the speed).
    Deflate,
}

pub struct Writer<W: Write + Seek> {
    out: W,
    /// BigTIFF layout (64-bit offsets) instead of classic.
    big: bool,
    width: usize,
    /// Samples per pixel: 3 (RGB) or 1 (gray).
    channels: usize,
    /// Bits per sample: 8 or 16 (16-bit rows are little-endian bytes).
    bits: usize,
    dpi: u32,
    compression: Compression,
    rows: usize,
    /// Rows of the strip being assembled (predictor already applied).
    strip: Vec<u8>,
    strip_rows: usize,
    offsets: Vec<u64>,
    counts: Vec<u64>,
    pos: u64,
}

/// Horizontal differencing (TIFF predictor 2) for 8-bit samples: each sample
/// minus the same channel of the pixel to its left.
fn predict(row: &mut [u8], channels: usize, bits: usize) {
    if bits == 16 {
        // Differencing on the 16-bit little-endian samples.
        let n = row.len() / 2;
        for i in (channels..n).rev() {
            let a = u16::from_le_bytes([row[2 * i], row[2 * i + 1]]);
            let b = u16::from_le_bytes([row[2 * (i - channels)], row[2 * (i - channels) + 1]]);
            row[2 * i..2 * i + 2].copy_from_slice(&a.wrapping_sub(b).to_le_bytes());
        }
    } else {
        for i in (channels..row.len()).rev() {
            row[i] = row[i].wrapping_sub(row[i - channels]);
        }
    }
}

impl<W: Write + Seek> Writer<W> {
    /// Start a `width` pixel wide image with at most `planned_rows` rows.
    /// Fails early if uncompressed pixel data could not fit a classic TIFF.
    pub fn new(
        out: W,
        width: usize,
        planned_rows: usize,
        dpi: u32,
        compression: Compression,
        channels: usize,
        bits: usize,
    ) -> io::Result<Self> {
        let data = width as u64 * channels as u64 * (bits as u64 / 8) * planned_rows as u64;
        let big = data > BIG_THRESHOLD;
        Self::start(out, big, width, dpi, compression, channels, bits)
    }

    /// Always write BigTIFF (tests).
    pub fn new_big(
        out: W,
        width: usize,
        dpi: u32,
        compression: Compression,
        channels: usize,
        bits: usize,
    ) -> io::Result<Self> {
        Self::start(out, true, width, dpi, compression, channels, bits)
    }

    fn start(
        mut out: W,
        big: bool,
        width: usize,
        dpi: u32,
        compression: Compression,
        channels: usize,
        bits: usize,
    ) -> io::Result<Self> {
        if big {
            // BigTIFF: version 43, offset size 8, reserved 0, u64 first-IFD offset.
            out.write_all(b"II\x2b\x00\x08\x00\x00\x00")?;
            out.write_all(&0u64.to_le_bytes())?; // patched by finish()
        } else {
            out.write_all(b"II\x2a\x00")?;
            out.write_all(&0u32.to_le_bytes())?; // IFD offset, patched by finish()
        }
        Ok(Writer {
            out,
            big,
            width,
            channels,
            bits,
            dpi,
            compression,
            rows: 0,
            strip: Vec::with_capacity(ROWS_PER_STRIP * width * 3),
            strip_rows: 0,
            offsets: Vec::new(),
            counts: Vec::new(),
            pos: if big { BIG_HEADER_LEN } else { HEADER_LEN },
        })
    }

    pub fn write_row(&mut self, row: &[u8]) -> io::Result<()> {
        debug_assert_eq!(row.len(), self.width * self.channels * self.bits / 8);
        let start = self.strip.len();
        self.strip.extend_from_slice(row);
        if self.compression == Compression::Deflate {
            predict(&mut self.strip[start..], self.channels, self.bits);
        }
        self.strip_rows += 1;
        self.rows += 1;
        if self.strip_rows == ROWS_PER_STRIP {
            self.flush_strip()?;
        }
        Ok(())
    }

    fn flush_strip(&mut self) -> io::Result<()> {
        if self.strip_rows == 0 {
            return Ok(());
        }
        let compressed;
        let data: &[u8] = match self.compression {
            Compression::None => &self.strip,
            Compression::Deflate => {
                let mut enc = ZlibEncoder::new(
                    Vec::with_capacity(self.strip.len() / 2),
                    flate2::Compression::fast(),
                );
                enc.write_all(&self.strip)?;
                compressed = enc.finish()?;
                &compressed
            }
        };
        if !self.big && self.pos + data.len() as u64 + 64 * 1024 > LIMIT {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "image exceeds the classic TIFF 4 GiB limit",
            ));
        }
        self.out.write_all(data)?;
        self.offsets.push(self.pos);
        self.counts.push(data.len() as u64);
        self.pos += data.len() as u64;
        self.strip.clear();
        self.strip_rows = 0;
        Ok(())
    }

    /// Write the directory for the rows received so far and patch the header.
    /// Returns the underlying writer.
    pub fn finish(mut self) -> io::Result<W> {
        self.flush_strip()?;
        let strips = self.offsets.len();
        let big = self.big;

        // Out-of-line values, each at a known offset after the pixel data.
        let mut pos = self.pos;
        if pos % 2 == 1 {
            self.out.write_all(&[0])?;
            pos += 1;
        }
        let mut blob = Vec::new();
        let mut place = |bytes: &[u8]| -> u64 {
            let at = pos + blob.len() as u64;
            blob.extend_from_slice(bytes);
            if blob.len() % 2 == 1 {
                blob.push(0);
            }
            at
        };
        const SHORT: u16 = 3;
        const LONG: u16 = 4;
        const RATIONAL: u16 = 5;
        const LONG8: u16 = 16;
        let b = self.bits as u8;
        // BitsPerSample ×3 (6 bytes) and a RATIONAL (8 bytes) fit a BigTIFF
        // entry inline; classic TIFF stores them out of line.
        let bps_at = if self.channels == 3 {
            if big {
                u64::from_le_bytes([b, 0, b, 0, b, 0, 0, 0])
            } else {
                place(&[b, 0, b, 0, b, 0])
            }
        } else {
            u64::from(b)
        };
        let res = [self.dpi.to_le_bytes(), 1u32.to_le_bytes()].concat();
        let (xres_at, yres_at) = if big {
            let v = u64::from_le_bytes(res.clone().try_into().unwrap());
            (v, v)
        } else {
            (place(&res), place(&res))
        };
        let array_kind = if big { LONG8 } else { LONG };
        let array_bytes = |v: &[u64]| -> Vec<u8> {
            if big {
                v.iter().flat_map(|x| x.to_le_bytes()).collect()
            } else {
                v.iter().flat_map(|x| (*x as u32).to_le_bytes()).collect()
            }
        };
        let offsets_at = if strips == 1 {
            self.offsets[0]
        } else {
            place(&array_bytes(&self.offsets))
        };
        let counts_at = if strips == 1 {
            self.counts[0]
        } else {
            place(&array_bytes(&self.counts))
        };
        let ifd_at = pos + blob.len() as u64;

        let compression_code = match self.compression {
            Compression::None => 1,
            Compression::Deflate => 8,
        };
        let mut entries: Vec<(u16, u16, u64, u64)> = vec![
            (256, LONG, 1, self.width as u64),
            (257, LONG, 1, self.rows as u64),
            (258, SHORT, self.channels as u64, bps_at),
            (259, SHORT, 1, compression_code),
            (262, SHORT, 1, if self.channels == 3 { 2 } else { 1 }), // RGB / BlackIsZero
            (273, array_kind, strips as u64, offsets_at),
            (277, SHORT, 1, self.channels as u64),
            (278, LONG, 1, ROWS_PER_STRIP as u64),
            (279, array_kind, strips as u64, counts_at),
            (282, RATIONAL, 1, xres_at),
            (283, RATIONAL, 1, yres_at),
            (284, SHORT, 1, 1), // chunky
            (296, SHORT, 1, 2), // inch
        ];
        if self.compression == Compression::Deflate {
            entries.push((317, SHORT, 1, 2)); // horizontal differencing
        }
        if big {
            blob.extend_from_slice(&(entries.len() as u64).to_le_bytes());
        } else {
            blob.extend_from_slice(&(entries.len() as u16).to_le_bytes());
        }
        for (tag, kind, count, value) in entries {
            blob.extend_from_slice(&tag.to_le_bytes());
            blob.extend_from_slice(&kind.to_le_bytes());
            if big {
                blob.extend_from_slice(&count.to_le_bytes());
                // Values that fit are stored inline, left-justified (little-endian).
                let mut field = [0u8; 8];
                if kind == SHORT && count == 1 {
                    field[..2].copy_from_slice(&(value as u16).to_le_bytes());
                } else {
                    field.copy_from_slice(&value.to_le_bytes());
                }
                blob.extend_from_slice(&field);
            } else {
                blob.extend_from_slice(&(count as u32).to_le_bytes());
                if kind == SHORT && count == 1 {
                    blob.extend_from_slice(&(value as u16).to_le_bytes());
                    blob.extend_from_slice(&[0, 0]);
                } else {
                    blob.extend_from_slice(&(value as u32).to_le_bytes());
                }
            }
        }
        if big {
            blob.extend_from_slice(&0u64.to_le_bytes()); // no next IFD
        } else {
            blob.extend_from_slice(&0u32.to_le_bytes());
        }
        self.out.write_all(&blob)?;
        if big {
            self.out.seek(SeekFrom::Start(8))?;
            self.out.write_all(&ifd_at.to_le_bytes())?;
        } else {
            self.out.seek(SeekFrom::Start(4))?;
            self.out.write_all(&(ifd_at as u32).to_le_bytes())?;
        }
        self.out.flush()?;
        Ok(self.out)
    }
}

/// Minimal reader for the TIFFs this crate writes (and other little-endian
/// classic TIFFs with 8-bit RGB chunky strips, uncompressed or Deflate with
/// the horizontal predictor).
pub struct Reader<R: Read + Seek> {
    inp: R,
    pub width: usize,
    /// Samples per pixel (3 or 1).
    pub channels: usize,
    /// Bits per sample (8 or 16).
    pub bits: usize,
    pub height: usize,
    pub rows_per_strip: usize,
    pub dpi: Option<u32>,
    offsets: Vec<u64>,
    counts: Vec<u64>,
    deflate: bool,
    predictor: bool,
}

use std::io::Read;

fn read_at<R: Read + Seek>(inp: &mut R, at: u64, n: usize) -> io::Result<Vec<u8>> {
    inp.seek(SeekFrom::Start(at))?;
    let mut b = vec![0; n];
    inp.read_exact(&mut b)?;
    Ok(b)
}
fn le16(b: &[u8]) -> u64 {
    u64::from(u16::from_le_bytes([b[0], b[1]]))
}
fn le64(b: &[u8]) -> u64 {
    u64::from_le_bytes(b[..8].try_into().unwrap())
}
fn le32(b: &[u8]) -> u64 {
    u64::from(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
}
fn bad(msg: &str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, msg.to_string())
}

impl<R: Read + Seek> Reader<R> {
    pub fn new(mut inp: R) -> io::Result<Self> {
        let h = read_at(&mut inp, 0, 16)?;
        let big = match (&h[..4], h[4]) {
            (b"II\x2a\x00", _) => false,
            (b"II\x2b\x00", 8) => true,
            _ => return Err(bad("not a little-endian TIFF or BigTIFF")),
        };
        let (ifd, n, entry_len) = if big {
            let ifd = le64(&h[8..]);
            (ifd, le64(&read_at(&mut inp, ifd, 8)?) as usize, 20usize)
        } else {
            let ifd = le32(&h[4..]);
            (ifd, le16(&read_at(&mut inp, ifd, 2)?) as usize, 12usize)
        };
        let entries = read_at(&mut inp, ifd + if big { 8 } else { 2 }, entry_len * n)?;
        // tag -> (type, count, inline value or offset)
        let mut tags: std::collections::HashMap<u16, (u64, u64, u64)> =
            std::collections::HashMap::new();
        for e in entries.chunks(entry_len) {
            let kind = le16(&e[2..]);
            let (count, inline) = if big {
                let count = le64(&e[4..]);
                (
                    count,
                    if kind == 3 && count == 1 {
                        le16(&e[12..])
                    } else {
                        le64(&e[12..])
                    },
                )
            } else {
                let count = le32(&e[4..]);
                (
                    count,
                    if kind == 3 && count == 1 {
                        le16(&e[8..])
                    } else {
                        le32(&e[8..])
                    },
                )
            };
            tags.insert(le16(e) as u16, (kind, count, inline));
        }
        let get = |t: u16| tags.get(&t).map(|v| v.2);
        let width = get(256).ok_or_else(|| bad("no width"))? as usize;
        let height = get(257).ok_or_else(|| bad("no height"))? as usize;
        let compression = get(259).unwrap_or(1);
        if compression != 1 && compression != 8 {
            return Err(bad("unsupported compression (only none and Deflate)"));
        }
        let channels = get(277).unwrap_or(1) as usize;
        // BitsPerSample: inline for one sample; for three the inline value is an
        // offset, so read the first sample's width from there.
        let bits = match tags.get(&258) {
            Some(&(_, 1, v)) => v as usize,
            // Three SHORTs: inline in BigTIFF (6 of 8 bytes), out of line in classic.
            Some(&(_, _, v)) if big => (v & 0xffff) as usize,
            Some(&(_, _, at)) => le16(&read_at(&mut inp, at, 2)?) as usize,
            None => 8,
        };
        if bits != 8 && bits != 16 {
            return Err(bad("only 8- and 16-bit samples"));
        }
        if (channels != 3 && channels != 1) || get(284).unwrap_or(1) != 1 {
            return Err(bad("not chunky 8-bit RGB or gray"));
        }
        let predictor = get(317) == Some(2);
        let rows_per_strip = get(278).unwrap_or(height as u64).max(1) as usize;
        let mut array = |t: u16| -> io::Result<Vec<u64>> {
            let (kind, count, val) = *tags.get(&t).ok_or_else(|| bad("missing strip tag"))?;
            if count == 1 {
                return Ok(vec![val]);
            }
            let size = match kind {
                3 => 2,
                16 => 8,
                _ => 4,
            };
            let b = read_at(&mut inp, val, size * count as usize)?;
            Ok(b.chunks(size)
                .map(|c| match size {
                    2 => le16(c),
                    8 => le64(c),
                    _ => le32(c),
                })
                .collect())
        };
        let offsets = array(273)?;
        let counts = array(279)?;
        let dpi = match tags.get(&282) {
            Some(&(5, 1, v)) if big => {
                let (num, den) = (v & 0xffff_ffff, v >> 32);
                num.checked_div(den).map(|v| v as u32)
            }
            Some(&(5, 1, at)) => {
                let b = read_at(&mut inp, at, 8)?;
                let (num, den) = (le32(&b), le32(&b[4..]));
                num.checked_div(den).map(|v| v as u32)
            }
            _ => None,
        };
        Ok(Reader {
            inp,
            width,
            channels,
            bits,
            height,
            rows_per_strip,
            dpi,
            offsets,
            counts,
            deflate: compression == 8,
            predictor,
        })
    }

    pub fn strips(&self) -> usize {
        self.offsets.len()
    }

    /// Rows in strip `i`.
    pub fn rows_in_strip(&self, i: usize) -> usize {
        (self.height - i * self.rows_per_strip).min(self.rows_per_strip)
    }

    /// Decoded RGB rows of strip `i`, `rows_in_strip(i) * width * 3` bytes.
    pub fn strip(&mut self, i: usize) -> io::Result<Vec<u8>> {
        let raw = read_at(&mut self.inp, self.offsets[i], self.counts[i] as usize)?;
        let mut data = if self.deflate {
            let mut out = Vec::with_capacity(self.rows_per_strip * self.width * 3);
            flate2::read::ZlibDecoder::new(&raw[..]).read_to_end(&mut out)?;
            out
        } else {
            raw
        };
        if self.predictor {
            let ch = self.channels;
            let bpp = ch * self.bits / 8;
            for row in data.chunks_mut(self.width * bpp) {
                if self.bits == 16 {
                    for i in ch..row.len() / 2 {
                        let a = u16::from_le_bytes([row[2 * i], row[2 * i + 1]]);
                        let b = u16::from_le_bytes([row[2 * (i - ch)], row[2 * (i - ch) + 1]]);
                        row[2 * i..2 * i + 2].copy_from_slice(&a.wrapping_add(b).to_le_bytes());
                    }
                } else {
                    for x in ch..row.len() {
                        row[x] = row[x].wrapping_add(row[x - ch]);
                    }
                }
            }
        }
        if data.len() != self.rows_in_strip(i) * self.width * self.channels * self.bits / 8 {
            return Err(bad("strip has the wrong length"));
        }
        Ok(data)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use flate2::read::ZlibDecoder;
    use std::io::{Cursor, Read};

    fn u16_at(b: &[u8], i: usize) -> u16 {
        u16::from_le_bytes([b[i], b[i + 1]])
    }
    fn u32_at(b: &[u8], i: usize) -> u32 {
        u32::from_le_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]])
    }
    /// (tag, type, count, value-or-offset) entries of the single IFD.
    fn ifd(b: &[u8]) -> Vec<(u16, u16, u32, u32)> {
        assert_eq!(&b[..4], b"II\x2a\x00");
        let at = u32_at(b, 4) as usize;
        let n = u16_at(b, at) as usize;
        (0..n)
            .map(|i| {
                let e = at + 2 + 12 * i;
                let kind = u16_at(b, e + 2);
                let count = u32_at(b, e + 4);
                let v = if kind == 3 && count == 1 {
                    u32::from(u16_at(b, e + 8))
                } else {
                    u32_at(b, e + 8)
                };
                (u16_at(b, e), kind, count, v)
            })
            .collect()
    }
    fn entry(d: &[(u16, u16, u32, u32)], tag: u16) -> (u16, u16, u32, u32) {
        *d.iter().find(|e| e.0 == tag).unwrap()
    }
    fn strips(b: &[u8]) -> Vec<(usize, usize)> {
        let d = ifd(b);
        let (o, c) = (entry(&d, 273), entry(&d, 279));
        if o.2 == 1 {
            return vec![(o.3 as usize, c.3 as usize)];
        }
        (0..o.2 as usize)
            .map(|i| {
                (
                    u32_at(b, o.3 as usize + 4 * i) as usize,
                    u32_at(b, c.3 as usize + 4 * i) as usize,
                )
            })
            .collect()
    }

    #[test]
    fn single_strip_image_has_inline_strip_tags() {
        let mut w =
            Writer::new(Cursor::new(Vec::new()), 2, 2, 300, Compression::None, 3, 8).unwrap();
        w.write_row(&[1, 2, 3, 4, 5, 6]).unwrap();
        w.write_row(&[7, 8, 9, 10, 11, 12]).unwrap();
        let b = w.finish().unwrap().into_inner();
        assert_eq!(&b[8..20], &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]);
        let d = ifd(&b);
        assert_eq!(d.len(), 13);
        assert!(d.windows(2).all(|p| p[0].0 < p[1].0), "tags sorted");
        assert!(d.contains(&(256, 4, 1, 2)));
        assert!(d.contains(&(257, 4, 1, 2)));
        assert!(d.contains(&(259, 3, 1, 1)));
        assert!(d.contains(&(273, 4, 1, 8)), "{d:?}");
        assert!(d.contains(&(279, 4, 1, 12)));
        assert!(d.contains(&(277, 3, 1, 3)));
        assert!(!d.iter().any(|e| e.0 == 317));
        let bps = entry(&d, 258).3 as usize;
        assert_eq!(&b[bps..bps + 6], &[8, 0, 8, 0, 8, 0]);
        let xres = entry(&d, 282).3 as usize;
        assert_eq!((u32_at(&b, xres), u32_at(&b, xres + 4)), (300, 1));
    }

    #[test]
    fn directory_describes_the_rows_actually_written() {
        let mut w = Writer::new(
            Cursor::new(Vec::new()),
            2,
            200,
            2400,
            Compression::None,
            3,
            8,
        )
        .unwrap();
        for i in 0..70u8 {
            w.write_row(&[i; 6]).unwrap();
        }
        let b = w.finish().unwrap().into_inner();
        let d = ifd(&b);
        assert!(d.contains(&(257, 4, 1, 70)));
        assert!(d.contains(&(278, 4, 1, 64)));
        assert_eq!(strips(&b), vec![(8, 64 * 6), (8 + 64 * 6, 6 * 6)]);
        assert_eq!(b[8 + 69 * 6], 69);
    }

    #[test]
    fn deflate_strips_decode_to_the_rows_with_the_predictor_undone() {
        let width = 5;
        let rows: Vec<Vec<u8>> = (0..70u32)
            .map(|r| {
                (0..width * 3)
                    .map(|i| ((i as u32 * 37 + r * 11) % 256) as u8)
                    .collect()
            })
            .collect();
        let mut w = Writer::new(
            Cursor::new(Vec::new()),
            width,
            70,
            600,
            Compression::Deflate,
            3,
            8,
        )
        .unwrap();
        for r in &rows {
            w.write_row(r).unwrap();
        }
        let b = w.finish().unwrap().into_inner();
        let d = ifd(&b);
        assert_eq!(d.len(), 14);
        assert!(d.contains(&(259, 3, 1, 8)));
        assert!(d.contains(&(317, 3, 1, 2)));
        assert!(d.contains(&(257, 4, 1, 70)));
        let s = strips(&b);
        assert_eq!(s.len(), 2);
        let mut decoded = Vec::new();
        for (off, len) in s {
            ZlibDecoder::new(&b[off..off + len])
                .read_to_end(&mut decoded)
                .unwrap();
        }
        assert_eq!(decoded.len(), 70 * width * 3);
        for (r, row) in decoded.chunks_mut(width * 3).enumerate() {
            for i in 3..row.len() {
                row[i] = row[i].wrapping_add(row[i - 3]);
            }
            assert_eq!(row, &rows[r][..], "row {r}");
        }
    }

    #[test]
    fn classic_tiff_is_kept_below_the_bigtiff_threshold() {
        // 8-bit full bed at 2400 dpi (3.48 GB) stays classic; twice that goes BigTIFF.
        assert!(
            !Writer::new(
                Cursor::new(Vec::new()),
                28751,
                40320,
                2400,
                Compression::None,
                3,
                8
            )
            .unwrap()
            .big
        );
        assert!(
            Writer::new(
                Cursor::new(Vec::new()),
                28751,
                40320 * 2,
                2400,
                Compression::None,
                3,
                8
            )
            .unwrap()
            .big
        );
    }

    #[test]
    fn reader_round_trips_both_compressions() {
        for compression in [Compression::None, Compression::Deflate] {
            let width = 7;
            let rows: Vec<Vec<u8>> = (0..150u32)
                .map(|r| {
                    (0..width * 3)
                        .map(|i| ((i as u32 * 31 + r * 7) % 256) as u8)
                        .collect()
                })
                .collect();
            let mut w =
                Writer::new(Cursor::new(Vec::new()), width, 150, 1200, compression, 3, 8).unwrap();
            for r in &rows {
                w.write_row(r).unwrap();
            }
            let b = w.finish().unwrap().into_inner();
            let mut rd = Reader::new(Cursor::new(b)).unwrap();
            assert_eq!(
                (rd.width, rd.height, rd.rows_per_strip, rd.dpi, rd.strips()),
                (7, 150, 64, Some(1200), 3)
            );
            assert_eq!(rd.rows_in_strip(2), 22);
            let mut all = Vec::new();
            for i in 0..rd.strips() {
                all.extend(rd.strip(i).unwrap());
            }
            assert_eq!(all, rows.concat());
        }
    }

    #[test]
    fn bigtiff_round_trips_through_the_reader() {
        let width = 5;
        let rows: Vec<Vec<u8>> = (0..150u32)
            .map(|r| {
                (0..width * 3)
                    .map(|i| ((i as u32 * 29 + r * 13) % 256) as u8)
                    .collect()
            })
            .collect();
        let mut w = Writer::new_big(
            Cursor::new(Vec::new()),
            width,
            2400,
            Compression::Deflate,
            3,
            8,
        )
        .unwrap();
        for r in &rows {
            w.write_row(r).unwrap();
        }
        let b = w.finish().unwrap().into_inner();
        assert_eq!(&b[..4], b"II\x2b\x00");
        let mut rd = Reader::new(Cursor::new(b)).unwrap();
        assert_eq!(
            (
                rd.width,
                rd.height,
                rd.channels,
                rd.bits,
                rd.dpi,
                rd.strips()
            ),
            (5, 150, 3, 8, Some(2400), 3)
        );
        let mut all = Vec::new();
        for i in 0..rd.strips() {
            all.extend(rd.strip(i).unwrap());
        }
        assert_eq!(all, rows.concat());
    }

    #[test]
    fn large_images_switch_to_bigtiff() {
        // 16-bit full bed at 2400 dpi: 7 GB of pixels → BigTIFF header.
        let w = Writer::new(
            Cursor::new(Vec::new()),
            28805,
            40320,
            2400,
            Compression::Deflate,
            3,
            16,
        )
        .unwrap();
        assert!(w.big);
        let w = Writer::new(
            Cursor::new(Vec::new()),
            3599,
            5040,
            300,
            Compression::None,
            3,
            8,
        )
        .unwrap();
        assert!(!w.big);
    }
}
