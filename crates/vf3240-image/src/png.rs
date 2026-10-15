//! Streaming 8-bit RGB PNG writer (Sub-filtered rows, zlib via flate2).
//!
//! Rows are compressed as they arrive and emitted as IDAT chunks of about
//! 1 MiB; if fewer rows than planned are written, IHDR is patched at the end.

use flate2::write::ZlibEncoder;
use flate2::{Compression, Crc};
use std::io::{self, Seek, SeekFrom, Write};

const IDAT_TARGET: usize = 1 << 20;

fn chunk<W: Write>(out: &mut W, kind: &[u8; 4], data: &[u8]) -> io::Result<()> {
    out.write_all(&(data.len() as u32).to_be_bytes())?;
    out.write_all(kind)?;
    out.write_all(data)?;
    let mut crc = Crc::new();
    crc.update(kind);
    crc.update(data);
    out.write_all(&crc.sum().to_be_bytes())
}

fn ihdr(width: usize, height: usize, channels: usize, bits: usize) -> [u8; 13] {
    let mut h = [0u8; 13];
    h[..4].copy_from_slice(&(width as u32).to_be_bytes());
    h[4..8].copy_from_slice(&(height as u32).to_be_bytes());
    // Colour type 2 = RGB, 0 = gray.
    // Colour type 2 = RGB, 0 = gray; 16-bit samples are big-endian.
    h[8..].copy_from_slice(&[bits as u8, if channels == 3 { 2 } else { 0 }, 0, 0, 0]);
    h
}

/// Collects compressed bytes and flushes them as IDAT chunks.
struct IdatSink<W: Write> {
    out: W,
    buf: Vec<u8>,
}

impl<W: Write> Write for IdatSink<W> {
    fn write(&mut self, data: &[u8]) -> io::Result<usize> {
        self.buf.extend_from_slice(data);
        if self.buf.len() >= IDAT_TARGET {
            chunk(&mut self.out, b"IDAT", &self.buf)?;
            self.buf.clear();
        }
        Ok(data.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        self.out.flush()
    }
}

pub struct Writer<W: Write + Seek> {
    enc: ZlibEncoder<IdatSink<W>>,
    width: usize,
    planned_rows: usize,
    rows: usize,
    filtered: Vec<u8>,
    channels: usize,
    /// Bytes per pixel, the Sub filter's distance.
    bpp: usize,
}

impl<W: Write + Seek> Writer<W> {
    pub fn new(
        mut out: W,
        width: usize,
        planned_rows: usize,
        dpi: u32,
        channels: usize,
        bits: usize,
    ) -> io::Result<Self> {
        out.write_all(b"\x89PNG\r\n\x1a\n")?;
        chunk(
            &mut out,
            b"IHDR",
            &ihdr(width, planned_rows, channels, bits),
        )?;
        let per_metre = (f64::from(dpi) / 0.0254).round() as u32;
        let mut phys = [0u8; 9];
        phys[..4].copy_from_slice(&per_metre.to_be_bytes());
        phys[4..8].copy_from_slice(&per_metre.to_be_bytes());
        phys[8] = 1;
        chunk(&mut out, b"pHYs", &phys)?;
        let enc = ZlibEncoder::new(
            IdatSink {
                out,
                buf: Vec::with_capacity(IDAT_TARGET + 4096),
            },
            Compression::fast(),
        );
        Ok(Writer {
            enc,
            width,
            planned_rows,
            rows: 0,
            filtered: Vec::with_capacity(width * channels * bits / 8 + 1),
            channels,
            bpp: channels * bits / 8,
        })
    }

    /// Append one row with the Sub filter (each byte minus the byte one pixel
    /// to the left), which buys ~25 % over filter 0 on scans at little cost.
    pub fn write_row(&mut self, row: &[u8]) -> io::Result<()> {
        debug_assert_eq!(row.len(), self.width * self.bpp);
        self.filtered.clear();
        self.filtered.push(1);
        self.filtered
            .extend_from_slice(&row[..row.len().min(self.channels)]);
        self.filtered.extend(
            row.iter()
                .skip(self.bpp)
                .zip(row.iter())
                .map(|(&b, &left)| b.wrapping_sub(left)),
        );
        self.enc.write_all(&self.filtered)?;
        self.rows += 1;
        Ok(())
    }

    /// Flush the stream and IEND; patch IHDR if fewer rows than planned arrived.
    /// Returns the underlying writer.
    pub fn finish(self) -> io::Result<W> {
        let IdatSink { mut out, buf } = self.enc.finish()?;
        if !buf.is_empty() {
            chunk(&mut out, b"IDAT", &buf)?;
        }
        chunk(&mut out, b"IEND", &[])?;
        if self.rows != self.planned_rows {
            out.seek(SeekFrom::Start(8))?;
            chunk(
                &mut out,
                b"IHDR",
                &ihdr(
                    self.width,
                    self.rows,
                    self.channels,
                    8 * self.bpp / self.channels,
                ),
            )?;
        }
        out.flush()?;
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    fn height_of(buf: &[u8]) -> u32 {
        u32::from_be_bytes([buf[20], buf[21], buf[22], buf[23]])
    }

    #[test]
    fn header_and_chunks_are_well_formed() {
        let mut w = Writer::new(Cursor::new(Vec::new()), 2, 1, 300, 3, 8).unwrap();
        w.write_row(&[255, 0, 0, 0, 255, 0]).unwrap();
        let buf = w.finish().unwrap().into_inner();
        assert_eq!(&buf[..8], b"\x89PNG\r\n\x1a\n");
        assert_eq!(&buf[12..16], b"IHDR");
        assert_eq!(&buf[16..20], &2u32.to_be_bytes());
        assert_eq!(height_of(&buf), 1);
        assert_eq!(&buf[37..41], b"pHYs");
        assert_eq!(&buf[buf.len() - 8..buf.len() - 4], b"IEND");
    }

    #[test]
    fn short_image_patches_the_height() {
        let mut w = Writer::new(Cursor::new(Vec::new()), 1, 10, 300, 3, 8).unwrap();
        w.write_row(&[1, 2, 3]).unwrap();
        w.write_row(&[4, 5, 6]).unwrap();
        let buf = w.finish().unwrap().into_inner();
        assert_eq!(height_of(&buf), 2);
        // CRC of the patched IHDR is consistent with its data.
        let mut crc = Crc::new();
        crc.update(&buf[12..29]);
        assert_eq!(&buf[29..33], &crc.sum().to_be_bytes());
    }

    #[test]
    fn rows_are_sub_filtered() {
        use flate2::read::ZlibDecoder;
        use std::io::Read;
        let mut w = Writer::new(Cursor::new(Vec::new()), 2, 1, 300, 3, 8).unwrap();
        w.write_row(&[10, 20, 30, 15, 18, 40]).unwrap();
        let buf = w.finish().unwrap().into_inner();
        // IDAT follows IHDR (8+25) and pHYs (21).
        let at = 8 + 25 + 21;
        assert_eq!(&buf[at + 4..at + 8], b"IDAT");
        let len = u32::from_be_bytes([buf[at], buf[at + 1], buf[at + 2], buf[at + 3]]) as usize;
        let mut raw = Vec::new();
        ZlibDecoder::new(&buf[at + 8..at + 8 + len])
            .read_to_end(&mut raw)
            .unwrap();
        assert_eq!(raw, vec![1, 10, 20, 30, 5, 254, 10]);
    }

    #[test]
    fn large_images_emit_several_idat_chunks() {
        let row = vec![0x5au8; 3000];
        let mut w = Writer::new(Cursor::new(Vec::new()), 1000, 2000, 300, 3, 8).unwrap();
        for _ in 0..2000 {
            w.write_row(&row).unwrap();
        }
        let buf = w.finish().unwrap().into_inner();
        let idats = buf.windows(4).filter(|w| w == b"IDAT").count();
        assert!(idats >= 1);
        assert_eq!(&buf[buf.len() - 8..buf.len() - 4], b"IEND");
    }
}
