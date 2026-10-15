//! Streaming binary PPM (P6) writer. No dependencies; Preview on macOS opens it.
//!
//! The height in the header is space-padded to a fixed width so a short scan
//! can patch it in place.

use std::io::{self, Seek, SeekFrom, Write};

pub struct Writer<W: Write + Seek> {
    out: W,
    width: usize,
    planned_rows: usize,
    rows: usize,
    channels: usize,
    bits: usize,
}

/// P6 for RGB, P5 for gray.
fn header(width: usize, height: usize, channels: usize, bits: usize) -> String {
    format!(
        "P{}\n{width} {height:>10}\n{}\n",
        if channels == 3 { 6 } else { 5 },
        if bits == 16 { 65535 } else { 255 }
    )
}

impl<W: Write + Seek> Writer<W> {
    pub fn new(
        mut out: W,
        width: usize,
        planned_rows: usize,
        channels: usize,
        bits: usize,
    ) -> io::Result<Self> {
        out.write_all(header(width, planned_rows, channels, bits).as_bytes())?;
        Ok(Writer {
            out,
            width,
            planned_rows,
            rows: 0,
            channels,
            bits,
        })
    }

    pub fn write_row(&mut self, row: &[u8]) -> io::Result<()> {
        debug_assert_eq!(row.len(), self.width * self.channels * self.bits / 8);
        self.out.write_all(row)?;
        self.rows += 1;
        Ok(())
    }

    /// Patch the height if fewer rows than planned arrived. Returns the writer.
    pub fn finish(mut self) -> io::Result<W> {
        if self.rows != self.planned_rows {
            self.out.seek(SeekFrom::Start(0))?;
            self.out
                .write_all(header(self.width, self.rows, self.channels, self.bits).as_bytes())?;
        }
        self.out.flush()?;
        Ok(self.out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    #[test]
    fn header_then_raw_bytes() {
        let mut w = Writer::new(Cursor::new(Vec::new()), 2, 1, 3, 8).unwrap();
        w.write_row(&[1, 2, 3, 4, 5, 6]).unwrap();
        let buf = w.finish().unwrap().into_inner();
        assert_eq!(buf, b"P6\n2          1\n255\n\x01\x02\x03\x04\x05\x06");
    }

    #[test]
    fn short_scan_patches_the_height_in_place() {
        let mut w = Writer::new(Cursor::new(Vec::new()), 1, 100, 3, 8).unwrap();
        w.write_row(&[9, 9, 9]).unwrap();
        let buf = w.finish().unwrap().into_inner();
        assert_eq!(buf, b"P6\n1          1\n255\n\x09\x09\x09");
    }
}
