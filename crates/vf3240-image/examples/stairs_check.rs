//! Run the live staircase stage over a raw scan and optionally write the
//! result: `stairs_check <in.tif> [out.tif] [--compress]`. Prints the
//! stage's measurement summary.

use std::fs::File;
use std::io::{BufReader, BufWriter};
use vf3240_image::stairs::Destair;
use vf3240_image::tiff::{Compression, Reader, Writer};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    let compress = args.iter().any(|a| a == "--compress");
    args.retain(|a| a != "--compress");
    let path = args
        .first()
        .cloned()
        .ok_or("usage: stairs_check <in.tif> [out.tif] [--compress]")?;
    let mut t = Reader::new(BufReader::with_capacity(8 << 20, File::open(&path)?))?;
    let (w, h) = (t.width, t.height);
    let mut writer = match args.get(1) {
        Some(out) => Some(Writer::new(
            BufWriter::with_capacity(4 << 20, File::create(out)?),
            w,
            h,
            t.dpi.unwrap_or(2400),
            if compress {
                Compression::Deflate
            } else {
                Compression::None
            },
            t.channels,
            t.bits,
        )?),
        None => None,
    };
    let period = match t.dpi.unwrap_or(2400) {
        1200 => 2,
        _ => 4,
    };
    let mut st = Destair::new(
        w,
        period,
        (0.417, 0.718),
        if t.bits == 16 { 65535 } else { 255 },
    );
    let mut emitted = 0usize;
    let bits = t.bits;
    let mut bytes = Vec::new();
    let mut emit = |r: &[u16]| -> std::io::Result<()> {
        emitted += 1;
        match writer.as_mut() {
            Some(wr) => {
                bytes.clear();
                if bits == 16 {
                    bytes.extend(r.iter().flat_map(|v| v.to_le_bytes()));
                } else {
                    bytes.extend(r.iter().map(|&v| v as u8));
                }
                wr.write_row(&bytes)
            }
            None => Ok(()),
        }
    };
    let t0 = std::time::Instant::now();
    let row_bytes = w * t.channels * bits / 8;
    for s in 0..t.strips() {
        let data = t.strip(s)?;
        for row in data.chunks(row_bytes) {
            // Samples as u16, three channels (gray rows replicated).
            let samples: Vec<u16> = if bits == 16 {
                row.chunks(2)
                    .map(|p| u16::from_le_bytes([p[0], p[1]]))
                    .collect()
            } else {
                row.iter().map(|&v| u16::from(v)).collect()
            };
            let rgb: Vec<u16> = if t.channels == 1 {
                samples.iter().flat_map(|&v| [v; 3]).collect()
            } else {
                samples
            };
            st.push(rgb, &mut emit)?;
        }
    }
    st.finish(&mut emit)?;
    if let Some(wr) = writer {
        wr.finish()?;
    }
    println!(
        "{path}: {h} rows in, {emitted} out, {:.1?}; {}",
        t0.elapsed(),
        st.summary()
    );
    Ok(())
}
