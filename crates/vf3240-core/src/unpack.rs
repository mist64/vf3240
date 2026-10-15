//! Host-side unpacking of a raw CIS line into RGB pixels.
//!
//! Pure functions, no I/O. From `docs/protocol/07-readout.md` §3–4, in
//! particular the net mapping in §4.6: a raw 8-bit colour line is three
//! planes (B, R, G) of `W = 3S` samples; within a plane the three sensor
//! segments are interleaved sample by sample and stored reversed; the host
//! then rotates segment 0 to the end with a 2-pixel overlap and crops.

/// Per-resolution sensor geometry. From `MyScanner_SetParameter2`
/// (`07-readout.md` §4.3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Geometry {
    /// Pixels per CIS segment, `S`.
    pub seg_pixels: usize,
    /// `dwStartPixelOffset`: added to the requested X origin before cropping.
    pub start_pixel_offset: usize,
}

impl Geometry {
    /// Geometry for a supported resolution bucket (200/300/600/1200/2400).
    pub fn for_dpi(dpi: u32) -> Option<Geometry> {
        let (seg_pixels, start_pixel_offset) = match dpi {
            200..=299 => (864, 170),
            300..=599 => (1296, 256),
            600..=1199 => (2592, 511),
            1200..=2399 => (5184, 1021),
            2400 => (10368, 2040),
            _ => return None,
        };
        Some(Geometry {
            seg_pixels,
            start_pixel_offset,
        })
    }

    /// Plane width `W = 3S`.
    pub fn line_pixels(&self) -> usize {
        3 * self.seg_pixels
    }

    /// Bytes in one raw 8-bit colour line: three planes of `W`.
    pub fn raw_line_bytes_color8(&self) -> usize {
        3 * self.line_pixels()
    }
}

/// Plane index of each output channel in a raw colour line (R, G, B).
/// From `ScanCtrl_Pack_3CH_CIS_Color`: plane 0 = B, plane 1 = R, plane 2 = G.
const PLANE_OF_RGB: [usize; 3] = [1, 2, 0];

/// Overlap, in pixels, by which segment 0 is advanced during rotation
/// (the literal `+6` bytes at 3 bytes/pixel in `Scanner_ReadScanData`).
/// In 16-bit modes the same byte constant is one pixel (07 §4.2, §7).
const ROTATE_OVERLAP_PX: usize = 2;
const ROTATE_OVERLAP_PX_16: usize = 1;

/// Within-plane sample index of segment `s`, pixel `k`: `σ(s,k) = W − 1 − (3k + s)`.
fn sigma(w: usize, s: usize, k: usize) -> usize {
    w - 1 - (3 * k + s)
}

/// Segment and pixel of *rotated* pixel `x` (`0 ≤ x < 3S`), per §4.6.
fn rotated_source(seg: usize, x: usize) -> (usize, usize) {
    rotated_source_with(seg, x, ROTATE_OVERLAP_PX)
}

fn rotated_source_with(seg: usize, x: usize, overlap: usize) -> (usize, usize) {
    if x < 2 * seg {
        (1 + x / seg, x % seg)
    } else if x < 3 * seg - overlap {
        (0, x - 2 * seg + overlap)
    } else {
        (1, x - (3 * seg - overlap))
    }
}

/// Little-endian u16 sample `i` of `raw`.
fn u16_at(raw: &[u8], i: usize) -> u16 {
    u16::from_le_bytes([raw[2 * i], raw[2 * i + 1]])
}

/// Unpack one raw 16-bit colour line (`07-readout.md` §7): three planes of
/// `W` u16 LE samples, same order as 8-bit, segment-0 overlap of one pixel.
/// Appends `width * 3` samples (R, G, B) to `out`.
pub fn unpack_color16(
    geom: &Geometry,
    raw: &[u8],
    x0: usize,
    width: usize,
    out: &mut Vec<u16>,
) -> Option<()> {
    let w = geom.line_pixels();
    if raw.len() != 6 * w || x0 + width > w {
        return None;
    }
    out.reserve(width * 3);
    for x in x0..x0 + width {
        let (s, k) = rotated_source_with(geom.seg_pixels, x, ROTATE_OVERLAP_PX_16);
        let idx = sigma(w, s, k);
        for plane in PLANE_OF_RGB {
            out.push(u16_at(raw, plane * w + idx));
        }
    }
    Some(())
}

/// Unpack one raw 16-bit gray line: one plane of `W` u16 LE samples.
pub fn unpack_gray16(
    geom: &Geometry,
    raw: &[u8],
    x0: usize,
    width: usize,
    out: &mut Vec<u16>,
) -> Option<()> {
    let w = geom.line_pixels();
    if raw.len() != 2 * w || x0 + width > w {
        return None;
    }
    out.reserve(width);
    for x in x0..x0 + width {
        let (s, k) = rotated_source_with(geom.seg_pixels, x, ROTATE_OVERLAP_PX_16);
        out.push(u16_at(raw, sigma(w, s, k)));
    }
    Some(())
}

/// Unpack one raw 8-bit colour line into `width` RGB pixels starting at
/// rotated pixel `x0`, appending `width * 3` bytes to `out`.
///
/// `raw` must be exactly `3W` bytes. Returns `None` (and appends nothing)
/// if `raw` has the wrong length or `x0 + width > W`; the vendor checks
/// neither.
pub fn unpack_color8(
    geom: &Geometry,
    raw: &[u8],
    x0: usize,
    width: usize,
    out: &mut Vec<u8>,
) -> Option<()> {
    let w = geom.line_pixels();
    if raw.len() != 3 * w || x0 + width > w {
        return None;
    }
    out.reserve(width * 3);
    for x in x0..x0 + width {
        let (s, k) = rotated_source(geom.seg_pixels, x);
        let idx = sigma(w, s, k);
        for plane in PLANE_OF_RGB {
            out.push(raw[plane * w + idx]);
        }
    }
    Some(())
}

/// Unpack one raw 8-bit gray line (`10-gray-mode.md` §3): a single plane of
/// `W` samples in the colour plane's order, same rotation and crop.
/// Appends `width` bytes to `out`; `None` if the lengths do not fit.
pub fn unpack_gray8(
    geom: &Geometry,
    raw: &[u8],
    x0: usize,
    width: usize,
    out: &mut Vec<u8>,
) -> Option<()> {
    let w = geom.line_pixels();
    if raw.len() != w || x0 + width > w {
        return None;
    }
    out.reserve(width);
    for x in x0..x0 + width {
        let (s, k) = rotated_source(geom.seg_pixels, x);
        out.push(raw[sigma(w, s, k)]);
    }
    Some(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const G300: Geometry = Geometry {
        seg_pixels: 1296,
        start_pixel_offset: 256,
    };

    /// A raw line where every byte encodes its own index, so the output
    /// reveals exactly which raw byte each channel came from.
    fn tagged_raw(w: usize) -> Vec<u16> {
        (0..3 * w as u16).collect()
    }

    fn unpack_tagged(geom: &Geometry, x0: usize, width: usize) -> Vec<u16> {
        // Run the 8-bit unpacker twice (low and high byte of the tag) so the
        // test can see 16-bit indices through an 8-bit API.
        let tags = tagged_raw(geom.line_pixels());
        let lo: Vec<u8> = tags.iter().map(|&t| t as u8).collect();
        let hi: Vec<u8> = tags.iter().map(|&t| (t >> 8) as u8).collect();
        let (mut out_lo, mut out_hi) = (Vec::new(), Vec::new());
        unpack_color8(geom, &lo, x0, width, &mut out_lo).unwrap();
        unpack_color8(geom, &hi, x0, width, &mut out_hi).unwrap();
        out_lo
            .iter()
            .zip(&out_hi)
            .map(|(&l, &h)| u16::from(l) | (u16::from(h) << 8))
            .collect()
    }

    #[test]
    fn geometry_table_matches_the_vendor() {
        assert_eq!(Geometry::for_dpi(300), Some(G300));
        assert_eq!(G300.line_pixels(), 3888);
        assert_eq!(G300.raw_line_bytes_color8(), 11664);
        assert_eq!(Geometry::for_dpi(2400).unwrap().seg_pixels, 10368);
        assert_eq!(Geometry::for_dpi(100), None);
    }

    #[test]
    fn worked_example_from_the_readout_doc_at_300dpi() {
        // (rotated x, B raw byte, R raw byte, G raw byte) from 07-readout.md §4.6.
        let table = [
            (0, 3886, 7774, 11662),
            (1, 3883, 7771, 11659),
            (1295, 1, 3889, 7777),
            (1296, 3885, 7773, 11661),
            (2591, 0, 3888, 7776),
            (2592, 3881, 7769, 11657),
            (3885, 2, 3890, 7778),
            (3886, 3886, 7774, 11662),
            (3887, 3883, 7771, 11659),
        ];
        let out = unpack_tagged(&G300, 0, 3888);
        for (x, b, r, g) in table {
            assert_eq!(&out[3 * x..3 * x + 3], &[r, g, b], "rotated pixel {x}");
        }
    }

    #[test]
    fn crop_offsets_into_the_rotated_line() {
        let out = unpack_tagged(&G300, 256, 1);
        assert_eq!(out, vec![7006, 10894, 3118]);
    }

    #[test]
    fn every_sample_used_once_except_the_documented_overlap() {
        let out = unpack_tagged(&G300, 0, 3888);
        let w = 3888u16;
        let mut count = vec![0u8; 3 * w as usize];
        for t in out {
            count[t as usize] += 1;
        }
        for plane in 0..3u16 {
            let base = plane * w;
            for s in 0..w {
                let expected = match w - 1 - s {
                    0 | 3 => 0, // σ = W−1, W−4: segment 0 pixels 0 and 1 are dropped
                    1 | 4 => 2, // σ = W−2, W−5: segment 1 pixels 0 and 1 appear twice
                    _ => 1,
                };
                assert_eq!(
                    count[(base + s) as usize],
                    expected,
                    "plane {plane} sample {s}"
                );
            }
        }
    }

    #[test]
    fn bad_lengths_and_overruns_are_rejected() {
        let mut out = Vec::new();
        assert!(unpack_color8(&G300, &[0; 10], 0, 1, &mut out).is_none());
        assert!(unpack_color8(&G300, &vec![0; 11664], 3888, 1, &mut out).is_none());
        assert!(out.is_empty());
    }

    #[test]
    fn gray_line_follows_the_single_plane_order() {
        let w = G300.line_pixels();
        // Byte value = (sample index) mod 251 so sources can be identified.
        let raw: Vec<u8> = (0..w).map(|i| (i % 251) as u8).collect();
        let mut out = Vec::new();
        unpack_gray8(&G300, &raw, 0, 4, &mut out).unwrap();
        // Rotated pixel x < S is segment 1, pixel k = x: σ = W − 1 − (3k + 1).
        let expect: Vec<u8> = (0..4)
            .map(|k| ((w - 1 - (3 * k + 1)) % 251) as u8)
            .collect();
        assert_eq!(out, expect);
        assert!(unpack_gray8(&G300, &raw[..w - 1], 0, 4, &mut out).is_none());
    }

    #[test]
    fn sixteen_bit_lines_use_u16_samples_and_a_one_pixel_overlap() {
        let w = G300.line_pixels();
        let raw: Vec<u8> = (0..3 * w as u32)
            .flat_map(|i| (i as u16).to_le_bytes())
            .collect();
        let mut out = Vec::new();
        unpack_color16(&G300, &raw, 0, 2, &mut out).unwrap();
        // Pixel 0: segment 1, k = 0 → σ = W − 2; planes R=1, G=2, B=0.
        let sigma0 = (w - 2) as u16;
        assert_eq!(
            out,
            vec![
                w as u16 + sigma0,
                2 * w as u16 + sigma0,
                sigma0,
                w as u16 + sigma0 - 3,
                2 * w as u16 + sigma0 - 3,
                sigma0 - 3
            ]
        );
        // Segment 0 starts at rotated x = 2S with k = overlap = 1 in 16-bit (2 in 8-bit).
        let s = G300.seg_pixels;
        let mut o16 = Vec::new();
        unpack_gray16(&G300, &raw[..2 * w], 2 * s, 1, &mut o16).unwrap();
        assert_eq!(o16, vec![(w - 1 - 3) as u16]);
        let raw8: Vec<u8> = (0..w).map(|i| (i % 251) as u8).collect();
        let mut o8 = Vec::new();
        unpack_gray8(&G300, &raw8, 2 * s, 1, &mut o8).unwrap();
        assert_eq!(o8, vec![((w - 1 - (3 * 2)) % 251) as u8]);
    }
}
