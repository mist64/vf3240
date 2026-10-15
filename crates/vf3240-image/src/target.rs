//! The printable seam-calibration target: an A4 landscape PDF with a square
//! dot grid, pitch 0.42333 mm (10 printer dots at 600 dpi, 20 at 1200 dpi,
//! 40 scanner pixels at 2400 dpi), 5 mm margins, a 0.5 mm frame and a
//! caption. Placed with its long edge along the scan line it spans all 16
//! chip seams (15–289 mm of the 304 mm bed). The grid is not rotated: the
//! horizontal lattice vector then measures the seam gap alone and the
//! vertical one the chip stagger alone (`seamcal`).
//!
//! Written by hand (catalog, pages, page, Flate content stream, Helvetica),
//! so the core needs nothing beyond `flate2`.

use flate2::write::ZlibEncoder;
use flate2::Compression;
use std::io::Write;

/// Dot pitch in mm (exactly 40 px at 2400 dpi).
pub const PITCH_MM: f64 = 10.0 * 25.4 / 600.0;
const PAGE_W_MM: f64 = 297.0;
const PAGE_H_MM: f64 = 210.0;
const MARGIN_MM: f64 = 5.0;
const PT_PER_MM: f64 = 72.0 / 25.4;

/// Build the PDF bytes.
pub fn seam_target_pdf() -> std::io::Result<Vec<u8>> {
    let pt = |mm: f64| mm * PT_PER_MM;
    let dot = PITCH_MM / 2.0;
    let (x0, x1) = (MARGIN_MM, PAGE_W_MM - MARGIN_MM);
    let (y0, y1) = (MARGIN_MM, PAGE_H_MM - MARGIN_MM);
    let mut content = String::with_capacity(8 << 20);
    content.push_str("0 g\n");
    // Dot field, filled in batches so no single path is enormous.
    let nx = ((x1 - x0 - dot) / PITCH_MM).floor() as usize;
    let ny = ((y1 - y0 - dot) / PITCH_MM).floor() as usize;
    let mut in_batch = 0;
    for j in 0..ny {
        let y = y0 + dot / 2.0 + j as f64 * PITCH_MM;
        for i in 0..nx {
            let x = x0 + dot / 2.0 + i as f64 * PITCH_MM;
            content.push_str(&format!(
                "{:.3} {:.3} {:.3} {:.3} re\n",
                pt(x),
                pt(y),
                pt(dot),
                pt(dot)
            ));
            in_batch += 1;
            if in_batch == 2000 {
                content.push_str("f\n");
                in_batch = 0;
            }
        }
    }
    if in_batch > 0 {
        content.push_str("f\n");
    }
    // Frame: 0.5 mm rules just inside the margins (horizontal rules for the
    // edge cross-check, vertical ones to locate the seams).
    let rule = 0.5;
    for (x, y, w, h) in [
        (x0 - rule, y0 - rule, x1 - x0 + 2.0 * rule, rule),
        (x0 - rule, y1, x1 - x0 + 2.0 * rule, rule),
        (x0 - rule, y0 - rule, rule, y1 - y0 + 2.0 * rule),
        (x1, y0 - rule, rule, y1 - y0 + 2.0 * rule),
    ] {
        content.push_str(&format!(
            "{:.3} {:.3} {:.3} {:.3} re f\n",
            pt(x),
            pt(y),
            pt(w),
            pt(h)
        ));
    }
    // Caption in the bottom margin.
    content.push_str(&format!(
        "BT /F1 7 Tf {:.2} {:.2} Td (VF3240 seam calibration target: print at 100% \\(no scaling\\), lay the long edge along the scan line, run vf3240 --calibrate-seams) Tj ET\n",
        pt(x0),
        pt(1.5)
    ));
    let mut enc = ZlibEncoder::new(Vec::new(), Compression::best());
    enc.write_all(content.as_bytes())?;
    let stream = enc.finish()?;

    let mut pdf: Vec<u8> = Vec::with_capacity(stream.len() + 1024);
    let mut offsets = Vec::new();
    pdf.extend_from_slice(b"%PDF-1.4\n%\xE2\xE3\xCF\xD3\n");
    let mut obj = |pdf: &mut Vec<u8>, body: &[u8]| {
        offsets.push(pdf.len());
        let n = offsets.len();
        pdf.extend_from_slice(format!("{n} 0 obj\n").as_bytes());
        pdf.extend_from_slice(body);
        pdf.extend_from_slice(b"\nendobj\n");
    };
    obj(&mut pdf, b"<< /Type /Catalog /Pages 2 0 R >>");
    obj(&mut pdf, b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>");
    obj(
        &mut pdf,
        format!(
            "<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {:.3} {:.3}] /Contents 4 0 R /Resources << /Font << /F1 5 0 R >> >> >>",
            pt(PAGE_W_MM),
            pt(PAGE_H_MM)
        )
        .as_bytes(),
    );
    let mut body = format!(
        "<< /Length {} /Filter /FlateDecode >>\nstream\n",
        stream.len()
    )
    .into_bytes();
    body.extend_from_slice(&stream);
    body.extend_from_slice(b"\nendstream");
    obj(&mut pdf, &body);
    obj(
        &mut pdf,
        b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>",
    );
    let xref = pdf.len();
    pdf.extend_from_slice(
        format!("xref\n0 {}\n0000000000 65535 f \n", offsets.len() + 1).as_bytes(),
    );
    for o in &offsets {
        pdf.extend_from_slice(format!("{o:010} 00000 n \n").as_bytes());
    }
    pdf.extend_from_slice(
        format!(
            "trailer\n<< /Size {} /Root 1 0 R >>\nstartxref\n{xref}\n%%EOF\n",
            offsets.len() + 1
        )
        .as_bytes(),
    );
    Ok(pdf)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pitch_is_forty_pixels_at_2400_dpi() {
        assert!((PITCH_MM / 25.4 * 2400.0 - 40.0).abs() < 1e-9);
    }

    #[test]
    fn pdf_has_the_expected_skeleton() {
        let pdf = seam_target_pdf().unwrap();
        assert!(pdf.starts_with(b"%PDF-1.4"));
        assert!(pdf.ends_with(b"%%EOF\n"));
        let text = String::from_utf8_lossy(&pdf);
        assert!(text.contains("/Type /Catalog"));
        assert!(text.contains("/FlateDecode"));
        assert_eq!(text.matches(" 0 obj").count(), 5);
        // startxref points at the xref table.
        let sx: usize = text
            .rsplit("startxref\n")
            .next()
            .unwrap()
            .lines()
            .next()
            .unwrap()
            .parse()
            .unwrap();
        assert!(pdf[sx..].starts_with(b"xref"));
    }
}
