//! What happens to a scan line after the scanner delivers it: the three
//! corrections (`register`, `stairs`, `seams`), the per-unit seam
//! calibration (`seamcal`, `target`) and the image file formats (`tiff`,
//! `png`, `ppm`). Independent of USB; the scan job in `vf3240-scan` feeds
//! rows through these.

pub mod png;
pub mod ppm;
pub mod register;
pub mod seamcal;
pub mod seams;
pub mod stairs;
pub mod target;
pub mod tiff;
