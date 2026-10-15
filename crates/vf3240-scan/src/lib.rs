//! The VF3240 scan sequence (`job`): home, init, calibrate, program, start,
//! read, stop, with every row passed through the corrections of
//! `vf3240-image` and streamed to a file or a callback. The `vf3240` binary
//! is a thin command line over it; a platform front-end would use `job`
//! the same way.

pub mod job;
