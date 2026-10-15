# vf3240

Open source driver for the VIISAN VF3240 A3 scanner that corrects what the
hardware gets wrong: the colour fringe from the LED sequence, the stick-slip
of the carriage, and the gaps and stagger of its 18 sensor chips. The vendor
driver passes all three through. Pure Rust over libusb, no vendor software.

Also sold as the Mysher VF3240 and the Sanwa PSC12UF / 400-SCN057; USB
`2e5a:d011`. The defects and the corrections are explained in
[this post](https://www.pagetable.com/?p=302).

## Features

- **Every mode of the scanner:** 200, 300, 600, 1200 and 2400 dpi, colour
  and gray, 8 and 16 bit, the whole A3 bed or any area of it.
- **Streamed output** to TIFF (BigTIFF above 3.5 GiB), PNG or PPM, so a
  3.5 GB full bed at 2400 dpi needs a few MB of memory. 7 minutes in 8 bit,
  17 in 16 bit (the scanner delivers at most about 8.5 MB/s over USB).
- **The vendor's calibration**, run before every scan as the vendor driver
  does it: AFE offset and exposure found by bisection on the white strip,
  white and dark shading tables uploaded to the ASIC, hardware gamma. The
  output matches the vendor driver's within noise.
- **Three corrections the vendor driver does not make.** The scanner has
  mechanical and timing flaws that are stable enough to measure and undo,
  so the result is sharper than the vendor's own output:
  - *Colour registration.* The red, green and blue LEDs flash one after the
    other within each line while the carriage keeps moving, so R and B land
    a fraction of a line below G: 0.42 and 0.72 lines at 2400 dpi, a little
    less at lower resolutions, the same in the vendor's output. The
    channels are resampled vertically by those offsets. This is firmware
    timing, so it should be the same on every unit.
  - *Motor staircase.* At 2400 dpi one motor step is four lines, and the
    carriage does not glide: it sticks and jumps once per step, so the four
    lines sample the paper at spacings of roughly 0.5 / 1.1 / 2.0 / 0.3
    lines instead of 1 / 1 / 1 / 1, and the jump point drifts along the
    rail. The real spacings are measured continuously from the edge
    statistics of the image itself and every row is resampled onto a
    uniform grid. Also applied at 1200 dpi (two lines per step).
  - *Sensor chip seams.* The sensor is 18 chips in a row. Each junction
    loses 3.3 to 5.4 pixels, which kinks every line that crosses it, and
    each chip sits up to 1.3 rows further along the scan direction than its
    neighbour, a stagger of up to 14 µm. Rows are resampled with the gaps
    filled and each chip's columns shifted back into line. Both are
    placement tolerances of the individual scanner, measured with a
    printed target (see Calibration).
- **Scan on the front-panel buttons**, into numbered files.

Not done: platform front-ends (SANE, TWAIN/WIA, ICA) and the C ABI they
would use.

## Calibration

The seam gaps and the chip stagger described above are placement
tolerances of the individual scanner, so the tool needs the numbers for
yours. Until it has them it fills 4 pixels at every seam and leaves the
stagger alone, which is only roughly right, and says so at every start.
Measuring takes a printed target and about a quarter of an hour:

```sh
vf3240 --seam-target target.pdf
```

Print it on A4 at 100 % with no scaling. Lay it on the glass with its long
edge along the short side of the bed, anywhere in the top half; it covers
all 16 seams. Then:

```sh
vf3240 --calibrate-seams
```

This scans the target at 2400 dpi three times, asking for the sheet to be
moved or turned between passes, measures every seam in every pass and
takes the median. Three passes are needed because the apparent gap at a
junction depends slightly on how the paper lies on the glass, by up to
half a pixel either way; the median is what a typical page sees, and the
table reports the range across the passes. It then prints one line of
the form

```
--seams 4.08,3.76/1.20,…,5.02/0.80
```

sixteen `gap/step` pairs at 2400 dpi. Pass that line to every scan, put it
in a file and pass `--seams @file`, or set it once in the environment as
`VF3240_SEAMS`. Seams the sheet did not cover keep their previous value and
are listed as such. `--passes N` changes the count. `--measure-seams`
runs the measurement on existing full-width 2400 dpi scans made with
`--no-deseam --no-destagger`, which also works on any page with a printed
halftone screen.

The other two corrections, the colour registration and the motor
staircase, are measured from the scan itself and need nothing.

## Use

```sh
cargo build --release                      # the binary is target/release/vf3240
vf3240 page.tif --dpi 2400                 # one scan of the whole bed
vf3240 photo.png --dpi 600 --area 1,2,6,4  # part of the bed: x, y, width, height in inches from the top-left corner
vf3240 --multiple ~/scans --dpi 600        # scan on every button press
```

Add the `--seams …` line from your calibration to every one of these, or
export it once as `VF3240_SEAMS`.

Run `vf3240` without arguments for the full option list. Needs libusb and
a scanner that no other driver has claimed.

## Platforms

Developed and tested on macOS only, with the vendor's ICA driver moved out
of `/Library/Image Capture/Devices` so that libusb can claim the scanner.
The code has nothing platform-specific in it: pure Rust, libusb through
`rusb`, the standard library for files and threads, and CI compiles it on
Linux as well. It has not been run anywhere else. What the other platforms
would need:

- **Linux:** a udev rule granting access to USB `2e5a:d011`, or root. No
  kernel driver claims the device, so nothing has to be detached.
- **Windows:** libusb can only reach a device bound to the WinUSB driver,
  so the vendor's TWAIN driver has to be replaced with WinUSB (Zadig does
  this). After that nothing in the code changes.

Reports from either platform are welcome.

## How it was made

- The USB protocol was recovered from the vendor's Linux SANE driver, the
  one build that ships with debug information (`docs/SOURCES.md` names the
  package and its hash).
- The result is the specification in `docs/protocol/`, one document per
  subsystem, each opening with a line saying whether the hardware has
  confirmed it.
- The code was written from the specification, and every protocol fact was
  then checked on the real scanner; `docs/hardware/README.md` is the record.
- The vendor package and the decompilation are not in this repository;
  citations of the form `re/exports/…` in the protocol documents point
  into that private lab.
- `crates/vf3240-core/src/tables.rs` holds the register initialisation
  tables, motor curves and default gamma transcribed from the vendor
  driver; the hardware does not work without them.
- The purpose is interoperability: making this hardware usable on systems
  the vendor does not support.

## Development

Plain Cargo workspace: `cargo test` needs no scanner. The hardware tests run
with `cargo test -p vf3240-core --features hardware -- --test-threads=1`
against a plugged-in scanner, single-threaded because each test claims the
device. The measurement tools used during development are in
`crates/vf3240-image/examples/`.

Rules for changes:

- Clean room. `docs/protocol/` is the specification, derived from the
  vendor driver in a private lab repository; the code is written from the
  specification. No vendor code or decompiler output here.
- Every protocol constant cites the vendor function it came from. Every
  protocol document opens with a `Status:` line saying whether hardware
  has confirmed it; observations go to `docs/hardware/README.md`.
- Known vendor bugs are not reproduced: short bulk transfers are errors.

## Author

Claude, directed by Michael Steil <mist64@mac.com>.

## License

MIT or Apache-2.0, at your option.
