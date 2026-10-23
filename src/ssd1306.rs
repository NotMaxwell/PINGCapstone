//! Async I²C driver for a 128 by 64 monochrome OLED with an SSD1306 or
//! SH1106 controller, showing lines of text.
//!
//! These are the small 0.96" and 1.3" modules with four pins (GND, VCC, SCL,
//! SDA) and pull-up resistors already on board. Both controllers keep the
//! picture in their own RAM as eight *pages*, each a strip eight pixels tall
//! across the full width, with one byte per column and the top pixel in bit
//! 0. The SSD1306 is 128 columns wide; the SH1106 has 132 and the glass sits
//! two columns in, which is all [`Ssd1306::new`]'s `column_offset` is for.
//! Only commands the two share are used, so either works.
//!
//! # Text
//!
//! Text is drawn with the 8 by 13 pixel font from `embedded-graphics`, on
//! rows two pages (16 pixels) apart, which gives [`ROWS`] lines of
//! [`COLUMNS`] characters. The font covers ISO 8859-1, so `°` and `±` show as
//! themselves; anything else shows as `?`.
//!
//! The driver keeps a copy of the picture and sends only the two pages of a
//! row when that row changes: 260 bytes, about 6 ms at 400 kHz.
//!
//! # Example
//!
//! ```ignore
//! let mut display = Ssd1306::new(i2c, 0x3C, 0);
//! display.init().await?;
//! display.write_line(0, "distance").await?;
//! display.print_line(1, format_args!("{} cm", 1234)).await?;
//! ```

use core::convert::Infallible;
use core::fmt::{self, Write as _};

use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::mono_font::iso_8859_1::FONT_8X13;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::text::{Baseline, Text};
use embedded_hal_async::i2c::I2c;

/// The panel's size in pixels.
pub const WIDTH: usize = 128;
pub const HEIGHT: usize = 64;
const PAGES: usize = HEIGHT / 8;

/// Characters per line.
pub const COLUMNS: usize = WIDTH / 8;
/// Lines of text, each two pages tall.
pub const ROWS: usize = PAGES / 2;

// The first byte of every I²C write says what follows.
const CONTROL_COMMANDS: u8 = 0x00;
const CONTROL_DATA: u8 = 0x40;

/// Brings the panel up the right way round, in page addressing mode, with
/// the internal charge pump on. Pairs are a command and its argument.
const INIT_SEQUENCE: [u8; 25] = [
    0xAE, // display off while setting up
    0xD5, 0x80, // clock divider: the datasheet's default
    0xA8, 0x3F, // multiplex ratio: 64 rows
    0xD3, 0x00, // no vertical offset
    0x40, // start at RAM line 0
    0x8D, 0x14, // charge pump on (SSD1306; the SH1106 ignores this)
    0x20, 0x02, // page addressing mode (SSD1306; the SH1106 has no other)
    0xA1, // column 127 on the left: the modules mount the glass that way
    0xC8, // scan rows bottom to top, for the same reason
    0xDA, 0x12, // row wiring for 64-row panels
    0x81, 0xCF, // contrast
    0xD9, 0xF1, // pre-charge period
    0xDB, 0x40, // VCOMH level
    0xA4, // show RAM contents
    0xA6, // not inverted
    0xAF, // display on
];

const DISPLAY_NORMAL: u8 = 0xA6;
const DISPLAY_INVERTED: u8 = 0xA7;
const SET_PAGE: u8 = 0xB0;
const SET_COLUMN_LOW: u8 = 0x00;
const SET_COLUMN_HIGH: u8 = 0x10;

/// An SSD1306 or SH1106 OLED on I²C.
pub struct Ssd1306<I> {
    i2c: I,
    address: u8,
    column_offset: u8,
    frame: Frame,
}

impl<I: I2c> Ssd1306<I> {
    /// A driver for the panel at `address`: 0x3C on most modules, 0x3D with
    /// the address resistor moved. `column_offset` is 0 for an SSD1306 and 2
    /// for an SH1106. Call [`Ssd1306::init`] before anything else.
    pub fn new(i2c: I, address: u8, column_offset: u8) -> Self {
        Self {
            i2c,
            address,
            column_offset,
            frame: Frame([[0; WIDTH]; PAGES]),
        }
    }

    /// Sets the panel up, blanks it and turns it on. Safe to call again at
    /// any time.
    pub async fn init(&mut self) -> Result<(), I::Error> {
        self.commands(&INIT_SEQUENCE).await?;
        self.clear().await
    }

    /// Blanks the whole screen.
    pub async fn clear(&mut self) -> Result<(), I::Error> {
        self.frame.0 = [[0; WIDTH]; PAGES];
        for page in 0..PAGES {
            self.send_page(page).await?;
        }
        Ok(())
    }

    /// Swaps light and dark across the whole screen, or swaps them back.
    /// Takes effect straight away and leaves the picture alone.
    pub async fn set_inverted(&mut self, inverted: bool) -> Result<(), I::Error> {
        let command = if inverted {
            DISPLAY_INVERTED
        } else {
            DISPLAY_NORMAL
        };
        self.commands(&[command]).await
    }

    /// Replaces line `row` with `text`, cut off at [`COLUMNS`] characters.
    /// Rows past the bottom of the screen are ignored.
    pub async fn write_line(&mut self, row: usize, text: &str) -> Result<(), I::Error> {
        if row >= ROWS {
            return Ok(());
        }
        let text = match text.char_indices().nth(COLUMNS) {
            Some((end, _)) => &text[..end],
            None => text,
        };

        let top = 2 * row;
        self.frame.0[top] = [0; WIDTH];
        self.frame.0[top + 1] = [0; WIDTH];
        let style = MonoTextStyle::new(&FONT_8X13, BinaryColor::On);
        // Drawing into the frame cannot fail, and anything off the edge is
        // simply not drawn.
        let _ = Text::with_baseline(text, Point::new(0, 8 * top as i32), style, Baseline::Top)
            .draw(&mut self.frame);

        self.send_page(top).await?;
        self.send_page(top + 1).await
    }

    /// Replaces line `row` with formatted text, as [`Ssd1306::write_line`]
    /// does.
    ///
    /// ```ignore
    /// display.print_line(0, format_args!("{:.1} m", range_m)).await?;
    /// ```
    pub async fn print_line(
        &mut self,
        row: usize,
        args: fmt::Arguments<'_>,
    ) -> Result<(), I::Error> {
        let mut text = Line::new();
        // A Line only ever fails by filling up, and what fits is kept.
        let _ = text.write_fmt(args);
        self.write_line(row, text.as_str()).await
    }

    /// Asks every ordinary I²C address in turn whether anything is there.
    ///
    /// For finding out why the display does not answer: an empty bus points
    /// at power or wiring, and a device somewhere else is usually the
    /// display at another address. Each address is asked for one byte, which
    /// a display answers with its status and nothing else.
    pub async fn scan(&mut self) -> Addresses {
        let mut found = 0u128;
        for address in 0x08..=0x77u8 {
            if self.i2c.read(address, &mut [0]).await.is_ok() {
                found |= 1 << address;
            }
        }
        Addresses(found)
    }

    /// Hands the I²C bus back.
    pub fn release(self) -> I {
        self.i2c
    }

    /// Sends one or more command bytes in a single write.
    async fn commands(&mut self, commands: &[u8]) -> Result<(), I::Error> {
        let mut buf = [0u8; 1 + INIT_SEQUENCE.len()];
        buf[0] = CONTROL_COMMANDS;
        buf[1..=commands.len()].copy_from_slice(commands);
        self.i2c.write(self.address, &buf[..=commands.len()]).await
    }

    /// Copies one page of the frame to the panel.
    async fn send_page(&mut self, page: usize) -> Result<(), I::Error> {
        let column = self.column_offset;
        self.commands(&[
            SET_PAGE | page as u8,
            SET_COLUMN_LOW | (column & 0x0F),
            SET_COLUMN_HIGH | (column >> 4),
        ])
        .await?;

        let mut buf = [0u8; 1 + WIDTH];
        buf[0] = CONTROL_DATA;
        buf[1..].copy_from_slice(&self.frame.0[page]);
        self.i2c.write(self.address, &buf).await
    }
}

/// The picture, laid out as the panel's RAM is: a page per row of bytes,
/// one byte per column, top pixel in bit 0.
struct Frame([[u8; WIDTH]; PAGES]);

impl OriginDimensions for Frame {
    fn size(&self) -> Size {
        Size::new(WIDTH as u32, HEIGHT as u32)
    }
}

impl DrawTarget for Frame {
    type Color = BinaryColor;
    type Error = Infallible;

    fn draw_iter<P>(&mut self, pixels: P) -> Result<(), Self::Error>
    where
        P: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(point, color) in pixels {
            let (Ok(x), Ok(y)) = (usize::try_from(point.x), usize::try_from(point.y)) else {
                continue;
            };
            if x >= WIDTH || y >= HEIGHT {
                continue;
            }
            let byte = &mut self.0[y / 8][x];
            let bit = 1 << (y % 8);
            if color.is_on() {
                *byte |= bit;
            } else {
                *byte &= !bit;
            }
        }
        Ok(())
    }
}

/// The I²C addresses that answered a [`Ssd1306::scan`]. Displays as a list,
/// such as `0x3c, 0x68`, or `none`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Addresses(u128);

impl Addresses {
    /// The addresses, lowest first.
    pub fn iter(self) -> impl Iterator<Item = u8> {
        (0..128u8).filter(move |&address| self.0 & (1 << address) != 0)
    }
}

impl fmt::Display for Addresses {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut addresses = self.iter();
        match addresses.next() {
            None => f.write_str("none"),
            Some(first) => {
                write!(f, "{first:#04x}")?;
                addresses.try_for_each(|address| write!(f, ", {address:#04x}"))
            }
        }
    }
}

/// A line of text being formatted, cut off at a little more than a screen
/// line's worth of bytes; [`Ssd1306::write_line`] trims it to [`COLUMNS`]
/// characters.
struct Line {
    bytes: [u8; 4 * COLUMNS],
    len: usize,
}

impl Line {
    const fn new() -> Self {
        Self {
            bytes: [0; 4 * COLUMNS],
            len: 0,
        }
    }

    fn as_str(&self) -> &str {
        // Only whole characters are ever stored, so this cannot fail.
        core::str::from_utf8(&self.bytes[..self.len]).unwrap_or("")
    }
}

impl fmt::Write for Line {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for c in s.chars() {
            let mut utf8 = [0u8; 4];
            let encoded = c.encode_utf8(&mut utf8).as_bytes();
            let end = self.len + encoded.len();
            if end > self.bytes.len() {
                return Err(fmt::Error);
            }
            self.bytes[self.len..end].copy_from_slice(encoded);
            self.len = end;
        }
        Ok(())
    }
}
