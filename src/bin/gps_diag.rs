//! Diagnostic binary for the NEO-M9N: finds out why there is no fix.
//!
//! The driver in `neo_m9n.rs` only decodes what the application needs. This
//! speaks raw UBX instead, so it can poll anything the receiver reports and
//! print every field, and prints over RTT only. In order it:
//!
//! 1. sniffs the line for a moment and reports what the receiver is sending
//!    unprompted (NMEA, UBX or nothing), which checks wiring and baud rate;
//! 2. prints every `UBX-MON-VER` string, including the supported GNSS;
//! 3. reads back the configuration that decides whether a fix is possible:
//!    enabled constellations, antenna supervision, navigation filters and
//!    power mode;
//! 4. every two seconds polls `UBX-NAV-STATUS`, `UBX-NAV-PVT`, `UBX-MON-RF`,
//!    `UBX-MON-SPAN` and `UBX-NAV-SAT`, which together separate "no antenna",
//!    "antenna but no sky" and "jammed", and counts TIMEPULSE edges.
//!
//! Nothing is written to the receiver except polls and VALGET requests, so
//! its configuration is left exactly as it was.

#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_io_async::{Read, Write};
use esp_hal::Async;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Pull};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, Uart};
use ping_capstone::constants::{GPS_BAUD_RATE, HEAP_SIZE};
use ping_capstone::{board_bus, board_pin};
use rtt_target::{rprint, rprintln};

#[panic_handler]
fn panic(panic_info: &core::panic::PanicInfo) -> ! {
    rprintln!("{}", panic_info);
    loop {}
}

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

/// TIMEPULSE edges seen since the last report.
static TIMEPULSE_EDGES: AtomicU32 = AtomicU32::new(0);

/// The receiver's uptime at the last NAV-STATUS, to spot it restarting.
static LAST_UPTIME_MS: AtomicU32 = AtomicU32::new(0);

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    rtt_target::rtt_init_print!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: HEAP_SIZE);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    // Wi-Fi is deliberately not started: the radio is the one on-board
    // source of RF noise, and this should see the receiver without it.
    let uart = Uart::new(
        board_bus!(peripherals.GPS_UART),
        UartConfig::default().with_baudrate(GPS_BAUD_RATE),
    )
    .expect("Failed to configure UART")
    .with_tx(board_pin!(peripherals.GPS_TX))
    .with_rx(board_pin!(peripherals.GPS_RX))
    .into_async();

    // TIMEPULSE is optional wiring. A pull-down keeps an unconnected pin
    // from counting noise as pulses.
    let timepulse = Input::new(
        board_pin!(peripherals.GPS_TIMEPULSE),
        InputConfig::default().with_pull(Pull::Down),
    );
    spawner.spawn(timepulse_task(timepulse).expect("Failed to spawn the TIMEPULSE task"));

    spawner.spawn(diag::diag_task(uart).expect("Failed to spawn the diagnostic task"));

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}

#[embassy_executor::task]
async fn timepulse_task(mut pin: Input<'static>) {
    loop {
        pin.wait_for_rising_edge().await;
        TIMEPULSE_EDGES.fetch_add(1, Ordering::Relaxed);
    }
}

mod diag {
    //! The diagnostic task, in its own module so the stack frame lint can be
    //! relaxed: the frame buffer lives in the task's static future.
    #![allow(clippy::large_stack_frames)]

    use super::*;

    const SYNC: [u8; 2] = [0xb5, 0x62];
    /// MON-SPAN is the largest message polled: 4 + 272 bytes per RF block.
    const MAX_PAYLOAD: usize = 1200;
    const POLL_TIMEOUT: Duration = Duration::from_millis(1500);

    // (class, id) of the messages used.
    const ACK_NAK: (u8, u8) = (0x05, 0x00);
    const CFG_VALGET: (u8, u8) = (0x06, 0x8b);
    const MON_VER: (u8, u8) = (0x0a, 0x04);
    const MON_RF: (u8, u8) = (0x0a, 0x38);
    const MON_SPAN: (u8, u8) = (0x0a, 0x31);
    const NAV_STATUS: (u8, u8) = (0x01, 0x03);
    const NAV_PVT: (u8, u8) = (0x01, 0x07);
    const NAV_SAT: (u8, u8) = (0x01, 0x35);

    /// Configuration keys worth reading back, with their names.
    const KEYS: &[(u32, &str)] = &[
        (0x1031001f, "SIGNAL-GPS_ENA"),
        (0x10310001, "SIGNAL-GPS_L1CA_ENA"),
        (0x10310020, "SIGNAL-SBAS_ENA"),
        (0x10310021, "SIGNAL-GAL_ENA"),
        (0x10310022, "SIGNAL-BDS_ENA"),
        (0x10310024, "SIGNAL-QZSS_ENA"),
        (0x10310025, "SIGNAL-GLO_ENA"),
        (0x10a3002e, "HW-ANT_CFG_VOLTCTRL"),
        (0x10a3002f, "HW-ANT_CFG_SHORTDET"),
        (0x10a30031, "HW-ANT_CFG_OPENDET"),
        (0x10a30033, "HW-ANT_CFG_PWRDOWN"),
        (0x20110011, "NAVSPG-FIXMODE"),
        (0x20110021, "NAVSPG-DYNMODEL"),
        (0x201100a1, "NAVSPG-INFIL_MINSVS"),
        (0x201100a3, "NAVSPG-INFIL_MINCNO"),
        (0x201100a4, "NAVSPG-INFIL_MINELEV"),
        (0x201100aa, "NAVSPG-INFIL_NCNOTHRS"),
        (0x201100ab, "NAVSPG-INFIL_CNOTHRS"),
        (0x20d00001, "PM-OPERATEMODE"),
        (0x1041000d, "ITFM-ENABLE"),
        (0x30210001, "RATE-MEAS"),
        (0x30210002, "RATE-NAV"),
        (0x40520001, "UART1-BAUDRATE"),
        (0x10740001, "UART1OUTPROT-UBX"),
        (0x10740002, "UART1OUTPROT-NMEA"),
        (0x20910007, "MSGOUT-UBX_NAV_PVT_UART1"),
    ];

    struct Ubx {
        uart: Uart<'static, Async>,
        payload: [u8; MAX_PAYLOAD],
        // Frame reassembly state.
        state: usize,
        class: u8,
        id: u8,
        len: usize,
        got: usize,
        ck: (u8, u8),
        /// Frames dropped for a bad checksum or oversize length.
        bad_frames: u32,
    }

    fn u16le(b: &[u8], at: usize) -> u16 {
        u16::from_le_bytes([b[at], b[at + 1]])
    }
    fn i16le(b: &[u8], at: usize) -> i16 {
        u16le(b, at) as i16
    }
    fn u32le(b: &[u8], at: usize) -> u32 {
        u32::from_le_bytes([b[at], b[at + 1], b[at + 2], b[at + 3]])
    }
    fn i32le(b: &[u8], at: usize) -> i32 {
        u32le(b, at) as i32
    }

    impl Ubx {
        async fn send(&mut self, class: u8, id: u8, payload: &[u8]) {
            let mut frame = [0u8; 64];
            let n = payload.len();
            frame[..2].copy_from_slice(&SYNC);
            frame[2] = class;
            frame[3] = id;
            frame[4..6].copy_from_slice(&(n as u16).to_le_bytes());
            frame[6..6 + n].copy_from_slice(payload);
            let (mut a, mut b) = (0u8, 0u8);
            for &byte in &frame[2..6 + n] {
                a = a.wrapping_add(byte);
                b = b.wrapping_add(a);
            }
            frame[6 + n] = a;
            frame[7 + n] = b;
            if let Err(e) = self.uart.write_all(&frame[..8 + n]).await {
                rprintln!("UART write failed: {:?}", e);
            }
            let _ = Write::flush(&mut self.uart).await;
        }

        /// Feeds one byte to the frame reassembler; returns true when a
        /// complete, valid frame is in `class`, `id`, `payload[..len]`.
        fn feed(&mut self, byte: u8) -> bool {
            let ck_add = |ck: &mut (u8, u8), byte: u8| {
                ck.0 = ck.0.wrapping_add(byte);
                ck.1 = ck.1.wrapping_add(ck.0);
            };
            match self.state {
                0 => self.state = usize::from(byte == SYNC[0]),
                1 => {
                    self.state = if byte == SYNC[1] {
                        2
                    } else {
                        usize::from(byte == SYNC[0])
                    }
                }
                2 => {
                    self.ck = (0, 0);
                    ck_add(&mut self.ck, byte);
                    self.class = byte;
                    self.state = 3;
                }
                3 => {
                    ck_add(&mut self.ck, byte);
                    self.id = byte;
                    self.state = 4;
                }
                4 => {
                    ck_add(&mut self.ck, byte);
                    self.len = byte as usize;
                    self.state = 5;
                }
                5 => {
                    ck_add(&mut self.ck, byte);
                    self.len |= (byte as usize) << 8;
                    self.got = 0;
                    if self.len > MAX_PAYLOAD {
                        self.bad_frames += 1;
                        self.state = 0;
                    } else {
                        self.state = if self.len == 0 { 7 } else { 6 };
                    }
                }
                6 => {
                    ck_add(&mut self.ck, byte);
                    self.payload[self.got] = byte;
                    self.got += 1;
                    if self.got == self.len {
                        self.state = 7;
                    }
                }
                7 => {
                    if byte == self.ck.0 {
                        self.state = 8;
                    } else {
                        self.bad_frames += 1;
                        self.state = 0;
                    }
                }
                _ => {
                    self.state = 0;
                    if byte == self.ck.1 {
                        return true;
                    }
                    self.bad_frames += 1;
                }
            }
            false
        }

        /// Reads until a frame of `want` arrives (Ok) or an ACK-NAK for
        /// `nak_for` does (Err), or the timeout passes (Err).
        async fn wait_for(
            &mut self,
            want: (u8, u8),
            nak_for: (u8, u8),
        ) -> Result<usize, &'static str> {
            let deadline = Instant::now() + POLL_TIMEOUT;
            let mut chunk = [0u8; 64];
            loop {
                let left = deadline.saturating_duration_since(Instant::now());
                if left == Duration::from_ticks(0) {
                    return Err("timeout");
                }
                let n = match with_timeout(left, Read::read(&mut self.uart, &mut chunk)).await {
                    Err(_) => return Err("timeout"),
                    Ok(Err(e)) => {
                        rprintln!("  (UART read error {:?})", e);
                        self.state = 0;
                        continue;
                    }
                    Ok(Ok(n)) => n,
                };
                for &byte in &chunk[..n] {
                    if self.feed(byte) {
                        let got = (self.class, self.id);
                        if got == want {
                            return Ok(self.len);
                        }
                        if got == ACK_NAK
                            && self.len >= 2
                            && (self.payload[0], self.payload[1]) == nak_for
                        {
                            return Err("NAK");
                        }
                    }
                }
            }
        }

        async fn poll(&mut self, msg: (u8, u8)) -> Result<usize, &'static str> {
            self.send(msg.0, msg.1, &[]).await;
            self.wait_for(msg, msg).await
        }
    }

    /// Prints what arrives on the line in `window` without asking for
    /// anything.
    async fn sniff(ubx: &mut Ubx, window: Duration) {
        let deadline = Instant::now() + window;
        let mut chunk = [0u8; 64];
        let (mut bytes, mut nmea_lines, mut ubx_frames) = (0u32, 0u32, 0u32);
        let mut line = [0u8; 100];
        let mut line_len = 0usize;
        let mut printed = 0;
        let mut first_bytes = [0u8; 24];
        let mut first_len = 0;
        while let Some(left) = deadline.checked_duration_since(Instant::now()) {
            let Ok(Ok(n)) = with_timeout(left, Read::read(&mut ubx.uart, &mut chunk)).await else {
                continue;
            };
            bytes += n as u32;
            for &byte in &chunk[..n] {
                if first_len < first_bytes.len() {
                    first_bytes[first_len] = byte;
                    first_len += 1;
                }
                if ubx.feed(byte) {
                    ubx_frames += 1;
                    if ubx_frames <= 5 {
                        rprintln!(
                            "  UBX frame class {:#04x} id {:#04x} len {}",
                            ubx.class,
                            ubx.id,
                            ubx.len
                        );
                    }
                }
                if byte == b'$' {
                    line_len = 0;
                }
                if line_len < line.len() {
                    line[line_len] = byte;
                    line_len += 1;
                }
                if byte == b'\n' && line[0] == b'$' {
                    nmea_lines += 1;
                    if printed < 12 {
                        printed += 1;
                        let text = core::str::from_utf8(&line[..line_len]).unwrap_or("<not text>");
                        rprint!("  NMEA {}", text);
                    }
                    line_len = 0;
                }
            }
        }
        rprintln!(
            "Sniff: {} bytes in {} ms, {} NMEA sentences, {} UBX frames, {} bad UBX frames",
            bytes,
            window.as_millis(),
            nmea_lines,
            ubx_frames,
            ubx.bad_frames
        );
        rprintln!("  first bytes: {:02x?}", &first_bytes[..first_len]);
    }

    /// Keeps reading for `window` so the 128 byte RX FIFO never overflows
    /// between reports, and throws away what arrives.
    async fn drain(ubx: &mut Ubx, window: Duration) {
        let mut chunk = [0u8; 64];
        let deadline = Instant::now() + window;
        while let Some(left) = deadline.checked_duration_since(Instant::now()) {
            if let Ok(Ok(n)) = with_timeout(left, Read::read(&mut ubx.uart, &mut chunk)).await {
                for &byte in &chunk[..n] {
                    ubx.feed(byte);
                }
            }
        }
    }

    async fn print_version(ubx: &mut Ubx) {
        match ubx.poll(MON_VER).await {
            Ok(len) => {
                fn text(b: &[u8]) -> &str {
                    let end = b.iter().position(|&c| c == 0).unwrap_or(b.len());
                    core::str::from_utf8(&b[..end]).unwrap_or("?")
                }
                rprintln!(
                    "MON-VER sw '{}' hw '{}'",
                    text(&ubx.payload[..30]),
                    text(&ubx.payload[30..40])
                );
                for at in (40..len).step_by(30) {
                    rprintln!("  ext '{}'", text(&ubx.payload[at..at + 30]));
                }
            }
            Err(e) => rprintln!("MON-VER: {}", e),
        }
    }

    async fn print_config(ubx: &mut Ubx) {
        rprintln!("Configuration (RAM layer):");
        for &(key, name) in KEYS {
            let mut req = [0u8; 8];
            // version 0, layer 0 (RAM), position 0, then the key.
            req[4..8].copy_from_slice(&key.to_le_bytes());
            ubx.send(CFG_VALGET.0, CFG_VALGET.1, &req).await;
            match ubx.wait_for(CFG_VALGET, CFG_VALGET).await {
                Ok(len) if len >= 9 => {
                    let size = match (key >> 28) & 0x7 {
                        1 | 2 => 1,
                        3 => 2,
                        4 => 4,
                        _ => 8,
                    };
                    let mut value = 0u64;
                    for i in 0..size.min(len - 8) {
                        value |= u64::from(ubx.payload[8 + i]) << (8 * i);
                    }
                    rprintln!("  {:<28} = {}", name, value);
                }
                Ok(len) => rprintln!("  {:<28} short reply ({} bytes)", name, len),
                Err(e) => rprintln!("  {:<28} {}", name, e),
            }
        }
    }

    const GNSS: [&str; 7] = ["GPS", "SBAS", "GAL", "BDS", "IMES", "QZSS", "GLO"];

    async fn report(ubx: &mut Ubx, started: Instant) {
        rprintln!(
            "---- t={}s  TIMEPULSE edges since last: {}  bad frames: {}",
            started.elapsed().as_secs(),
            TIMEPULSE_EDGES.swap(0, Ordering::Relaxed),
            ubx.bad_frames
        );

        match ubx.poll(NAV_STATUS).await {
            Ok(_) => {
                let p = &ubx.payload;
                let uptime = u32le(p, 12);
                let last = LAST_UPTIME_MS.swap(uptime, Ordering::Relaxed);
                if uptime < last {
                    rprintln!(
                        "!!!!! RECEIVER RESTARTED: uptime went from {} s to {} s",
                        last / 1000,
                        uptime / 1000
                    );
                }
                rprintln!(
                    "NAV-STATUS gpsFix {} flags {:#04x} fixStat {:#04x} flags2 {:#04x} ttff {} ms, receiver up {} s",
                    p[4],
                    p[5],
                    p[6],
                    p[7],
                    u32le(p, 8),
                    u32le(p, 12) / 1000
                );
            }
            Err(e) => rprintln!("NAV-STATUS: {}", e),
        }

        match ubx.poll(NAV_PVT).await {
            Ok(_) => {
                let p = &ubx.payload;
                rprintln!(
                    "NAV-PVT {:04}-{:02}-{:02} {:02}:{:02}:{:02} valid {:#04x} fixType {} flags {:#04x} numSV {} lat {} lon {} hAcc {} m pDOP {}",
                    u16le(p, 4),
                    p[6],
                    p[7],
                    p[8],
                    p[9],
                    p[10],
                    p[11],
                    p[20],
                    p[21],
                    p[23],
                    i32le(p, 28) as f64 * 1e-7,
                    i32le(p, 24) as f64 * 1e-7,
                    u32le(p, 40) / 1000,
                    u16le(p, 76) as f32 * 0.01
                );
            }
            Err(e) => rprintln!("NAV-PVT: {}", e),
        }

        match ubx.poll(MON_RF).await {
            Ok(_) => {
                let p = &ubx.payload;
                let blocks = p[1] as usize;
                for b in 0..blocks {
                    let o = 4 + 24 * b;
                    let ant = ["INIT", "DONTKNOW", "OK", "SHORT", "OPEN"];
                    let pwr = ["OFF", "ON", "DONTKNOW"];
                    let jam = ["unknown", "OK", "WARNING", "CRITICAL"];
                    rprintln!(
                        "MON-RF block {}: antStatus {} antPower {} jamming {} jamInd {}/255 noisePerMS {} AGC {}/8191 ({}%) I ofs {} mag {} Q ofs {} mag {}",
                        p[o],
                        ant.get(p[o + 2] as usize).unwrap_or(&"?"),
                        pwr.get(p[o + 3] as usize).unwrap_or(&"?"),
                        jam[(p[o + 1] & 0x3) as usize],
                        p[o + 16],
                        u16le(p, o + 12),
                        u16le(p, o + 14),
                        u32::from(u16le(p, o + 14)) * 100 / 8191,
                        p[o + 17] as i8,
                        p[o + 18],
                        p[o + 19] as i8,
                        p[o + 20]
                    );
                }
            }
            Err(e) => rprintln!("MON-RF: {}", e),
        }

        match ubx.poll(MON_SPAN).await {
            Ok(_) => {
                let p = &ubx.payload;
                let blocks = p[1] as usize;
                for b in 0..blocks {
                    let o = 4 + 272 * b;
                    let spectrum = &p[o..o + 256];
                    let span = u32le(p, o + 256);
                    let res = u32le(p, o + 260);
                    let center = u32le(p, o + 264);
                    let pga = p[o + 268];
                    let (min, max) = spectrum
                        .iter()
                        .fold((255u8, 0u8), |(lo, hi), &v| (lo.min(v), hi.max(v)));
                    let mean = spectrum.iter().map(|&v| u32::from(v)).sum::<u32>() / 256;
                    // Power around GPS L1 (1575.42 MHz) against the band edges.
                    let bin = |hz: i64| {
                        let start = i64::from(center) - i64::from(span) / 2;
                        ((hz - start) / i64::from(res.max(1))).clamp(0, 255) as usize
                    };
                    let l1 = spectrum[bin(1_575_420_000)];
                    rprintln!(
                        "MON-SPAN block {}: center {} MHz span {} MHz PGA {} dB  min {:.1} mean {:.1} max {:.1} dB  at L1 {:.1} dB",
                        b,
                        center / 1_000_000,
                        span / 1_000_000,
                        pga,
                        f32::from(min) * 0.25,
                        mean as f32 * 0.25,
                        f32::from(max) * 0.25,
                        f32::from(l1) * 0.25
                    );
                    // A coarse 32-column plot of the spectrum.
                    rprint!("  ");
                    for col in spectrum.chunks(8) {
                        let v = col.iter().map(|&v| u32::from(v)).sum::<u32>() / 8;
                        let level = (v.saturating_sub(u32::from(min)) * 8
                            / u32::from(max.saturating_sub(min)).max(1))
                            as usize;
                        rprint!(
                            "{}",
                            [" ", "▁", "▂", "▃", "▄", "▅", "▆", "▇", "█"][level.min(8)]
                        );
                    }
                    rprintln!();
                }
            }
            Err(e) => rprintln!("MON-SPAN: {}", e),
        }

        match ubx.poll(NAV_SAT).await {
            Ok(_) => {
                let p = &ubx.payload;
                let count = p[5] as usize;
                let mut with_signal = 0;
                let mut best = 0;
                rprintln!("NAV-SAT {} satellites in the almanac/search list:", count);
                for s in 0..count {
                    let o = 8 + 12 * s;
                    let cno = p[o + 2];
                    let flags = u32le(p, o + 8);
                    best = best.max(cno);
                    if cno > 0 {
                        with_signal += 1;
                        rprintln!(
                            "  {:<4} {:>3}  C/N0 {:>2} dBHz  elev {:>3}  azim {:>3}  quality {}  used {}  health {}",
                            GNSS.get(p[o] as usize).unwrap_or(&"?"),
                            p[o + 1],
                            cno,
                            p[o + 3] as i8,
                            i16le(p, o + 4),
                            flags & 0x7,
                            (flags >> 3) & 1,
                            (flags >> 4) & 0x3
                        );
                    }
                }
                rprintln!("  {} with any signal, strongest {} dBHz", with_signal, best);
            }
            Err(e) => rprintln!("NAV-SAT: {}", e),
        }
    }

    #[embassy_executor::task]
    pub async fn diag_task(uart: Uart<'static, Async>) {
        let mut ubx = Ubx {
            uart,
            payload: [0; MAX_PAYLOAD],
            state: 0,
            class: 0,
            id: 0,
            len: 0,
            got: 0,
            ck: (0, 0),
            bad_frames: 0,
        };
        let started = Instant::now();

        rprintln!("== Sniffing the GPS line at {} baud", GPS_BAUD_RATE);
        sniff(&mut ubx, Duration::from_millis(2500)).await;

        rprintln!("== Identity");
        print_version(&mut ubx).await;

        rprintln!("== Configuration");
        print_config(&mut ubx).await;

        rprintln!("== Status, every 2 s");
        loop {
            report(&mut ubx, started).await;
            drain(&mut ubx, Duration::from_secs(2)).await;
        }
    }
}
