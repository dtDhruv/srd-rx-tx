//! A tiny, robust text modem for an asymmetric SDR link: **HackRF transmits,
//! RTL-SDR receives**.
//!
//! Because the two radios run off independent clocks, there is a carrier
//! frequency offset (CFO) of potentially tens of kHz between them, and the
//! RTL-SDR cannot transmit. The modulation is chosen to survive exactly that:
//!
//! * **OOK (on-off keying)** — the transmitter turns a baseband tone on/off.
//!   The receiver decodes from the *envelope* `|sample|`, which is completely
//!   immune to frequency offset. This is the single most important property
//!   for an unsynchronized HackRF -> RTL-SDR link.
//! * **Manchester coding** — every bit becomes two chips (`1 -> [1,0]`,
//!   `0 -> [0,1]`). Guarantees a transition at least every two chips, which
//!   gives the receiver a clock to lock onto and keeps the average power ~50%.
//! * **Framing** — `preamble (0xAA x8) | SYNC | LEN | PAYLOAD | CRC16`.
//!   The receiver finds the frame by correlating the recovered *chip* stream
//!   against the known preamble+sync chip pattern; that single match recovers
//!   chip alignment, bit alignment, and frame start at once.
//!
//! The same library powers three binaries: `tx` (HackRF), `rx` (RTL-SDR), and
//! `loopback` (a pure-software self test that injects CFO + noise + a random
//! start offset and proves the DSP without any hardware).

use futuresdr::num_complex::Complex32;

/// Bytes of `0xAA` sent before the sync word, for AGC settling + clock lock.
pub const PREAMBLE: [u8; 8] = [0xAA; 8];
/// Distinctive sync word marking the start of the framed bytes.
pub const SYNC: [u8; 2] = [0x2D, 0xD4];
/// How many of the preamble bytes (the tail) participate in sync correlation.
const PREAMBLE_CORR_BYTES: usize = 4;
/// Maximum chip mismatches tolerated when matching the sync pattern.
const MAX_SYNC_ERRORS: usize = 6;
/// Maximum payload length we will accept/transmit (LEN is one byte).
pub const MAX_PAYLOAD: usize = 255;

/// Physical-layer parameters shared by transmitter and receiver.
#[derive(Clone, Copy, Debug)]
pub struct ModemConfig {
    /// Complex sample rate (samples/second).
    pub sample_rate: f64,
    /// Chip rate (chips/second). Two chips per bit (Manchester).
    pub chip_rate: f64,
    /// Baseband tone offset from DC, in Hz. Kept away from DC to dodge LO leakage.
    pub tone_offset: f64,
    /// Tone amplitude in [0, 1]. Keep <= ~0.8 to avoid HackRF clipping.
    pub amplitude: f32,
}

impl Default for ModemConfig {
    fn default() -> Self {
        ModemConfig {
            sample_rate: 2_000_000.0,
            chip_rate: 20_000.0,
            tone_offset: 250_000.0,
            amplitude: 0.7,
        }
    }
}

impl ModemConfig {
    /// Samples per chip (may be fractional; the receiver tracks it as a float).
    pub fn sps(&self) -> f64 {
        self.sample_rate / self.chip_rate
    }
}

// ---------------------------------------------------------------------------
// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, xorout 0x0000)
// ---------------------------------------------------------------------------

/// Compute CRC-16/CCITT-FALSE over `data`.
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

// ---------------------------------------------------------------------------
// Framing: payload bytes -> on-the-wire byte frame (LEN | PAYLOAD | CRC16)
// ---------------------------------------------------------------------------

/// Build the framed bytes (everything after preamble+sync): `LEN | PAYLOAD | CRC16`.
///
/// The CRC covers `LEN | PAYLOAD`.
pub fn build_frame_bytes(payload: &[u8]) -> Vec<u8> {
    assert!(payload.len() <= MAX_PAYLOAD, "payload too long");
    let mut v = Vec::with_capacity(payload.len() + 3);
    v.push(payload.len() as u8);
    v.extend_from_slice(payload);
    let crc = crc16(&v);
    v.push((crc >> 8) as u8);
    v.push((crc & 0xFF) as u8);
    v
}

// ---------------------------------------------------------------------------
// Bit / Manchester helpers
// ---------------------------------------------------------------------------

/// Expand bytes to bits, MSB first.
fn bytes_to_bits(bytes: &[u8]) -> Vec<u8> {
    let mut bits = Vec::with_capacity(bytes.len() * 8);
    for &b in bytes {
        for i in (0..8).rev() {
            bits.push((b >> i) & 1);
        }
    }
    bits
}

/// Pack bits (MSB first) back into bytes. Trailing partial byte is dropped.
fn bits_to_bytes(bits: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(bits.len() / 8);
    for chunk in bits.chunks(8) {
        if chunk.len() < 8 {
            break;
        }
        let mut b = 0u8;
        for &bit in chunk {
            b = (b << 1) | (bit & 1);
        }
        out.push(b);
    }
    out
}

/// Manchester-encode bits to chips: `1 -> [1,0]`, `0 -> [0,1]`.
fn manchester_encode(bits: &[u8]) -> Vec<u8> {
    let mut chips = Vec::with_capacity(bits.len() * 2);
    for &b in bits {
        if b & 1 == 1 {
            chips.push(1);
            chips.push(0);
        } else {
            chips.push(0);
            chips.push(1);
        }
    }
    chips
}

/// Manchester-decode chip pairs back to bits. Returns `(bits, error_count)`.
/// A pair that is `[1,1]` or `[0,0]` is an error; we decode it by majority
/// fallback (`[1,1] -> 1`, `[0,0] -> 0`) but count it so callers can judge sync.
fn manchester_decode(chips: &[u8]) -> (Vec<u8>, usize) {
    let mut bits = Vec::with_capacity(chips.len() / 2);
    let mut errors = 0;
    for pair in chips.chunks(2) {
        if pair.len() < 2 {
            break;
        }
        match (pair[0], pair[1]) {
            (1, 0) => bits.push(1),
            (0, 1) => bits.push(0),
            (1, 1) => {
                bits.push(1);
                errors += 1;
            }
            _ => {
                bits.push(0);
                errors += 1;
            }
        }
    }
    (bits, errors)
}

/// The full chip sequence transmitted on the wire for `payload`:
/// `manchester(PREAMBLE | SYNC | LEN | PAYLOAD | CRC16)`.
pub fn frame_to_chips(payload: &[u8]) -> Vec<u8> {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(&PREAMBLE);
    bytes.extend_from_slice(&SYNC);
    bytes.extend_from_slice(&build_frame_bytes(payload));
    manchester_encode(&bytes_to_bits(&bytes))
}

/// The chip pattern the receiver correlates against to find a frame:
/// `manchester(tail-of-preamble | SYNC)`.
fn sync_chip_pattern() -> Vec<u8> {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(&PREAMBLE[PREAMBLE.len() - PREAMBLE_CORR_BYTES..]);
    bytes.extend_from_slice(&SYNC);
    manchester_encode(&bytes_to_bits(&bytes))
}

// ---------------------------------------------------------------------------
// Modulation: chips -> complex baseband samples (OOK on a tone)
// ---------------------------------------------------------------------------

/// Turn `payload` into complex baseband samples ready for the SDR sink.
///
/// `lead_chips` / `tail_chips` add silence (carrier off) before/after the burst
/// so the hardware has time to settle and nothing gets clipped at the edges.
pub fn modulate(cfg: &ModemConfig, payload: &[u8], lead_chips: usize, tail_chips: usize) -> Vec<Complex32> {
    let chips = frame_to_chips(payload);
    let sps = cfg.sps();
    let total_chips = lead_chips + chips.len() + tail_chips;
    let mut out = Vec::with_capacity((total_chips as f64 * sps) as usize + 1);

    // Continuous-phase NCO for the tone so there are no phase discontinuities.
    let w = 2.0 * std::f64::consts::PI * cfg.tone_offset / cfg.sample_rate;
    let mut phase = 0.0f64;
    let emit = |on: bool, n: usize, phase: &mut f64, out: &mut Vec<Complex32>| {
        for _ in 0..n {
            let s = if on {
                Complex32::new(
                    (cfg.amplitude as f64 * phase.cos()) as f32,
                    (cfg.amplitude as f64 * phase.sin()) as f32,
                )
            } else {
                Complex32::new(0.0, 0.0)
            };
            out.push(s);
            *phase += w;
        }
    };

    // Use a fractional-sample accumulator so non-integer sps doesn't drift.
    let mut acc = 0.0f64;
    let samples_for_chip = |acc: &mut f64| -> usize {
        *acc += sps;
        let n = acc.floor() as usize;
        *acc -= n as f64;
        n
    };

    for _ in 0..lead_chips {
        let n = samples_for_chip(&mut acc);
        emit(false, n, &mut phase, &mut out);
    }
    for &c in &chips {
        let n = samples_for_chip(&mut acc);
        emit(c == 1, n, &mut phase, &mut out);
    }
    for _ in 0..tail_chips {
        let n = samples_for_chip(&mut acc);
        emit(false, n, &mut phase, &mut out);
    }
    out
}

// ---------------------------------------------------------------------------
// Receiver front-end: DC removal + envelope
// ---------------------------------------------------------------------------

/// Leaky DC blocker producing the envelope `|x - dc|`. Removes the RTL-SDR's
/// large DC spike before slicing. Construct once and feed samples in order.
pub struct Envelope {
    dc: Complex32,
    alpha: f32,
}

impl Envelope {
    pub fn new() -> Self {
        // ~1e-3 leak: fast enough to track DC, slow enough to ignore the signal.
        Envelope { dc: Complex32::new(0.0, 0.0), alpha: 1e-3 }
    }
    pub fn push(&mut self, x: Complex32) -> f32 {
        self.dc += (x - self.dc) * self.alpha;
        (x - self.dc).norm()
    }
}

impl Default for Envelope {
    fn default() -> Self {
        Self::new()
    }
}

/// Tone-selective envelope detector. This is what makes over-the-air reception
/// work: instead of taking `|x|` over the whole 2 MHz band (where a narrow tone
/// drowns in wideband noise), it mixes the OOK tone down to baseband and boxcar
/// low-pass filters it first — rejecting out-of-band noise for ~15 dB of
/// processing gain — then returns the magnitude. Magnitude is taken last, so the
/// result is still completely immune to carrier frequency offset.
pub struct ToneDetector {
    w: f64,
    phase: f64,
    buf: std::collections::VecDeque<Complex32>,
    sum: Complex32,
    len: usize,
}

impl ToneDetector {
    pub fn new(cfg: &ModemConfig) -> Self {
        // Quarter-chip boxcar: passband comfortably wider than the chip rate +
        // expected CFO, while still rejecting most of the wideband noise.
        let len = ((cfg.sps() / 4.0).round() as usize).max(1);
        ToneDetector {
            w: 2.0 * std::f64::consts::PI * cfg.tone_offset / cfg.sample_rate,
            phase: 0.0,
            buf: std::collections::VecDeque::with_capacity(len),
            sum: Complex32::new(0.0, 0.0),
            len,
        }
    }

    pub fn push(&mut self, x: Complex32) -> f32 {
        // Mix the +tone_offset tone down toward DC: multiply by e^{-j w n}.
        let m = Complex32::new(self.phase.cos() as f32, -(self.phase.sin() as f32));
        let mixed = x * m;
        self.phase += self.w;
        if self.phase > std::f64::consts::TAU {
            self.phase -= std::f64::consts::TAU;
        }
        // Complex boxcar low-pass (running mean).
        self.sum += mixed;
        self.buf.push_back(mixed);
        if self.buf.len() > self.len {
            self.sum -= self.buf.pop_front().unwrap();
        }
        (self.sum / self.buf.len() as f32).norm()
    }
}

// ---------------------------------------------------------------------------
// Decoder: envelope samples -> recovered frames
// ---------------------------------------------------------------------------

/// Result of a successfully framed message.
#[derive(Clone, Debug)]
pub struct Decoded {
    pub payload: Vec<u8>,
    pub crc_ok: bool,
    /// Manchester chip-pair errors seen across LEN+PAYLOAD+CRC (sync-quality hint).
    pub manchester_errors: usize,
}

#[derive(Clone, Copy, PartialEq)]
enum State {
    Search,
    ReadLen,
    ReadPayload,
}

/// Streaming OOK/Manchester decoder. Feed envelope samples one at a time with
/// [`Decoder::push`]; it returns `Some(Decoded)` when a frame completes.
pub struct Decoder {
    sps: f64,
    pattern: Vec<u8>,

    // --- envelope smoothing (boxcar) for processing gain against noise ---
    smooth_buf: std::collections::VecDeque<f32>,
    smooth_sum: f32,
    smooth_win: usize,

    // --- amplitude tracking (AGC) for the on/off slicer ---
    hi: f32, // tracked "on" level
    lo: f32, // tracked "off" level
    last_bin: u8,

    // --- chip clock recovery (edge-resynced mid-chip sampler) ---
    to_sample: f64,
    primed: bool,

    // --- chip-stream sync correlation ---
    window: std::collections::VecDeque<u8>,

    // --- frame assembly ---
    state: State,
    chip_acc: Vec<u8>,
    need_chips: usize,
    payload_len: usize,
}

impl Decoder {
    pub fn new(cfg: &ModemConfig) -> Self {
        let pattern = sync_chip_pattern();
        let smooth_win = ((cfg.sps() / 4.0).round() as usize).max(1);
        Decoder {
            sps: cfg.sps(),
            window: std::collections::VecDeque::with_capacity(pattern.len()),
            pattern,
            smooth_buf: std::collections::VecDeque::with_capacity(smooth_win),
            smooth_sum: 0.0,
            smooth_win,
            hi: 0.0,
            lo: 0.0,
            last_bin: 0,
            to_sample: 0.0,
            primed: false,
            state: State::Search,
            chip_acc: Vec::new(),
            need_chips: 0,
            payload_len: 0,
        }
    }

    /// Slice one envelope sample into a 0/1 chip level with hysteresis + AGC.
    fn slice(&mut self, raw: f32) -> u8 {
        // Boxcar-smooth the envelope first: a chip lasts ~sps samples, so a
        // quarter-chip moving average rejects noise while preserving edges.
        self.smooth_sum += raw;
        self.smooth_buf.push_back(raw);
        if self.smooth_buf.len() > self.smooth_win {
            self.smooth_sum -= self.smooth_buf.pop_front().unwrap();
        }
        let env = self.smooth_sum / self.smooth_buf.len() as f32;

        // Peak/trough trackers: fast attack, slow release (release ~ a few chips).
        let release = (1.0 / (self.sps as f32 * 4.0)).min(0.5);
        if env > self.hi {
            self.hi += (env - self.hi) * 0.5;
        } else {
            self.hi += (env - self.hi) * release;
        }
        if env < self.lo {
            self.lo += (env - self.lo) * 0.5;
        } else {
            self.lo += (env - self.lo) * release;
        }

        let span = self.hi - self.lo;
        // Require some contrast before believing there's a signal at all.
        if span < 1e-4 {
            self.last_bin = 0;
            return 0;
        }
        let mid = (self.hi + self.lo) * 0.5;
        let band = span * 0.20; // hysteresis half-width
        if env > mid + band {
            self.last_bin = 1;
        } else if env < mid - band {
            self.last_bin = 0;
        }
        self.last_bin
    }

    /// Feed one envelope sample. Returns `Some(Decoded)` when a frame completes.
    pub fn push(&mut self, env: f32) -> Option<Decoded> {
        let prev = self.last_bin;
        let bin = self.slice(env);

        // Edge -> resync the chip clock so we sample at chip centers.
        if bin != prev {
            self.to_sample = self.sps * 0.5;
            self.primed = true;
        }

        if !self.primed {
            return None;
        }

        self.to_sample -= 1.0;
        if self.to_sample > 0.0 {
            return None;
        }
        // We are at a chip center: emit a chip and arm the next center.
        self.to_sample += self.sps;
        self.on_chip(bin)
    }

    /// Consume one recovered chip and advance the frame state machine.
    fn on_chip(&mut self, chip: u8) -> Option<Decoded> {
        match self.state {
            State::Search => {
                if self.window.len() == self.pattern.len() {
                    self.window.pop_front();
                }
                self.window.push_back(chip);
                if self.window.len() == self.pattern.len() {
                    let errors = self
                        .window
                        .iter()
                        .zip(self.pattern.iter())
                        .filter(|(a, b)| a != b)
                        .count();
                    if errors <= MAX_SYNC_ERRORS {
                        // Aligned! The next 16 chips are the LEN byte.
                        self.state = State::ReadLen;
                        self.chip_acc.clear();
                        self.need_chips = 16;
                        self.window.clear();
                    }
                }
                None
            }
            State::ReadLen => {
                self.chip_acc.push(chip);
                if self.chip_acc.len() >= self.need_chips {
                    let (bits, _) = manchester_decode(&self.chip_acc);
                    let len = bits_to_bytes(&bits).first().copied().unwrap_or(0) as usize;
                    self.payload_len = len;
                    self.chip_acc.clear();
                    // PAYLOAD + CRC16 = (len + 2) bytes -> *8 bits *2 chips.
                    self.need_chips = (len + 2) * 16;
                    self.state = State::ReadPayload;
                }
                None
            }
            State::ReadPayload => {
                self.chip_acc.push(chip);
                if self.chip_acc.len() >= self.need_chips {
                    let (bits, errors) = manchester_decode(&self.chip_acc);
                    let bytes = bits_to_bytes(&bits);
                    self.state = State::Search;
                    self.chip_acc.clear();

                    if bytes.len() < self.payload_len + 2 {
                        return None;
                    }
                    let payload = bytes[..self.payload_len].to_vec();
                    let rx_crc =
                        ((bytes[self.payload_len] as u16) << 8) | bytes[self.payload_len + 1] as u16;
                    // CRC was computed over LEN | PAYLOAD.
                    let mut check = Vec::with_capacity(self.payload_len + 1);
                    check.push(self.payload_len as u8);
                    check.extend_from_slice(&payload);
                    let crc_ok = crc16(&check) == rx_crc;
                    return Some(Decoded { payload, crc_ok, manchester_errors: errors });
                }
                None
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Software channel simulator (used by the loopback binary and tests)
// ---------------------------------------------------------------------------

/// Deterministic xorshift RNG -> standard-normal samples (Box-Muller), so tests
/// don't need an external rand crate.
pub struct Rng(u64);
impl Rng {
    pub fn new(seed: u64) -> Self {
        Rng(seed | 1)
    }
    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }
    fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn normal(&mut self) -> f64 {
        let u1 = self.unit().max(1e-12);
        let u2 = self.unit();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
}

/// Pass `samples` through a simulated channel: prepend `lead` noise samples,
/// apply a constant carrier frequency offset `cfo_hz` (which the envelope
/// detector must ignore), and add complex AWGN of std `noise_std`.
pub fn simulate_channel(
    cfg: &ModemConfig,
    samples: &[Complex32],
    cfo_hz: f64,
    noise_std: f32,
    lead: usize,
    rng: &mut Rng,
) -> Vec<Complex32> {
    let mut out = Vec::with_capacity(lead + samples.len());
    let w = 2.0 * std::f64::consts::PI * cfo_hz / cfg.sample_rate;
    let mut n = 0.0f64;
    let push_noisy = |s: Complex32, out: &mut Vec<Complex32>, rng: &mut Rng, n: &mut f64| {
        let c = Complex32::new((*n).cos() as f32, (*n).sin() as f32);
        let noise = Complex32::new(
            (rng.normal() as f32) * noise_std,
            (rng.normal() as f32) * noise_std,
        );
        out.push(s * c + noise);
        *n += w;
    };
    for _ in 0..lead {
        push_noisy(Complex32::new(0.0, 0.0), &mut out, rng, &mut n);
    }
    for &s in samples {
        push_noisy(s, &mut out, rng, &mut n);
    }
    out
}

/// Convenience: run a full encode -> channel -> decode pass and return the
/// first decoded frame, if any.
pub fn run_loopback(
    cfg: &ModemConfig,
    text: &str,
    cfo_hz: f64,
    noise_std: f32,
    lead: usize,
    seed: u64,
) -> Option<Decoded> {
    let tx = modulate(cfg, text.as_bytes(), 16, 16);
    let mut rng = Rng::new(seed);
    let rx = simulate_channel(cfg, &tx, cfo_hz, noise_std, lead, &mut rng);
    let mut det = ToneDetector::new(cfg);
    let mut dec = Decoder::new(cfg);
    for s in rx {
        if let Some(d) = dec.push(det.push(s)) {
            return Some(d);
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_known_vector() {
        // CRC-16/CCITT-FALSE of "123456789" is 0x29B1.
        assert_eq!(crc16(b"123456789"), 0x29B1);
    }

    #[test]
    fn manchester_roundtrip() {
        let bytes = [0x00u8, 0xFF, 0xA5, 0x2D, 0xD4];
        let bits = bytes_to_bits(&bytes);
        let chips = manchester_encode(&bits);
        let (back_bits, errors) = manchester_decode(&chips);
        assert_eq!(errors, 0);
        assert_eq!(bits_to_bytes(&back_bits), bytes);
    }

    #[test]
    fn loopback_clean() {
        let cfg = ModemConfig::default();
        let d = run_loopback(&cfg, "HELLO WORLD!", 0.0, 0.0, 5000, 1).expect("decoded");
        assert!(d.crc_ok);
        assert_eq!(d.payload, b"HELLO WORLD!");
    }

    #[test]
    fn loopback_cfo_noise_offset() {
        let cfg = ModemConfig::default();
        // 18 kHz CFO (RTL-SDR-ish), moderate noise, odd start offset.
        let d = run_loopback(&cfg, "the quick brown fox", 18_000.0, 0.05, 7333, 42)
            .expect("decoded");
        assert!(d.crc_ok, "crc failed, manchester_errors={}", d.manchester_errors);
        assert_eq!(d.payload, b"the quick brown fox");
    }

    #[test]
    fn loopback_empty_and_max_ish() {
        let cfg = ModemConfig::default();
        let d = run_loopback(&cfg, "", 5000.0, 0.02, 1234, 9).expect("decoded");
        assert!(d.crc_ok);
        assert_eq!(d.payload, b"");
    }
}
