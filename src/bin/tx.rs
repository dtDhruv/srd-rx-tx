//! Transmit a text message with a **HackRF** using OOK/Manchester framing.
//!
//!   cargo run --release --bin tx -- --message "hello world" --freq 433.9e6 --gain 20
//!
//! LEGAL: you are responsible for what you radiate. Only transmit on frequencies
//! and at power levels you are licensed/permitted to use. For bench testing use a
//! shielded RF cable + attenuator (or a dummy load), low gain, and ideally an ISM
//! band. 433.92 MHz is ISM in much of the world but rules vary — check yours.

use anyhow::Result;
use clap::Parser;
use futuresdr::blocks::VectorSource;
use futuresdr::blocks::seify::Builder;
use futuresdr::num_complex::Complex32;
use futuresdr::prelude::*;
use rxtx::{ModemConfig, modulate};

#[derive(Parser, Debug)]
#[command(about = "HackRF OOK/Manchester text transmitter")]
struct Args {
    /// Message to transmit.
    #[arg(short, long, default_value = "hello from hackrf")]
    message: String,
    /// Center frequency in Hz.
    #[arg(short, long, default_value_t = 433.9e6)]
    freq: f64,
    /// Sample rate in Hz (HackRF minimum is 2e6).
    #[arg(short, long, default_value_t = 2_000_000.0)]
    sample_rate: f64,
    /// TX gain in dB. HackRF TX VGA is 0-47; use 0 for minimum range.
    #[arg(short, long, default_value_t = 20.0)]
    gain: f64,
    /// Digital tone amplitude in [0,1]. Lower = less drive = shorter range.
    #[arg(short, long, default_value_t = 0.7)]
    amplitude: f32,
    /// Times to repeat the frame so the receiver has several chances to catch it.
    #[arg(short, long, default_value_t = 10)]
    repeat: usize,
    /// seify device args. We go through SoapySDR (`driver=soapy`) and select the
    /// HackRF with `soapy_driver=hackrf`; append e.g. ", serial=0000..." to pin one.
    #[arg(long, default_value = "driver=soapy, soapy_driver=hackrf")]
    args: String,
    /// TEST MODE: instead of a message, transmit a continuous carrier tone for
    /// this many seconds (0 = off). Use it to verify the link with the RX meter.
    #[arg(long, default_value_t = 0.0)]
    carrier: f64,
}

fn main() -> Result<()> {
    let args = Args::parse();
    if args.message.len() > rxtx::MAX_PAYLOAD {
        anyhow::bail!("message too long (max {} bytes)", rxtx::MAX_PAYLOAD);
    }

    if !(0.0..=1.0).contains(&args.amplitude) {
        anyhow::bail!("amplitude must be in [0, 1]");
    }
    let cfg = ModemConfig {
        sample_rate: args.sample_rate,
        amplitude: args.amplitude,
        ..ModemConfig::default()
    };

    let samples: Vec<Complex32> = if args.carrier > 0.0 {
        // TEST MODE: a steady tone at +tone_offset for `carrier` seconds.
        let n = (cfg.sample_rate * args.carrier) as usize;
        let w = 2.0 * std::f64::consts::PI * cfg.tone_offset / cfg.sample_rate;
        let mut v = Vec::with_capacity(n);
        let mut phase = 0.0f64;
        for _ in 0..n {
            v.push(Complex32::new(
                (cfg.amplitude as f64 * phase.cos()) as f32,
                (cfg.amplitude as f64 * phase.sin()) as f32,
            ));
            phase += w;
        }
        println!(
            "TX CARRIER for {:.1}s on {:.4} MHz @ {} S/s, gain {}, amplitude {}, tone +{} Hz",
            args.carrier, args.freq / 1e6, cfg.sample_rate, args.gain, cfg.amplitude, cfg.tone_offset
        );
        v
    } else {
        // Build one burst with generous lead/tail silence, then repeat it. The
        // silence between bursts doubles as the inter-frame gap.
        let lead = 200; // chips of carrier-off before the preamble (AGC settle)
        let tail = 400; // chips of carrier-off after the CRC (gap + flush)
        let one = modulate(&cfg, args.message.as_bytes(), lead, tail);
        let mut samples: Vec<Complex32> = Vec::with_capacity(one.len() * args.repeat);
        for _ in 0..args.repeat {
            samples.extend_from_slice(&one);
        }
        // A final chunk of zeros to make sure the USB buffer is fully drained.
        samples.extend(std::iter::repeat_n(Complex32::new(0.0, 0.0), cfg.sample_rate as usize / 10));

        let burst_secs = one.len() as f64 / cfg.sample_rate;
        println!(
            "TX {:?}  ({} bytes) on {:.4} MHz @ {} S/s, gain {}, amplitude {}",
            args.message, args.message.len(), args.freq / 1e6, cfg.sample_rate, args.gain, cfg.amplitude
        );
        println!(
            "  {} samples/burst (~{:.1} ms), repeating {}x, tone +{} Hz",
            one.len(), burst_secs * 1e3, args.repeat, cfg.tone_offset
        );
        samples
    };

    let mut fg = Flowgraph::new();
    let src = fg.add(VectorSource::<Complex32>::new(samples));
    let snk = fg.add(
        Builder::new(&args.args)?
            .frequency(args.freq)
            .sample_rate(cfg.sample_rate)
            .gain(args.gain)
            .build_sink()?,
    );
    // The seify sink uses indexed channel ports (`inputs[0]`), so wire it explicitly.
    fg.stream_dyn(src, "output", snk, "inputs[0]")?;

    Runtime::new().run(fg)?;
    println!("done");
    Ok(())
}
