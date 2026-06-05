//! Receive text messages with an **RTL-SDR**, decoding the OOK/Manchester
//! framing transmitted by `tx`. Runs until you Ctrl-C it, printing each frame.
//!
//!   cargo run --release --bin rx -- --freq 433.9e6 --gain 30
//!
//! The RTL-SDR and HackRF do not share a clock, so there is a carrier frequency
//! offset between them. This receiver decodes from the signal *envelope*, which
//! is immune to that offset.

use anyhow::Result;
use clap::Parser;
use futuresdr::blocks::Apply;
use futuresdr::blocks::NullSink;
use futuresdr::blocks::seify::Builder;
use futuresdr::num_complex::Complex32;
use futuresdr::prelude::*;
use rxtx::{Decoder, ModemConfig, ToneDetector};

#[derive(Parser, Debug)]
#[command(about = "RTL-SDR OOK/Manchester text receiver")]
struct Args {
    /// Center frequency in Hz (must match the transmitter).
    #[arg(short, long, default_value_t = 433.9e6)]
    freq: f64,
    /// Sample rate in Hz (must match the transmitter).
    #[arg(short, long, default_value_t = 2_000_000.0)]
    sample_rate: f64,
    /// RX gain in dB. Try 20-40; too high overloads on a near transmitter.
    #[arg(short, long, default_value_t = 30.0)]
    gain: f64,
    /// seify device args. We go through SoapySDR (`driver=soapy`) and select the
    /// RTL-SDR with `soapy_driver=rtlsdr`; append e.g. ", serial=00000001" to pin one.
    #[arg(long, default_value = "driver=soapy, soapy_driver=rtlsdr")]
    args: String,
    /// Print a periodic signal-strength meter (avg/peak envelope) for debugging.
    #[arg(long, default_value_t = true)]
    meter: bool,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let cfg = ModemConfig {
        sample_rate: args.sample_rate,
        ..ModemConfig::default()
    };

    println!(
        "RX listening on {:.4} MHz @ {} S/s, gain {} (Ctrl-C to stop)",
        args.freq / 1e6,
        cfg.sample_rate,
        args.gain
    );

    let mut fg = Flowgraph::new();

    let src = Builder::new(&args.args)?
        .frequency(args.freq)
        .sample_rate(cfg.sample_rate)
        .gain(args.gain)
        .build_source()?;

    // 1) Tone-selective envelope: mix the +250 kHz tone to baseband, low-pass to
    //    reject wideband noise (~15 dB processing gain), then take magnitude.
    let mut det = ToneDetector::new(&cfg);
    let envelope = Apply::new(move |x: &Complex32| det.push(*x));

    // 2) Stateful decoder. Prints whenever a frame completes.
    let mut dec = Decoder::new(&cfg);
    let mut count: u64 = 0;
    // Periodic signal meter: avg + peak envelope over each ~0.5 s window. Lets you
    // see whether the transmitter's energy is even reaching the receiver.
    let meter_on = args.meter;
    let meter_interval = (cfg.sample_rate / 2.0) as u64;
    let mut m_n: u64 = 0;
    let mut m_sum: f64 = 0.0;
    let mut m_peak: f32 = 0.0;
    let decode = Apply::new(move |e: &f32| -> f32 {
        if meter_on {
            m_n += 1;
            m_sum += *e as f64;
            if *e > m_peak {
                m_peak = *e;
            }
            if m_n >= meter_interval {
                let avg = m_sum / m_n as f64;
                println!("  [meter] avg={:.4} peak={:.4}", avg, m_peak);
                m_n = 0;
                m_sum = 0.0;
                m_peak = 0.0;
            }
        }
        if let Some(d) = dec.push(*e) {
            count += 1;
            let text = String::from_utf8_lossy(&d.payload);
            if d.crc_ok {
                println!("[#{count}] OK  ({} bytes): {:?}", d.payload.len(), text);
            } else {
                println!(
                    "[#{count}] CRC FAIL (manchester_errors={}): {:?}",
                    d.manchester_errors, text
                );
            }
        }
        *e
    });

    let snk = NullSink::<f32>::new();
    connect!(fg, src.outputs[0] > envelope > decode > snk);

    Runtime::new().run(fg)?;
    Ok(())
}
