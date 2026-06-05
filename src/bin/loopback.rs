//! Pure-software self test: modulate a message, push it through a simulated
//! channel (carrier frequency offset + noise + random start delay), then
//! demodulate. Proves the DSP works with no radios attached.
//!
//!   cargo run --release --bin loopback -- --message "hello from rust sdr"

use anyhow::Result;
use clap::Parser;
use rxtx::{ModemConfig, run_loopback};

#[derive(Parser, Debug)]
#[command(about = "Software loopback self-test for the OOK/Manchester text modem")]
struct Args {
    /// Message to send through the simulated channel.
    #[arg(short, long, default_value = "hello from rust sdr")]
    message: String,
    /// Carrier frequency offset to inject (Hz). Envelope detection should ignore it.
    #[arg(long, default_value_t = 18_000.0)]
    cfo: f64,
    /// Complex AWGN standard deviation.
    #[arg(long, default_value_t = 0.05)]
    noise: f32,
    /// Number of noise-only samples before the burst (random start offset).
    #[arg(long, default_value_t = 5000)]
    lead: usize,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let cfg = ModemConfig::default();

    println!(
        "modem: {} S/s, {} chips/s, {:.1} samples/chip, tone +{} Hz",
        cfg.sample_rate,
        cfg.chip_rate,
        cfg.sps(),
        cfg.tone_offset
    );
    println!(
        "channel: cfo={} Hz, noise_std={}, lead={} samples",
        args.cfo, args.noise, args.lead
    );
    println!("TX: {:?}", args.message);

    match run_loopback(&cfg, &args.message, args.cfo, args.noise, args.lead, 1) {
        Some(d) => {
            let text = String::from_utf8_lossy(&d.payload);
            println!(
                "RX: {:?}  (crc_ok={}, manchester_errors={})",
                text, d.crc_ok, d.manchester_errors
            );
            if d.crc_ok && text == args.message {
                println!("OK: round-trip succeeded");
                Ok(())
            } else {
                anyhow::bail!("mismatch or bad CRC");
            }
        }
        None => anyhow::bail!("no frame decoded"),
    }
}
