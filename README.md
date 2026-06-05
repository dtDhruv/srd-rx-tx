# rx-tx — a tiny text modem in Rust + FutureSDR

Send a **text message over the air**: **HackRF transmits**, **RTL-SDR receives**.

Because the two radios run on independent clocks, there is a carrier frequency
offset (CFO) between them of potentially tens of kHz, and the RTL-SDR can only
receive. The modulation is built around that constraint:

| Layer        | Choice                | Why                                                        |
|--------------|-----------------------|------------------------------------------------------------|
| Modulation   | **OOK** (tone on/off) | Decoded from the envelope `\|sample\|` → **immune to CFO**  |
| Line code    | **Manchester**        | Guarantees transitions → clock recovery + ~50% duty cycle  |
| Framing      | preamble · SYNC · LEN · PAYLOAD · CRC16 | Frame sync by correlating the recovered chip stream |

```
TX (HackRF):  text → [0xAA×8 | SYNC | LEN | PAYLOAD | CRC16] → bits → Manchester
              → OOK upsample → × tone(+250 kHz) → HackRF @ 433.9 MHz
RX (RTL-SDR): RTL-SDR @ 433.9 MHz → DC-block → |·| envelope → AGC slicer
              → chip-clock recovery → sync correlate → Manchester decode → CRC → text
```

## Binaries

| Binary     | What it does                                                            |
|------------|-------------------------------------------------------------------------|
| `loopback` | Pure-software self test: modulate → simulated channel (CFO+noise+delay) → demodulate. **No hardware.** |
| `tx`       | Transmit a message with a HackRF.                                       |
| `rx`       | Receive + print messages with an RTL-SDR (runs until Ctrl-C).           |

## Prerequisites

- Rust **nightly** (pinned via `rust-toolchain.toml`; `rustup` installs it automatically).
- **SoapySDR + dev files**, plus the HackRF and RTL-SDR Soapy modules:
  ```sh
  sudo apt install libsoapysdr-dev soapysdr-module-hackrf soapysdr-module-rtlsdr
  ```
  We use the SoapySDR backend (not seify's native drivers) because seify 0.18's
  native HackRF driver is **RX-only** — its transmit path is unimplemented.
  Devices are selected through Soapy: `--args "driver=soapy, soapy_driver=hackrf"`
  (TX) and `--args "driver=soapy, soapy_driver=rtlsdr"` (RX) — these are the defaults.

### USB permissions (Linux)

If you get a permission error opening a device, add udev rules and join `plugdev`:

```sh
# HackRF (Great Scott Gadgets) + RTL-SDR (Realtek)
sudo tee /etc/udev/rules.d/53-sdr.rules >/dev/null <<'EOF'
ATTR{idVendor}=="1d50", ATTR{idProduct}=="6089", MODE="0660", GROUP="plugdev"
ATTR{idVendor}=="0bda", ATTR{idProduct}=="2838", MODE="0660", GROUP="plugdev"
ATTR{idVendor}=="0bda", ATTR{idProduct}=="2832", MODE="0660", GROUP="plugdev"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG plugdev "$USER"   # then log out/in
```

If `rtl-sdr` is loaded as a TV tuner, blacklist the kernel module:
`echo 'blacklist dvb_usb_rtl28xxu' | sudo tee /etc/modprobe.d/blacklist-rtl.conf`

## Quick start

**1. Prove the DSP with no hardware:**
```sh
cargo run --release --bin loopback -- --message "hello from rust sdr"
# also try a brutal channel (large frequency offset + heavy noise):
cargo run --release --bin loopback -- --message "test" --cfo 40000 --noise 0.25
```

**2. On the air.** Connect HackRF and RTL-SDR. Best/safest: join their antenna
ports with a coax cable and an **attenuator** (e.g. 30–40 dB), or use small
antennas a short distance apart. Start the receiver first:
```sh
# terminal 1 — receiver
cargo run --release --bin rx -- --freq 433.9e6 --gain 30

# terminal 2 — transmitter
cargo run --release --bin tx -- --message "hello world" --freq 433.9e6 --gain 20 --repeat 10
```
The receiver prints e.g. `[#1] OK  (11 bytes): "hello world"`.

### Tuning tips
- **Frequency** (`--freq`) must match on both sides. Both radios cover 433.9 MHz.
- **RX gain** (`--gain`): start ~30 dB. If the transmitter is close it can
  overload — *lower* RX gain. If nothing decodes, raise it.
- **TX gain** (`--gain`): keep low (≈10–20) for bench tests; raise if the link is weak.
- **Sample rate** must match (default 2 MS/s — HackRF's minimum, fine for RTL-SDR).
- **Repeat** (`--repeat`): the frame is sent N times so the receiver has several
  chances; increase it for a flaky link.

## ⚠️ Legal

You are responsible for everything you radiate. Only transmit on frequencies and
at power levels you are licensed/permitted to use in your jurisdiction. For
testing, prefer a **cabled** setup with an attenuator or dummy load and the
lowest gain that works. 433.92 MHz is an ISM band in much of the world, but rules
vary — check yours.

## How it fits together

- `src/lib.rs` — the whole modem (CRC-16, Manchester, OOK modulator, envelope +
  AGC + chip-clock-recovery decoder) plus a software channel simulator. Unit
  tested (`cargo test`).
- `src/bin/{tx,rx,loopback}.rs` — thin FutureSDR flowgraphs around the library.

Built against FutureSDR `0.0.42-dev` (pinned git rev in `Cargo.toml`).
