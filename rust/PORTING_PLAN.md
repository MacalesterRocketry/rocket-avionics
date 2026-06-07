# Rust + Embassy port — porting plan

think about https://gemini.google.com/share/bd9823a12fad

The existing C++/Arduino firmware in the repo root (~2,600 LOC, last flew
2026-05-17) is the source of truth. This subdirectory holds the in-progress
Rust port that will eventually replace it on the new RP2350 PCB. The C++ keeps
flying until the Rust port has been bench-validated and ground-tested.

This document is the contract for the rest of the port: it freezes the
architectural choices, names the driver crates, lays out the multicore split,
and defines the validation gates that must pass before flight.

---

## 1. Why Embassy (not Zephyr, not RTIC)

The original request was "multicore Zephyr." After looking at the C++ code,
Embassy is a better fit:

- **Zephyr's Rust support is experimental.** Most Zephyr APIs require `unsafe`
  shims; the Rust portion of Zephyr's tree covers ~10% of the kernel. A
  rocket-avionics firmware would spend more time on FFI glue than on AHRS work.
- **Embassy is async + multicore on RP2350 *today*.** `embassy-rp` 0.4+
  supports `rp235xa`/`rp235xb` with `multicore::spawn_core1`, and the
  ecosystem of sensor driver crates is the largest in embedded Rust.
- **RTIC was a strong contender** (deterministic, priority-based, no executor
  overhead). Rejected because the C++ code is structurally a cooperative
  super-loop — Embassy's `async` tasks map cleanly onto it, while RTIC would
  require restructuring everything around hardware-interrupt priorities. We can
  revisit RTIC later if Embassy task latency turns out to be a problem under
  the AHRS load. (Profiling target: <1 ms 99p loop time on core 0.)

---

## 2. Multicore architecture

```
 ┌─────────────────────── Core 0 (avionics) ──────────────────────────┐
 │  Embassy executor, 200 Hz Ticker                                   │
 │  ┌──────────────────────────────────────────────────────────────┐  │
 │  │ sample_task:                                                 │  │
 │  │   read_all_sensors() ──▶ ahrs::update() ──▶ state machine ──▶│  │
 │  │   roll_controller::step() ──▶ servo PWM writes               │  │
 │  └──────────────────────────────────────────────────────────────┘  │
 │      │                                                             │
 │      │   log packets (mpmc channel, ~256 deep)                     │
 │      ▼                                                             │
 └──────┼────────────────────────────────────────────────────────────-┘
        │ FIFO between cores (Embassy `Channel`, lock-free SPSC)
 ┌──────▼─────────────────── Core 1 (I/O) ─────────────────────────┐
 │   sd_writer_task: pop LogEntry → serialize → write_all().await │
 │   gps_task:       UART RX → nmea::parse → push PayloadGps      │
 │   indicator_task: state-driven NeoPixel + buzzer cadence       │
 │   supervisor_task: eject-button watch, low-battery watch       │
 └────────────────────────────────────────────────────────────────┘
```

**Why this split**: in the C++ source, `dataFile.sync()` blocks for up to
30 ms whenever it fires (every 5 s by default). That's a 30 ms hole in AHRS
sampling — explicitly TODO'd in `states.cpp` as "put it on another core."
Moving SD I/O to core 1 makes the AHRS loop genuinely real-time, with no
single core 0 op longer than the worst-case I²C burst read (≈200 µs at 400 kHz).

**Shared state**:

| Direction         | Vehicle                                    | Mechanism                            |
| ----------------- | ------------------------------------------ | ------------------------------------ |
| Core 0 → Core 1   | Log packets                                | `embassy_sync::channel::Channel` (SPSC, fixed-size, no alloc) |
| Core 1 → Core 0   | Latest GPS fix, "has_fix" flag             | `embassy_sync::mutex::Mutex<GpsFix>` |
| Core 0 ↔ Core 0   | AHRS state read by control task            | `embassy_sync::mutex::Mutex<AhrsState>` |
| Both → Core 1     | System state (for indicator policy)        | `embassy_sync::signal::Signal<SystemState>` |

All locks are `CriticalSectionRawMutex` (works across cores). For hot
single-word reads we use `portable_atomic::AtomicU64` where it fits.

---

## 3. Per-module port table

| C++ source                  | Rust target                          | Status      | Notes                                                                                  |
| --------------------------- | ------------------------------------ | ----------- | -------------------------------------------------------------------------------------- |
| `config.h`                  | `src/config.rs`                      | **Ported**  | Constants are `pub const`; pin assignments are `cfg(feature)`-gated for hw rev.        |
| `utils.h` types             | `src/types.rs`                       | **Ported**  | `SensorReadings`, `SystemState`, `EventType`, `ErrorFlags` (bitflags crate).           |
| `utils.h` Vec3/Quat + math  | `src/math.rs`                        | **Ported**  | f64 to preserve behavior. Host tests for Hamilton product + axis-angle.                |
| `utils.cpp`                 | `src/math.rs`                        | **Ported**  | Folded into math module since it's just impls of types declared in utils.h.            |
| `states.cpp`                | `src/state.rs`                       | Skeleton    | State machine + indicator policy. Replaces super-loop with Embassy task + Signal.      |
| `orientation/sensors.cpp`   | `src/sensors/{mod,lsm6dsox,...}.rs`  | Skeleton    | One submodule per chip; shared-bus I²C via `embassy-embedded-hal`.                     |
| `orientation/ahrs.cpp`      | `src/ahrs.rs`                        | Skeleton    | Pure-math layer + `AhrsState` struct behind a mutex. Madgwick gradients ported next.   |
| `orientation/gps.cpp`       | `src/orientation/gps.rs`             | Skeleton    | UART1 + `nmea` crate. Replaces the 1 kHz `repeating_timer` ISR with an async loop.     |
| `output/sdcard.cpp`         | `src/output/sdcard.rs`               | Skeleton    | `embedded-sdmmc` + SPI1. Channel-fed writer task on core 1.                            |
| `output/servo.cpp`          | `src/output/servo.rs`                | Skeleton    | `embassy_rp::pwm` instead of Arduino Servo lib. Pure math ported; PWM I/O is TODO.     |
| `output/roll-controller.cpp`| `src/output/roll_controller.rs`      | Skeleton    | PID lives in `RollPid` struct (no statics → trivially testable + thread-safe).         |
| `decoder.py`                | unchanged                            | n/a         | Binary log format is preserved bit-for-bit; existing decoder keeps working.            |
| `lib/BMP3XX/`               | `bmp388` crate                       | n/a         | Same registers as BMP388; no need to port vendor C.                                    |

Binary log format (the `logXX.bin` file structure) is **the immovable
constraint** of this port — the team's analysis tooling, post-flight Foxglove
viewer, and historic flight logs all depend on `decoder.py` continuing to
work. Every `Payload*` struct in `src/log_packets.rs` has a `const _: () =
assert!(SIZE == ...)` check tying it to the corresponding format string in
`decoder.py`, so any drift is caught at compile time.

---

## 4. Driver crate decisions

| Sensor    | Crate              | Confidence | Fallback                                                  |
| --------- | ------------------ | ---------- | --------------------------------------------------------- |
| LSM6DSOX  | `lsm6dsox` 0.5     | Medium     | Write a ~200-line register-level driver (well-documented) |
| LIS3MDL   | `lis3mdl` 0.2      | Low        | Bench-test crate first; trivial to replace if API rough   |
| ADXL375   | *(none mature)*    | n/a        | **Write our own.** ADXL343 superset; ~100 lines.          |
| BMP390    | `bmp388` 0.3       | High       | BMP388/BMP390 registers identical                         |
| MTK3333   | `nmea` 0.6 (parser)| High       | Hand-rolled parser if `nmea` pulls in too much heap usage |
| SD card   | `embedded-sdmmc` 0.8 | High     | Used in many flying-firmware projects                     |

The `sensors` Cargo feature gates the optional driver crates so early
porting can `cargo check` with just `embassy-rp` and `embedded-hal` in place.

---

## 5. Performance + numerics

The current code uses `double` (f64) throughout the AHRS, on a Cortex-M0+ with
**no FPU at all**. On RP2350 (Cortex-M33 + FPv5-SP, single-precision hardware
FPU), keeping f64 means soft-float math but with a real ALU — so the AHRS will
already be 2–5× faster without any other changes.

**Option (call out, do not change yet)**: convert AHRS math to f32 for ~10×
additional speedup. Madgwick converges fine at f32; the only place f64 matters
is the position double-integration (gyro→accel→velocity→position drifts
quadratically with float error). Defer this decision to after first bench
runs — we want apples-to-apples comparison with the C++ flight logs first.

**Hot-path budget** (200 Hz sample rate, 5 ms loop period):

| Stage                | Target time | Notes                                    |
| -------------------- | ----------- | ---------------------------------------- |
| I²C burst read × 4   | <800 µs     | LSM 14 B, LIS3 6 B, ADXL 6 B, BMP 6 B    |
| AHRS update          | <300 µs     | Was ~2 ms on RP2040 per existing PROFILING comments |
| State + indicators   | <50 µs      |                                          |
| Roll PID + servo PWM | <100 µs     | PWM is just a compare-register write     |
| Channel push (log)   | <10 µs      | Lock-free SPSC                           |
| **Total budget**     | **<1.5 ms** | Leaves 70% headroom for jitter           |

---

## 6. Validation phases

**Phase A — Host tests.** Pure-math modules (`math`, `log_packets`, parts of
`output::servo`, `output::roll_controller::effectiveness`) compile and pass
unit tests via `cargo hosttest` (alias defined in `.cargo/config.toml`). One
test per Hamilton product, axis-angle, and packet layout. **Status: scaffolded;
needs runs.**

**Phase B — Embedded smoke.** Build for `thumbv8m.main-none-eabihf`, flash to
a bare RP2350 dev board, confirm both cores boot and the defmt log shows
heartbeats from each. No sensors yet.

**Phase C — Per-sensor bench.** One sensor at a time: read it for 30 s, dump
to RTT, compare value distribution against the C++ output on the same bench.
Catches axis-orientation mistakes (the LSM has commented-out axis swaps in the
C++ code — verify whichever convention is currently flying).

**Phase D — AHRS replay.** Feed a known-good `log00.bin` from the 2026-05-17
flight back through the Rust AHRS (via a host-side wrapper that mocks the
sensor reads). Output orientation/velocity/position should match the original
to within 1° / 0.1 m/s / 1 m for the duration of the flight. *This is the
single most important validation step* — if the AHRS doesn't replay cleanly,
the rocket flies blind.

**Phase E — Hardware loop.** All four sensors live, SD logging on core 1, AHRS
on core 0. Vibration table test if available; otherwise hand-shake test that
the AHRS doesn't NaN-out under abuse.

**Phase F — Ground-test fire.** With motor in test stand, full firmware run.
Compare logs to C++ version on identical setup.

**Phase G — Flight.** Only after Phase F passes cleanly, with a Phase-A-pass
diff between Rust and C++ logs sitting in git for review.

The C++ firmware continues to fly until Phase G clears.

---

## 7. Open questions for the team

1. **Hardware revision**: the RP2350 PCB pinout in `src/config.rs` under
   `#[cfg(feature = "hw-v3")]` is placeholder. Drop the real GPIO assignments
   in once the schematic is finalized.
2. **GPS rate**: C++ runs the GPS at 5 Hz. Worth re-evaluating now that we
   have an unblocked core 1 — 10 Hz fixes are possible on the MTK3333 with
   firmware tweaks but lock quality drops.
3. **Magnetometer β**: currently 0.0 in flight config (i.e. mag is logged but
   not fused). Plan to keep at 0 for the port to preserve flight behavior;
   tune up only after the team has time for outdoor heading-truth comparisons.
4. **RP2350 RISC-V (Hazard-3) cores**: RP2350 has two Cortex-M33 + two Hazard-3
   RISC-V cores selectable at boot. Embassy supports the M33 cores only. Out
   of scope unless the team has a reason to investigate.

---

## 8. Build + flash

```bash
# from rust/ directory
cargo check                                # firmware target
cargo hosttest                             # host-side math + log tests
cargo build --release --features hw-v3     # firmware for new PCB
cargo run   --release --features hw-v3     # flash via probe-rs (needs Picoprobe)
```

UF2 path (no debug probe): `cargo build --release` then run
`picotool load -u -v -x -t elf target/thumbv8m.main-none-eabihf/release/rocket-avionics`.

---

## 9. What's NOT in this scaffold

These are deliberately deferred to keep this PR reviewable:

- Actual Madgwick gradient implementations (signatures in place, bodies stubbed)
- Real driver code for any sensor (modules in place, contents are doc-comments)
- SD writer task body (channel type + writer signature defined)
- GPS UART task (module in place, no code)
- Servo PWM channel allocation table (math ported; PWM init pending HW rev)

Each one will land as its own follow-up once this scaffold is reviewed.
