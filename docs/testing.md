# Testing and scope

Part of [mavlink-device](../README.md)

Host tests check the node against fakes and against the `mavlink` 0.21.0 crate; a demonstration program on the board (`hil/`) checks
it against the ground station.

## Host

`cargo test` runs 130 unit and property tests and 1 doc test; with `--features _test-hooks` 133 and 1 (adds the negative controls
of the slowdown test and of the command-hook rule). Every frame the node emits in a test is also decoded by the `mavlink` 0.21.0
reader (dev-dependency); input frames are made by that crate.

| File | Covers |
|---|---|
| `src/tests.rs` | Framing, heartbeat, streams, texts |
| `src/service_tests.rs` | Parameters and commands |
| `src/command_tests.rs` | 15 tests of the `command`, `command_deferrable`, `acks` and `custom_mode` hooks and of `gcs_seen_us` |
| `src/accept_tests.rs` | The accept list, the hand-over and the slowdown |
| `src/run_tests.rs` | `run` against a fake reader, writer and delay on a made-up clock |
| `src/e2e_tests.rs` | A ground-station model over a link that loses 20 % of the frames each way |
| `src/fuzz_tests.rs` | Property tests: arbitrary bytes in arbitrary splits (no panic, buffer bounded); valid frames in arbitrary splits with noise between; a real frame behind a false start; arbitrary use of the whole node |

## Target

```text
cargo build --target thumbv8m.main-none-eabihf
cargo tree -e normal --target thumbv8m.main-none-eabihf | grep -cE 'mavlink-bindgen|quick-xml|regex|mavlink v|usb|serialport|embassy'   # prints 0
```

## Features

| Feature | Default | Effect |
|---|---|---|
| `defmt` | off | `defmt::Format` on public types; enables `embedded-io-async/defmt` |
| `_test-hooks` | off | Test-only: `Node::set_slowdown_off`, `Node::receive_calling_command_hook`. Not part of the API |

## Not covered

- Windows and macOS: everything measured ran against one Linux computer (`cdc_acm`, the `serialport` 4.x crate). How another host's
  terminal layer treats the first bytes after an open is not known; the start gate is built from the Linux measurement.
- A USB-serial adapter on the computer: its cooked window is the same kind and the vehicle cannot see it; the ground station must
  resynchronise at the start of any session. Not run.
- Long runs: the longest session on the board is 20 s with 5 s recordings; a wrapping counter, clock drift or a leak is not
  excluded (the sequence number wraps every 256 frames and did so many times).
- Real sensors: the demonstration computes every value from the clock and keeps parameters in RAM. Load, timing and jitter with real
  drivers on the same executor are not measured; throughput at the fastest rates is unmeasured (only default rates ran).
- Signalling-NaN payloads in `I32` parameters rely on the platform; only checked on x86-64.
- On the board with a real radio: the test hook that switches the slowdown off and RC_CHANNELS_OVERRIDE through the hook were not
  exercised; the rest of the accept list, hand-over and slowdown was checked on the host only, with frames made by the crate's tests
  and the `marv-dialect` types.
- By design: signing, MAVLink 1, PARAM_EXT, PARAM_ERROR, COMMAND_INT, missions, persistence of parameters, parameter types other
  than INT32 and REAL32.
- A very long write (a host that stops reading) blocks the link task; the node does not time it out.

## Code

`src/*_tests.rs`, `src/tests.rs`.

## Related

[node](node.md).
