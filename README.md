# mavlink-device

MAVLink 2 device layer for the MARV V2 flight controller. The crate is `no_std` and runs on any byte stream: it knows nothing about USB, a transport or an executor ("usb lives alone, mavlink is a layer on top of it"). `Node` is a sans-IO core: bytes and a time in, frames out. Messages come from [`marv-dialect`](https://github.com/Lmx154/marv-dialect) (a git dependency pinned to a commit in `Cargo.toml`), re-exported as `mavlink_device::dialect`. The only other dependencies are `embedded-io-async` 0.7 and `embedded-hal-async` 1.0 (for the async `Node::run`) and, optionally, `defmt`.

**Status: version 0.1.0 (`publish = false`). The API may still change.** The accept list, the hand-over hook and the RADIO_STATUS slowdown (sections below) are new on branch `wip-accept-list`: **nothing of them has run against a real radio or on the board; only host tests have run.** Host tests pass and the crate has run on the MARV V2 Rev 1 board in a demonstration program against the ground station (results in "Measured on the MARV V2 Rev 1 board"; what is still open is in "Not covered and not verified"). The contract is [`docs/link-contract.md` of MARV-GCS](https://github.com/Lmx154/MARV-GCS/blob/main/docs/link-contract.md), sections 2 and 4.

## Works with

The crate works with any HAL and any MCU that provides `embedded-io-async` 0.7 streams and an `embedded-hal-async` 1.0 `DelayNs`; the messages come from `marv-dialect`. No embassy crate is needed.

## What is here

- Framing out and in, at the message-struct level (`serialize_message_data`, `deser`, `calculate_crc`); no dialect enum, no `write_*` helper.
- HEARTBEAT at 1 Hz from `Config` and `set_status` (or `Telemetry::status` while `run` is awaited); presence of the ground station (`gcs_present`).
- The stream scheduler: the firmware owns a slice of `StreamSlot`s and a `Telemetry` hook that is asked for each message when due.
- A queue of 4 STATUSTEXT (`queue_text`, or `Telemetry::texts` while `run` is awaited).
- The parameter protocol over the firmware's `ParamStore`: PARAM_REQUEST_LIST (one PARAM_VALUE per `param_list_interval_us`, 2 ms by default; a new request restarts at index 0; a store with no parameters answers with one PARAM_VALUE with `param_count` 0 and `param_index` 65535), PARAM_REQUEST_READ (by index, or by name when the index is -1), PARAM_SET (always one PARAM_VALUE with the value now in effect, also when refused). Unknown or invalid names and indices out of range get no reply.
- Commands (COMMAND_LONG): REQUEST_MESSAGE (512) for AUTOPILOT_VERSION (148, capabilities MAVLINK2 | PARAM_ENCODE_BYTEWISE, numbers from `Config::version`), HEARTBEAT, or a stream of the table; SET_MESSAGE_INTERVAL (511) for a stream of the table. Every other command gets UNSUPPORTED and nothing else happens.
- An accept list and a hand-over hook (`Config::with_accept_list`, `Node::receive_with`): messages the node does not interpret, handed to the application after their checksum and sender are checked. See "The accept list and the hand-over".
- RADIO_STATUS flow control: the node handles RADIO_STATUS itself and slows its streams when the radio's transmit buffer fills, as ArduPilot does. See "RADIO_STATUS slowdown".
- `Node::run`: the async driver over `embedded_io_async::{Read, Write}` and `embedded_hal_async::delay::DelayNs`. No executor crate. It wakes at least every 100 ms.

## Addressing and replies

- PARAM_REQUEST_LIST, PARAM_REQUEST_READ, PARAM_SET and COMMAND_LONG are acted on only when `target_system` is the node's or 0 and `target_component` is the node's or 0; otherwise `Stats::dropped_not_for_us`.
- At most 4 replies wait (read and set echoes, ACKs, requested messages). A request that arrives when the queue is full is dropped without being acted on (a PARAM_SET does not call `set`) and counted in `Stats::dropped_reply_full`; the ground station retries. A command that has a follow-up (REQUEST_MESSAGE: the ACK, then the message) takes one place for both.
- Values are encoded byte-wise: an `I32` travels as its own little-endian bytes in the 4-byte float field (`f32::from_bits(v as u32)`, back with `to_bits`) with type INT32; an `F32` as itself with REAL32. An `I32` whose bit pattern is a signalling NaN comes back bit-exact on the host, but this relies on the platform not altering NaN payloads when an `f32` is moved or stored (true for x86-64 SSE and for ARMv8-M with or without an FPU, as far as moves go); a platform that quiets NaNs in transit would corrupt such values. This case was not run on the board.
- A write is refused, without calling `ParamStore::set`, when `param_type` is neither INT32 nor REAL32 or differs from the stored value's type; otherwise `set` decides: `Ok(v)` is the value in effect, `Err(_)` means refused and `get(index)` is. A PARAM_SET with a `param_type` number the dialect does not know, and a COMMAND_LONG with a command number it does not know, cannot be decoded: no reply, `Stats::dropped_malformed`.
- SET_MESSAGE_INTERVAL: `param1` is the message id, `param2` the interval in microseconds (-1 off, 0 default, otherwise from the stream's `min_interval_us` to 10 s, whole microseconds); DENIED for an id not in the table, HEARTBEAT, `param3` not 0, or an interval outside the range. On ACCEPTED the stream restarts from now: its next sample is due at once.
- REQUEST_MESSAGE: ACCEPTED then one copy for 148, HEARTBEAT (an extra HEARTBEAT; the 1 Hz schedule is not touched) or an id in the table (one extra `Telemetry::sample`, outside the schedule, all the frames it pushes; it waits while frames of an earlier sample are still pending); any other id is DENIED. `param2`..`param7` are ignored.

## The accept list and the hand-over

For a link where other nodes talk through the same port (an ExpressLRS receiver in MAVLink mode forwards RC_CHANNELS_OVERRIDE and ground-station traffic, including messages of the project's own dialect), the application gives the node a fixed list of (message id, CRC_EXTRA) pairs and receives the listed messages through a hook.

```rust
let config = Config::new(1, 1)
    .with_accept_list(&[
        (RC_CHANNELS_OVERRIDE_DATA::ID, RC_CHANNELS_OVERRIDE_DATA::EXTRA_CRC),
        (RADIO_STATUS_DATA::ID, RADIO_STATUS_DATA::EXTRA_CRC),
    ])?; // Err(AcceptListFull) beyond 16 entries; nothing is truncated
let mut node = Node::new(config, &mut slots);
node.receive_with(&bytes, now_us(), &mut params, &mut |m: Incoming<'_>| {
    if let Some(rc) = m.decode::<RC_CHANNELS_OVERRIDE_DATA>() { /* m.system_id, m.component_id, m.now_us */ }
});
```

- The list is a fixed array of at most 16 entries inside `Config` (so inside the node): no allocation, no lifetime on `Node`. `Config::with_accept_list` is `const` and returns `Err(AcceptListFull)` for more than 16 pairs. Take the pair from the `marv-dialect` type (`ID`, `EXTRA_CRC` of `MessageData`). `AcceptList::iter()` yields the stored pairs in insertion order and `AcceptList::contains(message_id)` tells whether an id is listed, so a list can be read back through the public field `Config::accept` and rebuilt with `AcceptList::from_pairs` (which returns `Err(AcceptListFull)` beyond 16).
- `Node::receive_with(bytes, now_us, params, on_message: &mut dyn FnMut(Incoming<'_>))` calls `on_message` synchronously, from inside the call, once per frame. `Node::receive` is `receive_with` with a hook that does nothing (listed frames are then dropped, uncounted).
- A listed frame is handed over only if it is MAVLink 2 with incompatibility flags 0, its checksum is right when computed with the listed CRC_EXTRA, and it does not carry the node's own system id. Otherwise: MAVLink 1 `dropped_mavlink1`, a flag `dropped_incompat`, a wrong checksum `dropped_listed_checksum` (new; a false start that grows into a candidate frame is counted too), the own system id `dropped_own_system`.
- `Incoming { system_id, component_id, message_id, payload, now_us }`: the sender, the id, the payload slice **exactly as on the wire** and the `now_us` of the call. **MAVLink 2 senders cut the trailing zero bytes of a payload**, so the slice can be shorter than the message's encoded length, even empty: never index it. `Incoming::decode::<M>()` restores the zeros and decodes it with the `marv-dialect` type (`None` when `M::ID` differs or the payload does not decode). The slice borrows the receive buffer: copy what is needed inside the hook. The hook must not block.
- The node does not interpret listed messages, does not filter them by target system or component, does not queue them and does not route them. That is the application's. The hook runs in `receive_with`, so a `Node::run` task (which calls `receive`) has no hook: `run` is unchanged and drops listed frames.
- An entry for an id the node handles itself (HEARTBEAT, the parameter messages, COMMAND_LONG) has no effect. An entry for RADIO_STATUS (109) hands the frame over after the node has used it.
- Ids that are neither handled by the node nor listed are dropped unchecked and counted in `dropped_unknown_message`, as before.

## RADIO_STATUS slowdown

RADIO_STATUS (id 109) is handled by the node, from any sender except its own system id, with its checksum checked (`dropped_checksum` when wrong). Its `txbuf` (percent of free transmit buffer; a payload with its trailing zeros cut reads as 0) moves a per-node **slowdown**, exactly as ArduPilot's `GCS_MAVLINK::handle_radio_status` (`GCS_Common.cpp`; the architect's line numbers are 905-942, applied at 1254-1258). The four branches are tried in this order, the first that matches runs:

| `txbuf` | slowdown (ms) | change |
|---|---|---|
| < 20 | < 2000 | +60 |
| < 50 | < 2000 | +20 |
| > 95 | > 200 | -40 |
| > 90 | not 0 | -20, not below 0 |

Otherwise it does not change. The slowdown starts at 0, is reset to 0 by `on_link_down`, is read with `Node::stream_slowdown_ms()` and is added to the interval of every stream each time the stream is rescheduled (HEARTBEAT is not slowed). The bound is not a clamp: from 1980 ms a `txbuf` < 20 gives 2040 ms, as in ArduPilot; at 2000 ms or more nothing is added. On a link that never carries RADIO_STATUS (USB) the slowdown stays 0 and the schedule is what it was before this change (checked by a host test). Compared on 2026-11-13 with the master source of `GCS_Common.cpp`, fetched online and read through a summarising tool, not by eye: order, thresholds and steps agree; line numbers not checked. Not taken from ArduPilot: its multiplication of the interval by 4 during a parameter transfer, a mission transfer or FTP replies, and its 60 s cap (the architect's decision defers them).

`Config::param_list_interval_us` (builder `with_param_list_interval_us`, 2000 us by default) is per node; it is the pacing of the PARAM_VALUE frames of a parameter list and is not changed by the slowdown.

The feature `_test-hooks` (test-only, not API) adds `Node::set_slowdown_off(bool)`: the slowdown is still tracked and reported but no longer added to the intervals. It exists for a negative control on the board.

## Use

```rust
let mut slots = [StreamSlot::new(Stream { message_id: 29, default_interval_us: 100_000, min_interval_us: 20_000 })];
let mut node = Node::new(Config::new(1, 1), &mut slots);
// On each wake: bytes from the link, then frames until None, then sleep until next_deadline_us().
node.receive(&bytes, now_us(), &mut params);
let mut frame = [0u8; 280];
while let Some(n) = node.poll_transmit(now_us(), &params, &mut telemetry, &mut frame) {
    link.write_all(&frame[..n]).await?;
}
let wake_at = node.next_deadline_us(); // not after now: poll again at once
```

With an `embedded_io_async` reader and writer the whole loop is one call, in the link task:

```rust
let end = node.run(&mut rx, &mut tx, &mut delay, || now_us(), &mut params, &mut telemetry).await;
// RunError::Read(_) | Write(_) | EndOfStream: the link is gone; node.on_link_down(), wait for it, run again.
```

`Telemetry::sample(message_id, now_us, out)` is called once per period of a stream; push up to 4 frames with `out.push(&message)` (for example four HIGHRES_IMU instances) and return `true`. Nothing is queued across periods: if the caller polls late, missed periods are skipped, not sent in a burst.

## What the application provides

- The `[StreamSlot]` slice, the `ParamStore`, the clock, the byte stream and the `Telemetry` hook. `Telemetry` has one required method and two provided ones (existing implementers compile unchanged):

```rust
pub trait Telemetry {
    fn sample(&mut self, message_id: u32, now_us: u64, out: &mut Outgoing<'_>) -> bool;
    fn status(&mut self, now_us: u64) -> Option<dialect::MavState> { None }
    fn texts(&mut self, now_us: u64, out: &mut Texts<'_>) {}
}
impl Texts<'_> {
    pub fn push(&mut self, severity: dialect::MavSeverity, text: &str) -> Result<(), Full>;
}
```

- **While `run` is awaited the node is borrowed and cannot be touched**: `set_status` and `queue_text` cannot be called. Status and texts go through the two hooks, which the node calls itself, from the link task:
  - `status` is called once, right before each HEARTBEAT is built. `Some(state)` becomes the status from then on, exactly as `set_status` would; `None` leaves it as it is. A firmware that wants BOOT, then STANDBY, then ACTIVE answers from its own state (a latest-value cell it reads here).
  - `texts` is called at most once per `poll_transmit`, only when no HEARTBEAT is due and no reply waits (the order of priority is kept), and only while the queue of 4 has room. `Texts::push` has the rules of `queue_text`: cut at 50 bytes on a character boundary, `id` and `chunk_seq` 0; a `push` that does not fit returns `Full` and is counted in `Stats::texts_dropped`. A text the hook offers goes out at the next `poll_transmit` after that, behind a due heartbeat and replies and ahead of list items and streams.
  - **How soon a text goes out when the node is otherwise idle:** `next_deadline_us` knows nothing of texts the hook has not offered yet, so `run` caps its sleep at 100 ms (the cap is in `run`, not in `next_deadline_us`; a wake with nothing due changes no schedule). The hook is thus asked at least every 100 ms, and an offered text is on the wire within about 100 ms of the offer (plus the write time). A sans-IO user that sleeps until `next_deadline_us` must wake by itself at the period it wants (at most 1 s, the next heartbeat, otherwise).
  - Mixing with the plain calls is defined: `set_status` and `Telemetry::status` write the same status, the latest given wins; `queue_text` and `Texts::push` fill the same queue of 4.

## Rules for correct use

- **Who owns what.** The firmware owns the `[StreamSlot]` slice, the `ParamStore` (storage, validation, persistence), the `Telemetry` hook, the clock, the task and the byte stream; the node owns the framing, the sequence counter, the queues and the protocol. `ParamStore::set` is called from `receive` (and so from `run`), in the link task.
- **While `run` is awaited the node cannot be touched.** Do not wait for `run` to return to change the status or to send a text: implement `Telemetry::status` and `Telemetry::texts` (see "What the application provides"). Both are called in the link task, inside `poll_transmit`; like `sample` they must not block.
- **Time** comes only through `now_us` (microseconds, non-decreasing, on the same clock as the `DelayNs` handed to `run`). The crate has no clock, `static`, critical section or allocation.
- **Only the link task awaits `run`** (contract invariant 3). Sensor and control tasks never call into the node: they hand values over through non-blocking latest-value cells that `Telemetry::sample` reads. **Values handed to `Telemetry` are sampled when due, never queued**; if the caller polls late, missed periods are skipped, not sent in a burst. A write waits while the host does not read; only the link task waits, and while it does the node does not read either.
- **The reader must be cancel-safe.** `run` waits for the first of `rx.read` and the timer by polling both futures by hand; when the timer wins, the read future is dropped. A read future that has taken bytes from the link and is dropped before it returns them would lose them. The `usb-cdc` reader is cancel-safe (contract section 3). The writer is only ever awaited to completion (`write_all`), and each frame is handed to it whole, so frames are never interleaved; a write error may leave a frame partly written, and `run` returns. A `Write` that returns `Ok(0)` for a non-empty buffer panics inside `write_all` (the `embedded-io` rule); the `usb-cdc` writer does not.
- **`run` returns only** on a read error (`RunError::Read`), a write error (`RunError::Write`) or a read of 0 bytes (`RunError::EndOfStream`). It does not call `on_link_down`: the firmware does, when the link goes down (rates return to defaults, replies and the list are dropped), before it calls `run` again.
- **Receiving.** A frame is accepted only if it is MAVLink 2 with incompatibility flags 0, its checksum is right for a message the node handles (or for a message of the accept list, with the listed CRC_EXTRA: then it is handed over by `receive_with`), and its system id is not the node's own. Dropped and counted separately in `Stats`: garbage bytes, MAVLink 1 frames, frames with any incompatibility flag (signed ones too), wrong checksums (listed messages: `dropped_listed_checksum`), frames of other messages, frames with the node's system id, frames for another target, frames that did not decode. Frames of a message that is neither handled nor listed, or with an incompatibility flag, are recognised from their header and cannot be checksum-checked; their bytes are then counted as garbage. The receive buffer is 280 bytes. A false start (a stray 0xFD) never swallows a real frame, because a candidate that fails is dropped one byte at a time and the rest is scanned again; a candidate frame is waited for until its declared length has arrived.
- **Priority** in `poll_transmit`: a due heartbeat (the `status` hook is asked first), replies in order of arrival, queued texts (the `texts` hook is asked first, when the queue has room), the next due list item, the rest of the frames of the last sample, then due streams in slot order. A heartbeat waits for at most the frame being sent, also during a list of 300 parameters with every stream at its fastest. With a link too slow for all streams, slots late in the table are sampled less often. `next_deadline_us` is 0 while a reply, a text or a pushed frame waits, else the earliest of the next heartbeat, list item and stream; it does not know texts the hook has not offered yet, which is why `run` wakes at least every 100 ms.
- **The sequence number** is one counter for every frame; for frames pushed by the hook it is set when each is sent, so the numbers are consecutive on the wire.
- **`on_link_down`** returns every stream to its default interval with a fresh schedule, drops queued texts, waiting replies, the parameter list in progress and pushed frames, clears the receive buffer and the presence of the ground station, and sets the stream slowdown to 0. The sequence counter, the heartbeat schedule, the status and `Stats` carry on.
- **No failsafe.** The node takes no action when the ground station is lost, and never arms or changes a mode: arming (400) and reboot (246) get UNSUPPORTED and nothing else.
- A default interval of zero or less is off; one below `min_interval_us` is raised to it.

## `Stats`

All counters saturate and survive `on_link_down`. Made real by the parameter protocol and commands: `dropped_not_for_us` (a target that is neither the node's nor 0), `dropped_unknown_param` (PARAM_REQUEST_READ or PARAM_SET naming no parameter: unknown or invalid name, index out of range), `dropped_reply_full` (the queue of 4 replies was full), `dropped_malformed` (now also a COMMAND_LONG with an unknown command number or a PARAM_SET with an unknown type number) and `frames_sent` (replies and list items included). `unhandled_received` is gone: nothing is unhandled any more. New: `dropped_listed_checksum` (a frame of a message on the accept list with a wrong checksum).

## Not covered and not verified

- **Windows and macOS.** Everything in "Measured on the MARV V2 Rev 1 board" ran against one Linux computer (kernel 7.0.0-42, `cdc_acm`, the `serialport` 4.x crate). How another host's terminal layer treats the first bytes after an open is not known; the start gate below is built from the Linux measurement.
- **A USB-serial adapter on the computer.** Its cooked window is the same kind and the vehicle cannot see it; the ground station must resynchronise at the start of any session. Not run.
- **Long runs.** The longest session on the board is 20 s (`t1`) with 5 s recordings; nothing ran for minutes or hours, so a counter that wraps, a drift of the clock or a leak is not excluded (the sequence number wraps every 256 frames and did so many times).
- **Real sensors.** The firmware is a demonstration: every value is computed from the clock, the parameters live in RAM, there is no flight logic. Load, timing and jitter with real drivers on the same executor are not measured.
- The behaviour of the crate itself is checked on the host against fakes and against the `mavlink` 0.21.0 crate, and with the demonstration on the board as listed below.
- Signalling-NaN payloads in `I32` parameters rely on the platform (see above); only checked on x86-64.
- **The accept list, the hand-over and the RADIO_STATUS slowdown have not run against a real radio or on the board.** They are checked on the host only, with frames made by the crate's tests and the `marv-dialect` types. Whether an ExpressLRS receiver sends what the tests assume, and what its `txbuf` does under load, is not known.
- Not covered by design: signing, MAVLink 1, PARAM_EXT, PARAM_ERROR, COMMAND_INT, missions, persistence of parameters, more parameter types than INT32 and REAL32.
- The throughput at the fastest rates is unmeasured on the board (only the default rates ran).
- A very long write (a host that stops reading) blocks the link task; the node does not time it out.

## The start gate (what the application provides on USB)

The crate has no gate: on a USB link the firmware's link task holds its first write after each link-up until the computer has opened the port. In `hil/src/bin/mavlink_demo.rs` the loop is `link.wait_up()`, `start_gate`, `node.run`, `node.on_link_down()`. The gate writes nothing; it waits for the first of bytes from `rx` and a 5 ms timer, gives the bytes to `Node::receive`, and opens when `Node::gcs_present` is true or when `Link::dtr()` has read true at every check for 100 ms. A read error ends it (link down). After it the node runs as before, also into a closed port. Why: [`docs/link-contract.md` of MARV-GCS](https://github.com/Lmx154/MARV-GCS/blob/main/docs/link-contract.md) (section 3, "The start gate") and the README of [`usb-cdc`](https://github.com/Lmx154/usb-cdc).

## Measured on the MARV V2 Rev 1 board

All of it with `mavlink_demo` (`hil/src/bin/mavlink_demo.rs`, started through `tools/run.sh`) against the ground station's tests `crates/marv-gcs/tests/board.rs`, run by `tools/board_check.sh` of [MARV-GCS](https://github.com/Lmx154/MARV-GCS) (three phases, the device program restarted before each), on one Linux computer, two complete passes (`final1`, `final2`). Where the two passes differ both numbers are given.

| What | Number | Measured by |
|---|---|---|
| Time from the port open to Connected | 20 ms (the first 20 ms poll), both passes | `t1` |
| Rates after 10 s | HIGHRES_IMU 50.000 to 50.002 Hz for each of 4 instances, SCALED_PRESSURE 10.000, GPS_RAW_INT 5.000, HEARTBEAT 1.000 Hz | `t1` |
| First 20 s of the first session after a restart | 4343 frames, 0 checksum errors, 0 discarded bytes, 0 lost frames, both passes | `t1` |
| First byte read, first frame | 0xFD; HEARTBEAT, sequence number 0 | `t1` (recording reader, then the deframer) |
| Start-up text `MARV link demo ready` | in the state at the first poll after Connected (0 ms), both passes | `t1` |
| A listener that sends nothing | first byte 103.1 / 102.4 ms after the open returned; 628 frames in 3 s, 0 checksum errors, 0 discarded bytes, first sequence 0 then consecutive, text among them. The board logged `gate open by DTR` 325 / 250 ms after link-up | `listen_only` |
| A host that keeps DTR low | 0 bytes in the first second; first whole frame 0.6 ms after the ground-station HEARTBEAT was written, sequence 0; 650 frames in 3 s, 0 errors, 0 discarded bytes. The board logged `gate open by HEARTBEAT ... dtr false` after 1138 / 1199 ms | `no_dtr` |
| Parameters | 13 listed exactly in 20 ms; ten refreshes 41 ms each (40 ms once); a write confirmed in 20 ms; read-only refused with the old value in effect; 5000 clamped to 1000 | `t2`, `t3` |
| Recording | 1103 frames, 87253 bytes, 1103 records (1097 from the vehicle, 6 from the ground station), the same 1103 decoded by pymavlink, 0 BAD_DATA | `t4` and `tools/board_check.sh` |
| Reconnect after a close | Connected in 20 ms, list in 20 ms | `t5` |
| Sizes on the board (before the accept list; `Node` is now larger by its 16-entry list) | `Node` 1952 bytes, the `run` future 536 bytes, `Params` 260 bytes | start-up lines of `mavlink_demo` |

With the gate switched off by a temporary edit of the firmware, `t1` did not receive the start-up text, `listen_only` read 0xC5 as its first byte and `no_dtr` read 15792 bytes in the silent second (see the report of the integration run).

## Tests

Host: `cargo test` (115 unit and property tests, 1 doc test, observed on this tree; 117 and 1 with `--all-features`, which adds the negative control of the slowdown test; `src/accept_tests.rs`: the accept list, the hand-over and the slowdown). Every frame the node emits in a test is also decoded by the `mavlink` 0.21.0 reader (dev-dependency); input frames are made by that crate. `src/service_tests.rs`: parameters and commands, `src/run_tests.rs`: `run` against a fake reader, writer and delay on a made-up clock, `src/e2e_tests.rs`: a ground-station model over a link that loses 20 % of the frames each way. `src/fuzz_tests.rs` holds the property tests: arbitrary bytes in arbitrary splits (no panic, buffer bounded), valid frames in arbitrary splits with noise between them, a real frame behind a false start, arbitrary use of the whole node.

For the target: `cargo build --target thumbv8m.main-none-eabihf`, and `cargo tree -e normal --target thumbv8m.main-none-eabihf | grep -cE 'mavlink-bindgen|quick-xml|regex|mavlink v|usb|serialport|embassy'` prints 0.

## Cargo features

| Feature | Default | Effect |
|---|---|---|
| `defmt` | off | `defmt::Format` on public types; enables `embedded-io-async/defmt` |
| `_test-hooks` | off | Test-only: `Node::set_slowdown_off`. Not part of the API. |

## Layout and licence

```text
src/lib.rs       docs, lints, exports
src/node.rs      Node, Stats, receive, poll_transmit
src/node/service.rs  parameter requests, commands, the reply queue, the list
src/run.rs       Node::run, RunError
src/deframer.rs  bounded MAVLink 2 deframer
src/accept.rs    AcceptList, Incoming (the hand-over)
src/config.rs    Config, VersionInfo, Stream, StreamSlot
src/out.rs       Telemetry (sample, status, texts), Outgoing, Texts
src/params.rs    ParamStore, ParamName, ParamValue, ParamError
```

MIT, see `LICENSE`.
