# TF03 LiDAR troubleshooting test plan

A bench procedure for a TF03 that shows no distance in `lidar_demo`. Each step
proves one link between the ESP32-S3 and the TF03, so the first step that
fails points at the fault. Work through the steps in order and stop at the
first failure.

## Background

The TF03 here is the **5–24 V UART/CAN version**. Its six-wire cable follows
Benewake's 2024 *TF03 UART/CAN User Manual*, not the older 5 V version's V1.0
manual, which has different wire colours.

| TF03 wire | Signal          | Board pin           | Needed for distance |
|-----------|-----------------|---------------------|---------------------|
| Red       | 5–24 V supply   | fused 12 V line     | yes                 |
| Black     | GND             | GND                 | yes                 |
| Brown     | UART TxD (out)  | GPIO18 (UART1 RX)   | yes                 |
| Blue      | UART RxD (in)   | GPIO17 (UART1 TX)   | no, commands only   |
| White     | CAN_L           | not connected       | no                  |
| Green     | CAN_H           | not connected       | no                  |

The UART is 3.3 V LVTTL at 115200 8N1. When powered, the TF03 sends 100
measurements a second without being asked, so **distances need only red,
black and brown**. Blue carries commands to the TF03, such as a firmware
version request or a frame rate change. The project doesn't send any
commands yet beyond that one version request at startup.

Don't leave blue unconnected. Put it on GPIO17, or on 3V3 if the board will
never send commands. In earlier runs the TF03 appeared to stop streaming
while blue was unconnected or held at GND.

The full pin map is in [`src/constants.rs`](../src/constants.rs).

## Equipment

- Multimeter
- A spare jumper wire
- A 10 kΩ resistor (optional, for step 8)

Wherever a step says to run the demo, use:

```sh
cargo run --release --bin lidar_demo
```

## What the demo reports

The display and RTT show, five times a second:

| Message                                | Meaning                                    |
|----------------------------------------|--------------------------------------------|
| `dist … m` / `strength …`              | A distance reading. This is the goal.      |
| `out of range` / `raw … cm`            | Readings arrive, but nothing is in range   |
| `TF03 no data` / `check power, …`      | No reading for a whole second              |

At startup, RTT also logs one line about the firmware version request. The
demo reads distances whatever this line says.

| Startup line                              | What reached GPIO18 after the request          |
|-------------------------------------------|------------------------------------------------|
| `TF03 firmware x.y.z`                     | The answer: blue and brown both work           |
| `… NoReply(Measurements) …`               | Measurements, but the request didn't reach the TF03 |
| `… NoReply(OwnCommand) …`                 | Only a copy of the board's own request         |
| `… NoReply(Noise) …`                      | Bytes, but no valid packet                     |
| `… NoReply(Nothing) …`                    | Not a single byte                              |

## Steps

### 1. Board self-test (TF03 disconnected)

Disconnect all TF03 wires from the board. Put a jumper from **GPIO17** to
**GPIO18**, then run the demo.

- [ ] **Pass:** the startup line says `NoReply(OwnCommand)`. This proves the
  board's TX, its RX and the firmware are working. The display then shows
  `TF03 no data`, which is expected here.
- **Fail:** stop. The fault is on the board or in the firmware. Keep the log.

Remove the jumper afterwards.

### 2. TF03 power

Connect only the TF03's red and black wires. Measure red to black **at the
TF03 plug**.

- [ ] **Pass:** a steady 12 V (anything from 5 to 24 V is in spec).
- **Fail:** check the 12 V feed, fuse F1, diode D1 and the crimps in the
  Molex plug.

### 3. Ground

With the power off, check continuity between:

- [ ] TF03 black and the board's **GND** pin: under 1 Ω
- [ ] the 12 V supply's negative and the board's **GND** pin: under 1 Ω

**Fail:** the TF03 and the board do not share a ground. Serial data means
nothing without one.

### 4. Distances

Connect brown to **GPIO18** and blue to **3V3**. Run the demo for a full
minute.

- [ ] **Pass:** distances on every refresh that change as you move your hand
  in front of the lens, with no `TF03 no data` in the whole minute. This
  proves power, ground, the brown wire and the baud rate, since readings can
  only be decoded if the baud rate matches. **If you only need distances,
  you're done:** skip to step 8.
- **Fail, with occasional `TF03 no data`:** a power, ground or brown
  connection is loose. With the demo running, wiggle red, black and brown in
  turn at both ends. The wire that flips the message is the culprit.
- **Fail, `TF03 no data` throughout:** recheck steps 2 and 3, then brown's
  continuity to GPIO18.

### 5. Blue wire, end to end (commands only)

With the power off, check continuity from blue **at the TF03 plug** to the
board pins:

- [ ] to **GPIO17**: under 1 Ω
- [ ] to **GPIO16** and **GPIO18** on either side: open circuit

**Fail:** fix the crimp, jumper or breadboard row.

### 6. Board TX voltage (commands only)

Connect blue to GPIO17 and run the demo. Measure to GND:

- [ ] **GPIO17:** a steady 3.3 V
- [ ] **blue at the TF03 plug:** a steady 3.3 V

The UART's TX line rests high between requests, and the brief requests barely
move a meter.

- **Fail at GPIO17:** the board is not driving GPIO17. Keep the log; this is a
  board or firmware fault.
- **Fail only at the plug:** the blue connection is bad even though step 5
  passed. Look for an intermittent contact by wiggling it.

### 7. Commands (commands only)

With everything connected as in the table at the top, run the demo.

- [ ] **Pass:** the startup line says `TF03 firmware x.y.z`, and distances
  follow as in step 4.
- **Fail, `NoReply(Measurements)`, with steps 1–6 all passing:** the wiring
  is proven. Keep the log and a note of which wire is on which pin. The
  remaining suspects are the TF03's firmware or a pinout that differs from
  the 2024 manual. Distances still work without commands.

### 8. Hardening, once it works

- [ ] A 10 kΩ resistor from blue to 3V3, so a loose blue wire leaves the TF03
  streaming instead of stalling.
- [ ] A 10 kΩ resistor from GPIO18 to 3V3. That keeps a disconnected RX line
  quiet, so it reads `NoReply(Nothing)` instead of picking up the board's own
  request.
- [ ] Strain relief on the Molex plug, so the cable can't pull on the crimps.

## References

- Benewake, [*TF03 UART/CAN User Manual*](https://en.benewake.com/uploadfiles/2025/04/20250430180007797.pdf)
  (2024): wire colours, LVTTL level, command set.
- [`src/tf03.rs`](../src/tf03.rs): the driver, including how `NoReply`
  diagnoses an unanswered command.
