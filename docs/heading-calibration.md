# Heading calibration and troubleshooting

How to measure the device's heading correction, how to tell when a correction
is the wrong fix, and what to do then.

The heading is the weakest link in a coordinate. Range and position are good
to centimetres; a degree of heading error moves the target **1.7 m sideways
per 100 m of range**, so the whole accuracy of the instrument rests on this
one number.

## Background

The compass takes its heading from the magnetometer, so the bearings it
reports are measured from **magnetic** north, and they carry two other errors
with them: the breakout sitting a little crooked against the
direction the LiDAR actually looks, and whatever magnetised iron rides along
with the device that the magnetometer calibration has not removed. Three causes, one effect — the device cannot tell them apart,
and only their sum changes a coordinate.

So the firmware keeps their sum, as one measured number:

```rust
// src/constants.rs
pub const HEADING_CORRECTION_DEG: f32 = 0.0;
```

It is added to every bearing the device reports, positive clockwise. While it
is zero the device reports the compass's own magnetic bearings, which is where
it starts: nothing in the firmware guesses the correction from a magnetic model.
The `heading_cal` binary measures it against bearings that are known, and
prints the value to put here.

Two numbers the measurement can be sanity-checked against, both for
Huntsville, Alabama, from the World Magnetic Model (WMM2025) for late 2026:

| Quantity                    | Expected          | Where it is used          |
|-----------------------------|-------------------|---------------------------|
| Magnetic declination        | 4.3° **west**     | `heading_cal` result      |
| Total field strength        | 48.8 µT           | `compass_diag`            |
| Dip below the horizon       | 62.6°             | `compass_diag`            |

Declination 4.3° west means a magnetic bearing reads about **4.3° high**, so a
correctly mounted, magnetically clean device should measure a correction near
`-4.3`. A measurement far from that is not wrong by itself — mounting and iron
are in there too — but it says one of those two is contributing, and it is
worth knowing which before trusting the number. Declination also drifts about
a tenth of a degree a year, and it is a property of the place: a correction
measured here does not travel to another city.

## Equipment

- The device, assembled the way it will be used.
- A level surface, or better a tripod or camera mount: a hand-held heading
  wanders a degree or two, and the routine rejects a station that moves.
- A map tool that gives true bearings — Google Earth Pro's ruler shows a
  **Heading**, which is a true azimuth — or the NOAA solar calculator for the
  shadow method below.
- Open ground away from buildings, vehicles and fences. Structural steel bends
  the field exactly where accuracy is wanted.

Wherever a step says to run a binary, use:

```sh
cargo run --release --bin imu_cal
cargo run --release --bin heading_cal
cargo run --release --bin compass_diag
```

RTT output appears in that terminal; the display shows the prompts.

## Before you measure

Each of these can spoil the measurement on its own, so work through them
first.

### 1. Assemble the device as it will be used

- [ ] IMU, LiDAR and board mounted in their final relative positions, with the
  final screws, standoffs and enclosure.
- [ ] Battery, buck converter and 12 V LiDAR feed in place and powered.

Anything magnetic that moves with the IMU becomes part of the correction, and
anything that moves *relative* to it makes the correction a fiction. A
correction measured on a loose breadboard does not apply to the assembled
device.

### 2. Calibrate the magnetometer

Run `imu_cal` away from desks, laptops, vehicles and building steel. It wipes
what the hub has learnt, then walks through the six faces of the device and
needs nothing but the display:

1. **Rest it with the named face up** (`+X up`, `-X up`, `+Y up`, `-Y up`,
   `+Z up`, `-Z up`, using the axes printed on the breakout). It starts on its
   own once the face is within 10° of straight up and still for a second and
   a half. Propping it against a box works better than holding it.
2. **Figure of eight for five seconds**, when the display counts down.
3. After the sixth face, **tumble it slowly through every direction** until
   `cover` reaches 24 of 26.

Then it fits an ellipsoid to the readings and prints the result over RTT. It
is good if:

- [ ] `sectors covered` is at least 24 of 26. Readings from one side alone can
  fit with a tiny residual and still be wrong.
- [ ] `residual` is under 0.03. Around 0.01 is normal.
- [ ] `field strength` is within 20% of **48.8 µT**.

Paste the printed `MAG_CALIBRATION` over the one in
[`src/constants.rs`](../src/constants.rs). A failed run prints why and no
constant; move away from whatever is near and press reset to go again.

The magnetometer correction lives in the firmware, because the hub has no way
to accept one. The hub saves only its own accelerometer and gyroscope
calibration. The `accelerometer, six point check` lines are a check of that:
bias near 0 and scale near 1. Each axis reads gravity up and then down, and a
face that was tilted shows up here as a scale error.

Headings measured without a calibration are worth nothing; `heading_cal`
records them anyway but marks the result provisional.

### 3. Check the field with `compass_diag`

Run `compass_diag` where the measurement will happen and compare its field
against the table above.

- [ ] Strength within a few µT of **48.8**.
- [ ] Dip within a few degrees of **62.6**.
- [ ] The `compass (app)` heading and the `-> heading` on the field line agree
  within a degree or two.
- [ ] The compass status reads `High`.

**Fail:** something near the magnetometer is bending the field. Stop here and
work through *When the error is not a constant* below — a correction measured
in a bent field is only an average of a wrong answer.

### 4. Get the known bearings

The easiest way to get four well-spread bearings from one standing spot: pick
four landmarks you can both identify on a map and sight along — a tower, a
pole, a building corner, a far gate — and read each one's true bearing from
where the device will stand.

- Use landmarks a few hundred metres away. Aiming is part of the measurement,
  and 1 m of sighting error at 200 m is only 0.3°, while at 20 m it is 3°.
- Google Earth Pro: ruler, line, from the device's spot to the landmark; read
  **Heading**. That is a true bearing already.
- No map? Use the sun. The NOAA solar calculator gives the sun's azimuth for
  a place and a time to the minute; the shadow of a vertical rod points
  180° away from it. Note the time to the second and work out the azimuth
  for exactly that moment.
- **Not** a handheld or phone compass. Those are magnetic too, so they would
  measure the device against the very thing being corrected for.

Then put the bearings, in the order they will be aimed at, in `STATIONS` near
the top of [`src/bin/heading_cal.rs`](../src/bin/heading_cal.rs):

```rust
const STATIONS: [f32; 4] = [12.4, 103.9, 198.0, 291.5];
```

They need not be evenly spaced or four in number, but between them they must
cover a good part of the circle — that is what separates a constant error from
one that changes with the heading.

## Measuring the correction

1. Flash and start the routine: `cargo run --release --bin heading_cal`.
2. Check the display reads `high`. If it reads `compass poor`, run `imu_cal`
   first, or move away from whatever is disturbing the field.
3. Set the device down level on its mount at the measuring spot, and aim it
   along the first station's bearing: the LiDAR's own line of sight, at that
   station's landmark.
4. Press the button and keep still. The display reads `measuring...` for a
   second and a half, then shows what it got and the error.
5. Repeat for each station. The display counts them off (`stn 2/4`).
6. After the last station the routine prints the table and the verdict, and
   waits. Press the button again for a fresh run.

The routine offers a station again, rather than spoiling the average, when:

| Display              | Meaning                                            |
|----------------------|----------------------------------------------------|
| `moved 3.4 deg`      | The heading wandered while averaging; hold still   |
| `no IMU reports`     | The hub sent nothing for the whole window          |

## Reading the result

```
---- heading calibration, 4 stations
station  known  reported  error   elev   roll  steady  compass  hub +/-
   1      12.4     16.6    +4.2   -0.3   +0.8    0.4   high     5.0
   2     103.9    108.0    +4.1   -0.5   +1.1    0.5   high     5.0
   3     198.0    202.4    +4.4   -0.2   +0.9    0.3   high     5.0
   4     291.5    295.5    +4.0   -0.4   +1.0    0.6   high     5.0
average error +4.2 deg, spread 0.4 deg peak to peak (limit 5.0)
The error is the same whichever way the device faces, so one number removes it:
    pub const HEADING_CORRECTION_DEG: f32 = -4.2;   // was 0.0
```

| Column    | What it is                                                   |
|-----------|--------------------------------------------------------------|
| `known`   | The station's bearing from `STATIONS`                        |
| `reported`| What the device said, exactly as the application computes it |
| `error`   | Reported minus known: what a correction has to remove        |
| `elev`    | Elevation of the boresight; near zero means level            |
| `roll`    | Roll of the breakout; near zero means level                  |
| `steady`  | How far the heading wandered while averaging                 |
| `compass` | The worst the compass thought of itself during the station   |
| `hub +/-` | The compass's estimate of its heading error, from the fit's residual and how far the field strength is off |

A result worth acting on has:

- [ ] **spread** well under 5°. This is the headline number: it is how much of
  the error no correction can remove.
- [ ] **compass** `high` at every station.
- [ ] **steady** under a degree, and `elev`/`roll` within a few degrees of
  zero.
- [ ] an average error in the neighbourhood of **+4.3°** — the declination
  here. Much more than that is mounting or iron on top of the declination;
  see the troubleshooting table.

## Applying it, and checking

1. Put the printed number in [`src/constants.rs`](../src/constants.rs) as
   `HEADING_CORRECTION_DEG`.
2. Flash the application: `cargo run --release --bin ping_capstone`.
3. Run `heading_cal` once more, from the same spot, at the same stations.
   The errors should now average near zero — the routine accounts for the
   correction already in the firmware, so a second run is a check and not
   another offset to add.
4. Finish on a real target: sight something identifiable 50–150 m away, press
   the button, and compare the coordinate it reports with the same thing on a
   map. At 100 m it should land within a few metres.

Measure again whenever the device is taken apart and rebuilt, the IMU or LiDAR
is remounted, anything magnetic is added near the IMU, or the device is used
in another city. After a year, declination alone has moved by about a tenth of
a degree, which is not worth chasing.

## When the error is not a constant

A spread wider than 5° means `heading_cal` found a heading-dependent error.
Averaging it would only spread it evenly around the circle; the source has to
be found instead. It is a current or a piece of iron that turns *with* the
device, so the field it adds points a different way at every heading.

Change one thing at a time, and after each change run `compass_diag` and watch
the field strength and dip against the expected 48.8 µT and 62.6°:

1. **Move away from the building.** Walk out into the open, away from walls,
   vehicles and chain-link fences, and measure again. Steel in a wall is the
   most common cause by far.
2. **Unpower the LiDAR.** The TF03 draws up to 480 mA in peaks, and the
   current in its supply wires makes a field of its own. If the error settles,
   route its 12 V and ground as a tight pair, away from the IMU, and keep the
   loop they make small.
3. **Check Wi-Fi.** `heading_cal` and `compass_diag` deliberately leave the
   radio off. If the application will aim with Wi-Fi on, measure with it on
   too, and compare.
4. **Look at what is bolted near the IMU.** Steel standoffs and screws,
   magnets in a speaker or a buzzer, a battery's nickel tabs, the buck
   converter's inductor. Swap steel fasteners near the IMU for brass or nylon.
5. **Move the IMU.** A few centimetres further from the board, the converter
   and the battery is often all it takes; the field of a nearby source falls
   off steeply with distance.

Once the field reads right and the spread is small, measure the correction
again.

## Troubleshooting the angle

| Symptom | Likely cause | What to do |
|---------|--------------|------------|
| Compass accuracy never reaches `high` | `MAG_CALIBRATION` is `None` or out of date, or a bent field | Run `imu_cal`. If it already passed, run `compass_diag`: a field far off 48.8 µT is the real problem |
| `imu_cal` stops at `residual too big` or `wrong field` | Something near the device, or a magnet that moves relative to the IMU | Move away and run it again. A magnet fixed to the device is fine and is what the calibration removes; one that shifts, such as an antenna on a loose cable, cannot be calibrated out |
| `compass_diag` field strength far off 48.8 µT, or dip far off 62.6° | Magnetised iron or a current near the IMU | *When the error is not a constant*, above |
| Spread over 5°, average looks plausible | Soft iron or a current turning with the device | *When the error is not a constant*, above |
| Error close to +90°, −90° or 180° at every station | `LIDAR_BORESIGHT` names the wrong axis | Set it to the axis the LiDAR actually looks along, in the breakout's printed frame, in [`src/constants.rs`](../src/constants.rs). A whole quarter turn is a frame mistake, not something to trim away |
| Error consistent but tens of degrees, and the boresight is right | The breakout is mounted crooked | The correction will absorb it, but a large one is fragile: it only holds while the IMU never shifts in its mount. Straighten the mounting if you can |
| Heading drifts slowly while the device sits still | Field being rejected as disturbed, so the heading is coasting on the gyroscope | Compare the headings in `compass_diag`. The compass and field headings should agree; the game heading is gyro-only and drifting is normal for it |
| Heading jumps a degree or two when the button is pressed | Pressing jogs the device | Use a mount, press gently. The application samples at the moment the button goes down for this reason, so a jog afterwards does not matter |
| Heading good one run, several degrees off the next | The compass settling after a reset (about three seconds), or the IMU shifting in its mount | Wait a few seconds before measuring. If it repeats, check that nothing in the assembly moves |
| `hub +/-` large (tens of degrees) | The compass does not trust its heading | Treat the result as provisional: calibrate, check the field, measure again |
| Elevation has the wrong sign, or roll looks swapped | The breakout is mounted in a different orientation than the boresight assumes | Check the printed axes against [`src/bno085/reports.rs`](../src/bno085/reports.rs)'s description of yaw, pitch and roll, then fix `LIDAR_BORESIGHT` |
| Correction good outdoors, wrong indoors (or vice versa) | Building steel | Calibrate where the device is used, and expect indoor bearings to be poor |
| Coordinates off by a consistent sideways distance, bearings look right | The correction was never applied | Check `HEADING_CORRECTION_DEG` in the flashed build, and that the application was reflashed after it changed |

## Notes

**Why one number and not declination plus a trim.** Two constants that are
only ever added together cannot be told apart by any measurement the device
can make, and splitting them invites setting one from a model and the other
from a measurement that already includes it — counting the declination twice.
One measured number, checked against the local declination for plausibility,
is harder to get wrong.

**Why not the hub's tare.** The BNO085 can rotate its own reference frame and
persist that in its flash ([`Bno085::tare`](../src/bno085.rs)). It would apply
only to the hub's own outputs, and it hides the correction inside a part,
where it survives reflashing,
cannot be read back or reviewed, and quietly disappears when a hub is
replaced. The number stays in `constants.rs`, where it is visible in the
source tree.

## References

- [`src/bin/imu_cal.rs`](../src/bin/imu_cal.rs): the six face sequence.
- [`src/mag_cal.rs`](../src/mag_cal.rs): the ellipsoid fit and the compass.
- MathWorks, [Magnetometer
  calibration](https://www.mathworks.com/help/fusion/ug/magnetometer-calibration.html):
  the model, the `sym` fit and the residual used here.
- [`src/bin/heading_cal.rs`](../src/bin/heading_cal.rs): the routine, its
  stations and its limits.
- [`src/bin/compass_diag.rs`](../src/bin/compass_diag.rs): the field and
  heading diagnostic, and the expected field for Huntsville.
- [`src/constants.rs`](../src/constants.rs): `MAG_CALIBRATION`,
  `HEADING_CORRECTION_DEG` and `LIDAR_BORESIGHT`.
- [`src/geo.rs`](../src/geo.rs): how a bearing and a range become a
  coordinate, and how heading error propagates into it.
- NOAA, [magnetic field
  calculator](https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml):
  declination, total field and dip for a place and date.
- NOAA, [solar position
  calculator](https://gml.noaa.gov/grad/solcalc/azel.html): the sun's azimuth,
  for the shadow method.
