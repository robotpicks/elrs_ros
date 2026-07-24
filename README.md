# elrs_ros

A generic **ExpressLRS / CRSF receiver driver for ROS 2**. It reads an RC receiver's serial
stream (ExpressLRS or TBS Crossfire — both speak the **CRSF** protocol) over a UART and:

- publishes the RC channels as **`sensor_msgs/Joy`**, so any joy-based teleop maps them, and
- pushes **battery telemetry back to the handset**, sourced from a standard
  **`sensor_msgs/BatteryState`** your robot publishes.

It depends only on `rclpy` + `sensor_msgs` — no robot-specific messages — so it drops into any
ROS 2 stack.

## Why this exists

CRSF is a great robot RC link (long range, one UART, RC **and** telemetry bidirectional), but the
existing ROS options don't fit a general robot:

- [Husarion `crsf_teleop`](https://husarion.com/tutorials/ros-equipment/radiomaster-tx16s/)
  outputs `geometry_msgs/Twist` directly and is tied to their UGV ecosystem/dongle.
- [`crsf_parser`](https://github.com/AlessioMorale/crsf_parser) is a fine Python CRSF codec but
  not a ROS driver.

`elrs_ros` publishes **standard `sensor_msgs/Joy`** instead of a fixed command type, so you reuse
whatever teleop/mapping you already have, and it closes the loop with **telemetry back to the
handset**.

## Architecture — and where MAVLink fits

The node is **wire-format-agnostic**. All serial framing lives in a pluggable *link codec*
(`elrs_driver/links.py`); the node only ever speaks standard ROS messages:

```
  RX serial UART ──►┌ CrsfLink   (crsf.py)     ┐──► RC frames  → sensor_msgs/Joy   (node output)
   (the ELRS link)  └ MavlinkLink (pymavlink) *┘◄── telemetry  ← sensor_msgs/BatteryState (input)
                        ▲ link_mode: crsf | mavlink  — the ROS interface is identical either way
                        * roadmap
```

So **MAVLink-over-ELRS** (receiver in MAVLink mode, the same UART carrying a MAVLink stream) is
simply a **second codec** — `link_mode: mavlink` — that parses `RC_CHANNELS` into the same Joy and
encodes `BATTERY_STATUS` from the same BatteryState. The node doesn't change. (Sending MAVLink to
a *network* ground station like QGroundControl is a different job that
[`mavros`](https://github.com/mavlink/mavros) already does — out of scope here.)

## Install

```bash
cd ~/ros2_ws/src
git clone https://github.com/robotpicks/elrs_ros.git
cd ~/ros2_ws
rosdep install --from-paths src/elrs_ros --ignore-src -r -y   # pulls pyserial (python3-serial)
colcon build --packages-select elrs_driver
source install/setup.bash
```

On a PEP 668 (externally-managed) Python, install pyserial with
`pip install --user --break-system-packages pyserial` instead of rosdep.

## Usage

```bash
ros2 launch elrs_driver elrs_driver.launch.py            # uses config/elrs_driver.yaml
# or run the node directly with overrides:
ros2 run elrs_driver elrs_driver --ros-args -p serial_port:=/dev/ttyUSB0 -p baud:=420000
```

Then point any joy teleop at `/joy`, e.g. `teleop_twist_joy`:

```bash
ros2 run teleop_twist_joy teleop_node --ros-args -r joy:=/joy
```

Remap topics as needed: `-r joy:=/rc/joy -r battery:=/robot/battery`.

## Wiring

- Put the receiver in **CRSF serial** mode (its default), **not** MAVLink mode. Default baud
  **420000**.
- Simplest hookup is a 3V3 USB-UART adapter: adapter **RX ← RX TX pad** (channels), adapter
  **TX → RX RX pad** (telemetry to the handset), plus GND and 5V. On a Pi, a header UART
  (`/dev/ttyAMA0`, console disabled) works too.
- CRSF is half-duplex; on a two-wire adapter hookup telemetry writes are fine. On a true
  single-wire hookup, telemetry timing collisions are possible — this driver writes telemetry on
  a simple timer and does not yet arbitrate RX telemetry slots.

## Parameters

| Param | Default | Meaning |
|-------|---------|---------|
| `serial_port` | `/dev/ttyUSB0` | Receiver UART device |
| `baud` | `420000` | CRSF baud |
| `require_serial` | `true` | `false` = dry-run: open no port, log intended telemetry |
| `link_mode` | `crsf` | Wire codec; `mavlink` is roadmap |
| `poll_rate_hz` | `200.0` | Serial-drain / Joy-publish tick |
| `axis_channels` | `[0,1,2,3]` | Channel (0-based) → Joy axis, in axis order (AETR default) |
| `deadman_channel` | `4` | Channel used as arm/deadman switch |
| `deadman_button_index` | `4` | Joy button index the switch maps to |
| `deadman_threshold` | `0.5` | Switch "on" when its normalized value exceeds this |
| `rc_timeout_sec` | `0.5` | Publish a neutral Joy if RC goes stale (failsafe) |
| `telemetry_rate_hz` | `5.0` | Battery telemetry rate to the handset (0 disables) |

Channel numbering is transmitter-dependent — verify by watching `/joy` while moving each
stick/switch before trusting the defaults.

## Telemetry

Publish a `sensor_msgs/BatteryState` on `/battery` (voltage, current, and optionally percentage);
the driver encodes it as a CRSF `BATTERY_SENSOR` frame for the handset. `current` follows the ROS
convention (negative when discharging); its magnitude is reported.

## Testing without a receiver

- **Dry-run:** `ros2 run elrs_driver elrs_driver --ros-args -p require_serial:=false` — no port
  opened, telemetry frames logged.
- **Virtual serial:** `socat -d -d pty,raw,echo=0 pty,raw,echo=0`, point `serial_port` at one
  pty, and write canned `RC_CHANNELS_PACKED` bytes into the other (see
  `elrs_driver/crsf.py:build_rc_channels_frame`).
- **Codec unit tests** (no ROS needed): `pytest src/elrs_ros/elrs_driver/test`.

## Roadmap

- `link_mode: mavlink` — MAVLink-over-ELRS via a `MavlinkLink` codec (pymavlink), no node changes.
- More telemetry sensors to the handset: GPS from `sensor_msgs/NavSatFix`, attitude from
  `sensor_msgs/Imu` (CRSF `GPS` / `ATTITUDE` frames).
- Half-duplex telemetry-slot arbitration.

## Credits / prior art

[ExpressLRS](https://github.com/ExpressLRS/ExpressLRS) ·
[Husarion crsf_teleop](https://husarion.com/tutorials/ros-equipment/radiomaster-tx16s/) ·
[AlessioMorale/crsf_parser](https://github.com/AlessioMorale/crsf_parser)

## License

Apache-2.0 — see [LICENSE](LICENSE).
