"""Generic ExpressLRS / CRSF receiver driver for ROS 2.

Reads an RC receiver's serial stream (ExpressLRS or TBS Crossfire, CRSF protocol) and publishes
the channels as ``sensor_msgs/Joy`` — so any existing joy-based teleop maps them — and pushes
battery telemetry (from a ``sensor_msgs/BatteryState`` the robot publishes) back down the link to
the handset.

The node is wire-format-agnostic: all serial framing lives in a pluggable link codec (see
``links.py``). ``link_mode: crsf`` is implemented; ``link_mode: mavlink`` (MAVLink-over-ELRS) is
the roadmap slot and needs no changes here — the ROS interface (Joy out, BatteryState in) is the
same for any codec.

Design conventions (robot-agnostic, no dependency on any particular message set beyond the
standard ``sensor_msgs``): the transport lib (pyserial) is imported lazily so the package builds
and dry-runs without it; ``require_serial: false`` opens no port and logs intended telemetry
instead of sending (handy for CI / bring-up); and all serial I/O is wrapped so a hardware hiccup
logs but never kills the node.
"""

import math

import rclpy
from rclpy.node import Node as RosNode
from sensor_msgs.msg import BatteryState, Joy

from elrs_driver import links


class ElrsDriverNode(RosNode):

    def __init__(self):
        super().__init__('elrs_driver')

        # --- serial / link -----------------------------------------------------------------
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 420000)
        self.declare_parameter('require_serial', True)
        self.declare_parameter('link_mode', 'crsf')          # 'crsf' now; 'mavlink' = roadmap
        self.declare_parameter('poll_rate_hz', 200.0)         # serial-drain / Joy-publish tick

        # --- RC channel -> Joy mapping -----------------------------------------------------
        # Which channel (0-based) feeds each Joy axis, in axis order. Downstream teleop then
        # selects the axes it wants. Defaults are AETR order -- verify against your transmitter.
        self.declare_parameter('axis_channels', [0, 1, 2, 3])
        # Arm/deadman switch: a channel exposed as a Joy button so a joy teleop's deadman gate
        # works unchanged. deadman_button_index is where in Joy.buttons it lands.
        self.declare_parameter('deadman_channel', 4)
        self.declare_parameter('deadman_button_index', 4)
        self.declare_parameter('deadman_threshold', 0.5)     # "on" when channel unit > this
        self.declare_parameter('rc_timeout_sec', 0.5)        # neutral Joy if RC goes stale

        # --- telemetry back to the handset -------------------------------------------------
        self.declare_parameter('telemetry_rate_hz', 5.0)     # battery frames pushed to the RX

        self._require_serial = self.get_parameter('require_serial').value
        self._axis_channels = list(self.get_parameter('axis_channels').value)
        self._deadman_channel = self.get_parameter('deadman_channel').value
        self._deadman_button_index = self.get_parameter('deadman_button_index').value
        self._deadman_threshold = self.get_parameter('deadman_threshold').value
        self._rc_timeout = self.get_parameter('rc_timeout_sec').value

        self._link = self._make_link(self.get_parameter('link_mode').value)
        self._serial = None
        self._last_rc_time = None
        self._battery = None                                  # latest BatteryState

        self._joy_pub = self.create_publisher(Joy, 'joy', 10)
        self.create_subscription(BatteryState, 'battery', self._on_battery, 10)

        if self._require_serial:
            self._serial = self._open_serial()
            poll_hz = self.get_parameter('poll_rate_hz').value
            self.create_timer(1.0 / poll_hz, self._poll_serial)
            self.create_timer(0.1, self._rc_watchdog)
        else:
            self.get_logger().warn(
                'require_serial=false: dry-run, no serial port opened; RC input is disabled and '
                'telemetry frames are logged instead of sent')

        telem_hz = self.get_parameter('telemetry_rate_hz').value
        if telem_hz > 0.0:
            self.create_timer(1.0 / telem_hz, self._on_telemetry_tick)

    def _make_link(self, link_mode: str):
        try:
            link = links.make_link(link_mode)
            self.get_logger().info("link_mode='%s'" % link.name)
            return link
        except KeyError:
            self.get_logger().error(
                "unknown link_mode='%s'; falling back to 'crsf'" % link_mode)
        except NotImplementedError as exc:
            self.get_logger().error('%s falling back to crsf.' % exc)
        return links.CrsfLink()

    # -- serial ----------------------------------------------------------------------------

    def _open_serial(self):
        import serial  # lazy: pyserial is a pip dep, not required to import/build the package
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud').value
        # timeout=0 -> non-blocking reads; drain in_waiting on a fast ROS2 timer instead of
        # blocking the executor.
        ser = serial.Serial(port, baudrate=baud, timeout=0)
        self.get_logger().info('%s link up on %s @ %d baud' % (self._link.name, port, baud))
        return ser

    def _poll_serial(self) -> None:
        try:
            waiting = self._serial.in_waiting
            data = self._serial.read(waiting if waiting else 1)
        except Exception as exc:  # noqa: BLE001 - a serial read error must not kill the node
            self.get_logger().error('serial read error: %s' % exc)
            return
        if not data:
            return
        for channels in self._link.decode(data):
            self._last_rc_time = self.get_clock().now()
            self._publish_joy(channels)

    def _write_frame(self, frame: bytes, what: str) -> None:
        if frame is None:
            return
        if self._serial is None:
            self.get_logger().debug('dry-run %s: %s' % (what, frame.hex()))
            return
        try:
            self._serial.write(frame)
        except Exception as exc:  # noqa: BLE001 - a dropped telemetry write must not kill the node
            self.get_logger().error('serial write error: %s' % exc)

    # -- RC channels -> Joy ----------------------------------------------------------------

    def _publish_joy(self, channels) -> None:
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = [self._link.raw_to_unit(channels[c]) if 0 <= c < len(channels) else 0.0
                    for c in self._axis_channels]
        buttons = [0] * (self._deadman_button_index + 1)
        if 0 <= self._deadman_channel < len(channels):
            on = self._link.raw_to_unit(channels[self._deadman_channel]) > self._deadman_threshold
            buttons[self._deadman_button_index] = 1 if on else 0
        msg.buttons = buttons
        self._joy_pub.publish(msg)

    def _rc_watchdog(self) -> None:
        """On RC loss (no valid frame within rc_timeout) publish a neutral Joy: zero axes and
        deadman released, so a downstream joy teleop zeroes its output -- the RC-side failsafe."""
        if self._last_rc_time is None:
            return
        stale = (self.get_clock().now() - self._last_rc_time).nanoseconds / 1e9
        if stale > self._rc_timeout:
            self._publish_joy([self._link.center()] * self._link.num_channels())

    # -- telemetry -> handset --------------------------------------------------------------

    def _on_battery(self, msg: BatteryState) -> None:
        self._battery = msg

    def _on_telemetry_tick(self) -> None:
        bat = self._battery
        if bat is None or math.isnan(bat.voltage):
            return  # nothing to report yet
        current = 0.0 if math.isnan(bat.current) else bat.current
        remaining = 0
        if not math.isnan(bat.percentage):
            remaining = max(0, min(100, int(round(bat.percentage * 100))))
        frame = self._link.encode_battery(bat.voltage, current, remaining)
        self._write_frame(frame, 'battery telemetry')


def main(args=None):
    rclpy.init(args=args)
    node = ElrsDriverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
