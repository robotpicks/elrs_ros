"""Link codecs: the pluggable boundary between the RX serial stream and the ROS node.

A *link codec* is the only thing that knows the wire format on the ExpressLRS/receiver UART.
The node (``elrs_driver_node``) is deliberately codec-agnostic: whatever the codec, the node
always publishes ``sensor_msgs/Joy`` (RC in) and consumes ``sensor_msgs/BatteryState``
(telemetry out). Swapping the wire format is therefore just a choice of codec (`link_mode`),
with no change to the ROS interface.

    RX serial ──►┌ CrsfLink   (crsf.py)    ┐──► decode() -> RC channel updates -> Joy
                 └ MavlinkLink (pymavlink)  ┘◄── encode_battery() <- BatteryState

That is where MAVLink fits: MAVLink-over-ELRS (the receiver put in MAVLink mode, the same UART
carrying a MAVLink stream both ways) is a *second codec* here, not a rewrite of the node. It
would parse pymavlink ``RC_CHANNELS`` into the same channel lists and encode ``BATTERY_STATUS``
in place of a CRSF frame. (Sending MAVLink to a *network* ground station is a different problem
that ``mavros`` already solves — out of scope for this receiver-link driver.)

A codec implements:
  * ``decode(data: bytes) -> list[list[int]]`` — append inbound bytes, return a list of complete
    RC channel updates now available (each a list of raw channel values). Non-RC frames are the
    codec's own business (telemetry-from-RX, link stats, ...) and are simply not returned here.
  * ``encode_battery(voltage_v, current_a, remaining_pct) -> bytes | None`` — encode a battery
    telemetry frame to write back down the link, or None if this codec/link can't carry it.
"""

from elrs_driver import crsf


class LinkCodec:
    """Interface documented in the module docstring. Subclasses set ``name`` and implement both
    methods."""

    name = 'base'

    def decode(self, data: bytes):
        raise NotImplementedError

    def encode_battery(self, voltage_v: float, current_a: float, remaining_pct: int):
        raise NotImplementedError


class CrsfLink(LinkCodec):
    """CRSF (Crossfire) codec — the standard ExpressLRS / TBS Crossfire serial protocol."""

    name = 'crsf'

    def __init__(self):
        self._parser = crsf.CrsfParser()

    def decode(self, data: bytes):
        updates = []
        for _addr, frame_type, payload in self._parser.feed(data):
            if frame_type == crsf.FRAMETYPE_RC_CHANNELS_PACKED:
                channels = crsf.unpack_channels(payload)
                if len(channels) >= crsf.NUM_RC_CHANNELS:
                    updates.append(channels)
            # LINK_STATISTICS / telemetry-from-RX frames are ignored here.
        return updates

    def encode_battery(self, voltage_v: float, current_a: float, remaining_pct: int):
        return crsf.build_battery_frame(
            voltage_dv=int(round(voltage_v * 10)),
            current_da=int(round(abs(current_a) * 10)),
            capacity_mah=0,                       # used-capacity integration not tracked here
            remaining_pct=int(remaining_pct),
        )

    @staticmethod
    def raw_to_unit(raw: int) -> float:
        return crsf.raw_to_unit(raw)

    @staticmethod
    def center() -> int:
        return crsf.CRSF_CHANNEL_MID

    @staticmethod
    def num_channels() -> int:
        return crsf.NUM_RC_CHANNELS


class MavlinkLink(LinkCodec):
    """MAVLink-over-ELRS codec — placeholder for the roadmap.

    This is the natural home for MAVLink in this driver: with the receiver in MAVLink mode the
    same UART carries a MAVLink stream, so ``decode`` would turn pymavlink ``RC_CHANNELS`` into
    the same channel lists ``CrsfLink`` returns, and ``encode_battery`` would emit
    ``BATTERY_STATUS``. Because the node only ever sees Joy/BatteryState, adding this class (plus
    a ``mavlink.py`` codec using pymavlink) is the *entire* change — the node is untouched.
    """

    name = 'mavlink'

    def __init__(self):
        raise NotImplementedError(
            "link_mode='mavlink' (MAVLink-over-ELRS) is on the roadmap but not implemented yet. "
            'It slots in beside CrsfLink with no node changes — see links.py / the README. '
            "Use link_mode='crsf' for now.")


_LINKS = {CrsfLink.name: CrsfLink, MavlinkLink.name: MavlinkLink}


def make_link(link_mode: str) -> LinkCodec:
    """Construct the codec for ``link_mode``. Raises KeyError for an unknown mode and
    NotImplementedError for a known-but-unbuilt one (e.g. 'mavlink')."""
    return _LINKS[link_mode]()
