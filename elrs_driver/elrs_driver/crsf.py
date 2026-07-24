"""CRSF (Crossfire) wire-protocol codec for ExpressLRS -- pure Python, no ROS dependency.

This module is deliberately transport- and ROS-agnostic: it only turns bytes into decoded
frames and structured values back into bytes. `elrs_node.py` owns the serial port and the ROS
topics. Keeping the protocol here means it is unit-testable on its own (see test/test_crsf.py)
and that a future MAVLink codec can sit beside it without touching either the parser or the node
(the Phase-2 MAVLink-over-ELRS seam -- see rp1/CLAUDE.md).

Frame layout on the wire:

    [addr/sync] [len] [type] [payload ...] [crc8]

* ``len`` counts everything after itself: type + payload + crc  (== len(payload) + 2).
* ``crc8`` is CRC8/DVB-S2 (poly 0xD5) over [type] + [payload] (i.e. it does NOT cover
  addr or len).

References: the CRSF frame format as implemented by Betaflight/ExpressLRS. Only the handful of
frame types rp1 needs are handled here; unknown types are returned verbatim for the caller to
ignore.
"""

# --- addresses / sync bytes -------------------------------------------------
ADDR_FLIGHT_CONTROLLER = 0xC8   # sync byte used on the RX<->FC link (both directions in practice)
ADDR_RADIO_TRANSMITTER = 0xEA

# --- frame types we care about ---------------------------------------------
FRAMETYPE_GPS = 0x02
FRAMETYPE_BATTERY_SENSOR = 0x08
FRAMETYPE_LINK_STATISTICS = 0x14
FRAMETYPE_RC_CHANNELS_PACKED = 0x16
FRAMETYPE_ATTITUDE = 0x1E

# --- RC channel scaling -----------------------------------------------------
# CRSF carries 11-bit channels. These are the canonical endpoints ELRS/Betaflight use:
# 172 == 1000us == -100%, 992 == 1500us == centre, 1811 == 2000us == +100%.
CRSF_CHANNEL_MIN = 172
CRSF_CHANNEL_MID = 992
CRSF_CHANNEL_MAX = 1811
# Half-range measured from centre to max (819). The endpoints are very slightly asymmetric about
# centre (992-172=820 vs 1811-992=819); using the centre->max span keeps the gain symmetric
# about centre and lets both extremes reach full scale under clamping (the bottom clamps ~0.1%
# early, which is inaudible for a stick).
_CRSF_CHANNEL_HALF_RANGE = float(CRSF_CHANNEL_MAX - CRSF_CHANNEL_MID)  # 819.0

NUM_RC_CHANNELS = 16
_MAX_FRAME_LEN = 62  # max value of the length byte (total frame <= 64 bytes)


def crc8_dvb_s2(data) -> int:
    """CRSF CRC8 (DVB-S2, polynomial 0xD5), computed over an iterable of bytes."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def build_frame(frame_type: int, payload: bytes, addr: int = ADDR_FLIGHT_CONTROLLER) -> bytes:
    """Wrap a frame type + payload into a full CRSF frame (addr, len, type, payload, crc)."""
    body = bytes([frame_type]) + bytes(payload)   # crc covers type + payload
    length = len(body) + 1                          # + crc byte
    return bytes([addr, length]) + body + bytes([crc8_dvb_s2(body)])


def raw_to_unit(raw: int) -> float:
    """Map a raw 11-bit CRSF channel value to a normalized [-1.0, 1.0] float about centre."""
    return max(-1.0, min(1.0, (raw - CRSF_CHANNEL_MID) / _CRSF_CHANNEL_HALF_RANGE))


def unpack_channels(payload: bytes):
    """Decode an RC_CHANNELS_PACKED payload (22 bytes) into 16 raw 11-bit channel values."""
    bits = 0
    nbits = 0
    channels = []
    for byte in payload:
        bits |= byte << nbits
        nbits += 8
        while nbits >= 11 and len(channels) < NUM_RC_CHANNELS:
            channels.append(bits & 0x7FF)
            bits >>= 11
            nbits -= 11
    return channels


def pack_channels(channels) -> bytes:
    """Encode up to 16 raw 11-bit channel values into a 22-byte RC_CHANNELS_PACKED payload.

    Mainly for tests / synthetic frame generation (the RX is normally the one sending these).
    """
    bits = 0
    nbits = 0
    out = bytearray()
    for value in list(channels)[:NUM_RC_CHANNELS]:
        bits |= (int(value) & 0x7FF) << nbits
        nbits += 11
        while nbits >= 8:
            out.append(bits & 0xFF)
            bits >>= 8
            nbits -= 8
    if nbits > 0:
        out.append(bits & 0xFF)
    return bytes(out)


def build_rc_channels_frame(channels, addr: int = ADDR_FLIGHT_CONTROLLER) -> bytes:
    """Build a full RC_CHANNELS_PACKED frame (convenience wrapper, used by tests/sim)."""
    return build_frame(FRAMETYPE_RC_CHANNELS_PACKED, pack_channels(channels), addr=addr)


def build_battery_frame(voltage_dv: int, current_da: int, capacity_mah: int,
                        remaining_pct: int, addr: int = ADDR_FLIGHT_CONTROLLER) -> bytes:
    """Build a BATTERY_SENSOR telemetry frame.

    All multi-byte fields are big-endian, per CRSF telemetry convention:
      voltage_dv     uint16  battery voltage in 0.1 V units (decivolts)
      current_da     uint16  battery current in 0.1 A units (deciamps)
      capacity_mah   uint24  used capacity in mAh
      remaining_pct  uint8   estimated remaining battery percent
    """
    v = int(voltage_dv) & 0xFFFF
    i = int(current_da) & 0xFFFF
    cap = int(capacity_mah) & 0xFFFFFF
    payload = bytes([
        (v >> 8) & 0xFF, v & 0xFF,
        (i >> 8) & 0xFF, i & 0xFF,
        (cap >> 16) & 0xFF, (cap >> 8) & 0xFF, cap & 0xFF,
        int(remaining_pct) & 0xFF,
    ])
    return build_frame(FRAMETYPE_BATTERY_SENSOR, payload, addr=addr)


class CrsfParser:
    """Incremental CRSF frame parser: feed it arbitrary byte chunks, get whole frames back.

    Resynchronizes by dropping one byte at a time on a bad length or a failed CRC, so a garbled
    or partially-read stream recovers on the next valid frame boundary.
    """

    def __init__(self):
        self._buf = bytearray()

    def feed(self, data):
        """Append raw bytes and return a list of (addr, frame_type, payload) for each complete,
        CRC-valid frame now available."""
        self._buf.extend(data)
        frames = []
        while True:
            if len(self._buf) < 2:
                break
            length = self._buf[1]
            if length < 2 or length > _MAX_FRAME_LEN:
                # not a plausible frame boundary -- drop one byte and resync
                del self._buf[0]
                continue
            total = length + 2                      # addr + len + (type..crc)
            if len(self._buf) < total:
                break                               # wait for the rest of the frame
            addr = self._buf[0]
            body = bytes(self._buf[2:2 + length])   # [type][payload...][crc]
            if crc8_dvb_s2(body[:-1]) == body[-1]:
                frames.append((addr, body[0], body[1:-1]))
                del self._buf[:total]
            else:
                del self._buf[0]                    # bad CRC -- resync
        return frames
