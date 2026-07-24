"""Pure-python round-trip / vector tests for rp1_elrs.crsf (no ROS runtime needed).

Run with: pytest rp1/ros2_ws/src/rp1_elrs/test/test_crsf.py
"""

from elrs_driver import crsf


def test_crc8_known_vector():
    # CRC8/DVB-S2 of the ASCII bytes "123456789" is 0xBC (standard check value).
    assert crsf.crc8_dvb_s2(b'123456789') == 0xBC


def test_channels_round_trip():
    channels = [172, 992, 1811, 992, 1500, 200, 1000, 2000,
                172, 1811, 992, 500, 700, 1300, 1811, 172]
    packed = crsf.pack_channels(channels)
    assert len(packed) == 22  # 16 * 11 bits == 176 bits == 22 bytes
    assert crsf.unpack_channels(packed) == [c & 0x7FF for c in channels]


def test_raw_to_unit_endpoints():
    assert crsf.raw_to_unit(crsf.CRSF_CHANNEL_MID) == 0.0
    assert crsf.raw_to_unit(crsf.CRSF_CHANNEL_MIN) == -1.0
    assert crsf.raw_to_unit(crsf.CRSF_CHANNEL_MAX) == 1.0
    assert crsf.raw_to_unit(3000) == 1.0     # clamps above max
    assert crsf.raw_to_unit(0) == -1.0       # clamps below min


def test_parser_decodes_built_rc_frame():
    channels = list(range(172, 172 + crsf.NUM_RC_CHANNELS))
    frame = crsf.build_rc_channels_frame(channels)
    parser = crsf.CrsfParser()
    frames = parser.feed(frame)
    assert len(frames) == 1
    addr, ftype, payload = frames[0]
    assert addr == crsf.ADDR_FLIGHT_CONTROLLER
    assert ftype == crsf.FRAMETYPE_RC_CHANNELS_PACKED
    assert crsf.unpack_channels(payload) == channels


def test_parser_resyncs_after_garbage_and_split_reads():
    frame = crsf.build_rc_channels_frame([crsf.CRSF_CHANNEL_MID] * crsf.NUM_RC_CHANNELS)
    parser = crsf.CrsfParser()
    # leading garbage byte, then the frame delivered in two chunks
    assert parser.feed(b'\xff' + frame[:5]) == []
    frames = parser.feed(frame[5:])
    assert len(frames) == 1
    assert frames[0][1] == crsf.FRAMETYPE_RC_CHANNELS_PACKED


def test_battery_frame_is_wellformed():
    frame = crsf.build_battery_frame(voltage_dv=168, current_da=25,
                                     capacity_mah=1234, remaining_pct=90)
    parser = crsf.CrsfParser()
    frames = parser.feed(frame)
    assert len(frames) == 1
    _addr, ftype, payload = frames[0]
    assert ftype == crsf.FRAMETYPE_BATTERY_SENSOR
    assert len(payload) == 8
    voltage_dv = (payload[0] << 8) | payload[1]   # big-endian
    assert voltage_dv == 168
    assert payload[7] == 90                        # remaining percent
