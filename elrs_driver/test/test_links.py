"""Tests for the pluggable link codecs (no ROS runtime needed).

Run with: pytest elrs_driver/test/test_links.py
"""

import pytest

from elrs_driver import crsf, links


def test_make_crsf_link_decodes_rc():
    link = links.make_link('crsf')
    assert link.name == 'crsf'
    channels = list(range(172, 172 + crsf.NUM_RC_CHANNELS))
    frame = crsf.build_rc_channels_frame(channels)
    # split across two feeds to exercise buffering
    assert link.decode(frame[:7]) == []
    updates = link.decode(frame[7:])
    assert updates == [channels]


def test_crsf_link_encode_battery_roundtrips():
    link = links.make_link('crsf')
    frame = link.encode_battery(voltage_v=16.8, current_a=-2.5, remaining_pct=90)
    _addr, ftype, payload = crsf.CrsfParser().feed(frame)[0]
    assert ftype == crsf.FRAMETYPE_BATTERY_SENSOR
    assert (payload[0] << 8 | payload[1]) == 168          # 16.8 V -> 168 dV
    assert (payload[2] << 8 | payload[3]) == 25           # abs(-2.5 A) -> 25 dA
    assert payload[7] == 90


def test_mavlink_link_is_a_documented_stub():
    with pytest.raises(NotImplementedError):
        links.make_link('mavlink')


def test_unknown_link_mode_raises_keyerror():
    with pytest.raises(KeyError):
        links.make_link('nope')
