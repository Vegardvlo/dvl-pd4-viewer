import os
import struct
import sys

# Ensure the project root is importable when tests are run from a subdirectory
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from pd4_viewer import checksum_ok

def build_frame(payload: bytes) -> bytes:
    header = b"\x7d\x00" + struct.pack('<H', len(payload))
    data = header + payload
    csum = sum(data) & 0xFFFF
    return data + struct.pack('<H', csum)

def test_checksum_ok_ignores_trailing_bytes():
    payload = b"\x01\x02\x03"
    frame = build_frame(payload)
    assert checksum_ok(frame, len(payload))
    frame_with_extra = frame + b"\xaa\xbb"
    assert checksum_ok(frame_with_extra, len(payload))
