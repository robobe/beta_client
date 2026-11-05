"""Send RC channels to an MSP receiver."""

import struct
from typing import Sequence

from beta_client.transport import Transport

MSP_V1_REQUEST = b'$M<'
MSP_SET_RAW_RC = 200
RC_CHANNEL_COUNT = 8
RC_CHANNEL_MAX = 0xFFFF


def build_rc_message(channels: Sequence[int]) -> bytes:
    """Build an MSP v1 RC frame from eight unsigned 16-bit channel values."""
    if len(channels) != RC_CHANNEL_COUNT or any(
        not isinstance(value, int) or not 0 <= value <= RC_CHANNEL_MAX
        for value in channels
    ):
        raise ValueError('Expected eight unsigned 16-bit RC channel values')
    payload = struct.pack('<8H', *channels)
    body = bytes((len(payload), MSP_SET_RAW_RC)) + payload
    checksum = 0
    for byte in body:
        checksum ^= byte
    return MSP_V1_REQUEST + body + bytes((checksum,))


def send_rc(transport: Transport, channels: Sequence[int]) -> None:
    """Send an RC frame using an open transport."""
    transport.send(build_rc_message(channels))
