"""Compact 16-bit message format for the Pi-puck R&B team mission.

Packet layout:
    bits 15..13: message type (3 bits)
    bits 12..11: sender robot ID (2 bits; IDs 0..3)
    bits 10..4 : sequence field or message-specific payload (7 bits)
    bits 3..0  : CRC-4 over the upper 12 bits

LEADER_MOVING interprets the seven-bit field as LeaderMotionMode; ELECTION
uses it to advertise a robot ID; follower HEARTBEAT uses it for status.
SEARCH and RESERVED both encode as message type 7 (an IntEnum alias).
"""

from enum import IntEnum


class MessageType(IntEnum):
    """Identifiers for the eight encodable team-message values.

    RESERVED is an alias of SEARCH, not a distinct ninth wire value.
    """
    HELLO = 0
    ELECTION = 1
    LEADER_WAITING = 2
    READY = 3
    LEADER_MOVING = 4
    LEADER_STOPPED = 5
    HEARTBEAT = 6
    SEARCH = 7
    RESERVED = 7


class LeaderMotionMode(IntEnum):
    """Modes encoded in the seven-bit field of LEADER_MOVING packets."""
    SEEK = 0
    TURN_TO_WALL = 1
    AVOID = 2


class TeamMessage(object):
    """Decoded packet fields with the original 16-bit word retained."""
    def __init__(self, message_type, sender_id, sequence, raw):
        """Store a decoded 16-bit message and its raw packet word.

        The seven-bit sequence field is interpreted according to the
        message type; it is not necessarily a running counter.
        """
        self.message_type = MessageType(int(message_type))
        self.sender_id = int(sender_id)
        self.sequence = int(sequence)
        self.raw = int(raw) & 0xFFFF

    def __repr__(self):
        """Return a readable representation for debugging messages."""
        return (
            "TeamMessage(type={}, sender_id={}, sequence={}, raw=0x{:04X})"
        ).format(
            self.message_type.name,
            self.sender_id,
            self.sequence,
            self.raw,
        )


def _crc4_itu_12bit(value: int) -> int:
    """CRC-4 over exactly 12 input bits."""
    value &= 0x0FFF
    crc = 0x0

    for bit_index in range(11, -1, -1):
        incoming = (value >> bit_index) & 1
        feedback = ((crc >> 3) & 1) ^ incoming
        crc = ((crc << 1) & 0xF)

        if feedback:
            crc ^= 0x3

    return crc & 0xF


def encode_message(message_type, sender_id: int, sequence: int) -> int:
    """Pack a message type, sender ID, and seven-bit field into 16 bits.

    The upper twelve bits contain the header and the lower four bits
    contain its CRC. Value ranges are checked before encoding.

    Returns:
        Unsigned 16-bit integer suitable for the R&B payload.

    Raises:
        ValueError: Any field falls outside its allotted bit width.
    """
    message_type = int(message_type)
    sender_id = int(sender_id)
    sequence = int(sequence)

    if not 0 <= message_type <= 7:
        raise ValueError("message_type must fit in 3 bits")
    if not 0 <= sender_id <= 3:
        raise ValueError("sender_id must be between 0 and 3")
    if not 0 <= sequence <= 127:
        raise ValueError("sequence must be between 0 and 127")

    header = (
        ((message_type & 0x7) << 9)
        | ((sender_id & 0x3) << 7)
        | (sequence & 0x7F)
    )
    crc = _crc4_itu_12bit(header)
    return ((header & 0x0FFF) << 4) | crc


def decode_message(raw: int):
    """Validate the CRC and decode a 16-bit team message.

    Returns:
        TeamMessage when the four-bit CRC matches; otherwise None.

    The CRC detects some transmission errors but is not an
    authentication mechanism. The decoded seven-bit field remains
    uninterpreted until the caller considers message_type.
    """
    raw = int(raw) & 0xFFFF
    header = (raw >> 4) & 0x0FFF
    received_crc = raw & 0xF

    if _crc4_itu_12bit(header) != received_crc:
        return None

    return TeamMessage(
        message_type=(header >> 9) & 0x7,
        sender_id=(header >> 7) & 0x3,
        sequence=header & 0x7F,
        raw=raw,
    )
