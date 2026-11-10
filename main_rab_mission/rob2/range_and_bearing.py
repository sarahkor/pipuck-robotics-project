"""Low-level I2C driver for the GCtronic Pi-puck range-and-bearing board.

The driver uses a register interface to read 16-bit message payloads and
raw bearing/range measurements. It can attempt I2C channel 12, then the
legacy channel 4 if opening the first channel fails. Range values are
hardware readings; they are not converted into physical distance here.
"""

import math
from typing import Optional

try:
    from smbus2 import SMBus
except ImportError:
    class SMBus:
        """Fallback placeholder when the smbus2 package is missing."""
        def __init__(self, channel):
            """Raise a descriptive error when smbus2 is unavailable.

            This placeholder allows importing the module on machines without
            Pi-puck hardware, but no actual bus operations can be performed.
            """
            raise RuntimeError(
                "smbus2 is required for real range-and-bearing hardware access"
            )


I2C_CHANNEL = 12
LEGACY_I2C_CHANNEL = 4
RAB_ADDR = 0x20

# Read registers.
REG_DATA_AVAILABLE = 0
REG_DATA_MSB = 1
REG_DATA_LSB = 2
REG_BEARING_MSB = 3
REG_BEARING_LSB = 4
REG_RANGE_MSB = 5
REG_RANGE_LSB = 6
REG_MAX_PEAK_MSB = 7
REG_MAX_PEAK_LSB = 8
REG_SENSOR_ID = 9

# Write registers.
REG_TX_POWER = 12
REG_TX_DATA_MSB = 13
REG_TX_DATA_LSB = 14
REG_SEND_STORED = 15
REG_CALIBRATE_LIGHT = 16
REG_CALCULATION_MODE = 17


def _join_u16(msb: int, lsb: int) -> int:
    """Combine two bytes encoded most-significant byte first."""
    return ((int(msb) & 0xFF) << 8) | (int(lsb) & 0xFF)


def _u16_to_i16(value: int) -> int:
    """Interpret an unsigned 16-bit value as a signed 16-bit value."""
    value &= 0xFFFF
    return value - 0x10000 if value & 0x8000 else value


class RangeAndBearingReading(object):
    """One packet received by the range-and-bearing extension."""

    def __init__(
        self,
        data: int,
        bearing_raw: int,
        range_raw: int,
        max_peak: int,
        sensor_id: int,
        raw: bytes,
    ) -> None:
        """Store a decoded board reading and its original register bytes.

        The bearing is converted from units of 1/10,000 radian into
        radians and degrees. The range and peak fields remain raw board
        measurements, not calibrated distances in millimeters.
        """
        self.data = int(data) & 0xFFFF
        self.bearing_raw = int(bearing_raw)
        self.bearing_radians = self.bearing_raw / 10000.0
        self.bearing_degrees = math.degrees(self.bearing_radians)
        self.range_raw = int(range_raw) & 0xFFFF
        self.max_peak = int(max_peak) & 0xFFFF
        self.sensor_id = int(sensor_id) & 0xFF
        self.raw = bytes(raw)

    @property
    def angle(self) -> float:
        """Convenience alias for bearing_degrees."""
        return self.bearing_degrees

    @property
    def distance(self) -> int:
        """Convenience alias for the board's raw range estimate."""
        return self.range_raw

    @property
    def data_bytes(self) -> bytes:
        """Return the payload as MSB, LSB."""
        return bytes(((self.data >> 8) & 0xFF, self.data & 0xFF))

    @property
    def has_signal_measurement(self) -> bool:
        """Whether range and bearing contain a usable nonzero signal."""
        return self.range_raw > 0 and self.max_peak > 0

    def __repr__(self) -> str:
        """Return a diagnostic summary of the received measurement."""
        return (
            "RangeAndBearingReading(data={}, bearing_degrees={:.3f}, "
            "bearing_radians={:.5f}, range_raw={}, max_peak={}, sensor_id={})"
        ).format(
            self.data,
            self.bearing_degrees,
            self.bearing_radians,
            self.range_raw,
            self.max_peak,
            self.sensor_id,
        )


class RangeAndBearing(object):
    """I2C driver for the GCtronic e-puck range-and-bearing extension."""

    def __init__(
        self,
        i2c_channel: Optional[int] = None,
        address: int = RAB_ADDR,
        auto_fallback: bool = True,
    ) -> None:
        """Configure the board address, I2C bus selection, and packet latch.

        Opening is lazy: a hardware bus is acquired by open(), by the
        context manager, or on the first register access. The fallback
        channel is considered only if creating the first SMBus fails.
        """
        self.address = int(address)
        self._bus = None  # type: Optional[SMBus]
        self._i2c_channel_requested = i2c_channel
        self._auto_fallback = bool(auto_fallback)

        # Register 0 pops/latches a packet in the board firmware. Cache it so
        # data_available() followed by read() does not accidentally pop twice.
        self._latched_available = None  # type: Optional[int]

    def open(self) -> None:
        """Open the selected I2C bus, optionally trying the legacy channel.

        This checks that the SMBus object can be opened; it does not
        verify that the range-and-bearing board answers at its address.

        Raises:
            RuntimeError: None of the configured bus channels can open.
        """
        if self._bus is not None:
            return

        if self._i2c_channel_requested is not None:
            channels = [self._i2c_channel_requested]
            if (
                self._auto_fallback
                and self._i2c_channel_requested != LEGACY_I2C_CHANNEL
            ):
                channels.append(LEGACY_I2C_CHANNEL)
        else:
            channels = [I2C_CHANNEL]
            if self._auto_fallback:
                channels.append(LEGACY_I2C_CHANNEL)

        last_error = None  # type: Optional[Exception]
        for channel in channels:
            try:
                self._bus = SMBus(channel)
                return
            except Exception as error:
                last_error = error
                self._bus = None

        raise RuntimeError(
            "Cannot open range-and-bearing I2C device on channels {}: {}".format(
                channels, last_error
            )
        )

    def close(self) -> None:
        """Release the I2C bus and discard any cached packet latch."""
        self._latched_available = None
        if self._bus is not None:
            try:
                self._bus.close()
            finally:
                self._bus = None

    def __enter__(self) -> "RangeAndBearing":
        """Open the I2C bus for use in a with statement."""
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        """Close the I2C bus when leaving a with statement."""
        self.close()

    def _require_bus(self) -> SMBus:
        """Return an open SMBus, opening it lazily if necessary."""
        if self._bus is None:
            self.open()
        assert self._bus is not None
        return self._bus

    def read_register(self, register: int) -> int:
        """Read one raw byte from a board register."""
        register = int(register)
        if not 0 <= register <= 255:
            raise ValueError("register must be between 0 and 255")
        return int(
            self._require_bus().read_byte_data(self.address, register)
        ) & 0xFF

    def read_bytes(self, start_register: int, length: int) -> bytes:
        """Read consecutive registers one register at a time."""
        start_register = int(start_register)
        length = int(length)
        if length < 0:
            raise ValueError("length must not be negative")
        if start_register < 0 or start_register + length > 256:
            raise ValueError("requested register range is invalid")
        return bytes(
            self.read_register(start_register + offset)
            for offset in range(length)
        )

    def read_u16(self, msb_register: int) -> int:
        """Read an unsigned 16-bit value stored MSB first."""
        msb = self.read_register(msb_register)
        lsb = self.read_register(msb_register + 1)
        return _join_u16(msb, lsb)

    def write_register(self, register: int, value: int) -> None:
        """Write one raw byte to a board register."""
        register = int(register)
        value = int(value)
        if not 0 <= register <= 255:
            raise ValueError("register must be between 0 and 255")
        if not 0 <= value <= 255:
            raise ValueError("value must be between 0 and 255")
        self._require_bus().write_byte_data(self.address, register, value)

    def data_available(self) -> bool:
        """Poll and latch the next packet without popping it twice."""
        if self._latched_available is not None:
            return True

        available = self.read_register(REG_DATA_AVAILABLE)
        if available != 0:
            self._latched_available = available
            return True
        return False

    def read(self) -> Optional[RangeAndBearingReading]:
        """Read the next latched board packet, or return None if unavailable.

        The initial availability-register access latches a packet in the
        firmware. If data_available() was called first, its cached result
        prevents a second latch/pop. The remaining registers are read in
        sequence, then converted into a RangeAndBearingReading.
        """
        if self._latched_available is None:
            available = self.read_register(REG_DATA_AVAILABLE)
        else:
            available = self._latched_available

        if available == 0:
            self._latched_available = None
            return None

        try:
            payload = self.read_bytes(REG_DATA_MSB, 9)
        finally:
            self._latched_available = None

        data = _join_u16(payload[0], payload[1])
        bearing_raw = _u16_to_i16(_join_u16(payload[2], payload[3]))
        range_raw = _join_u16(payload[4], payload[5])
        max_peak = _join_u16(payload[6], payload[7])
        sensor_id = payload[8]

        return RangeAndBearingReading(
            data=data,
            bearing_raw=bearing_raw,
            range_raw=range_raw,
            max_peak=max_peak,
            sensor_id=sensor_id,
            raw=bytes((available,)) + payload,
        )

    def send(self, data: int) -> None:
        """Write a 16-bit payload into the board's transmit-data registers.

        The method writes the most-significant then least-significant
        byte. It does not separately write REG_SEND_STORED.
        """
        data = int(data)
        if not 0 <= data <= 0xFFFF:
            raise ValueError("data must be between 0 and 65535")
        self.write_register(REG_TX_DATA_MSB, (data >> 8) & 0xFF)
        self.write_register(REG_TX_DATA_LSB, data & 0xFF)

    def set_transmission_power(self, value: int) -> None:
        """Set communication range: 0 is maximum, 255 is minimum."""
        value = int(value)
        if not 0 <= value <= 255:
            raise ValueError("transmission power must be between 0 and 255")
        self.write_register(REG_TX_POWER, value)

    def calibrate_light(self) -> None:
        """Store current ambient-light conditions as the board offset."""
        self.write_register(REG_CALIBRATE_LIGHT, 0)

    def set_host_calculation(self, enabled: bool) -> None:
        """Select onboard (False) or host-side (True) calculations."""
        self.write_register(REG_CALCULATION_MODE, 1 if enabled else 0)
