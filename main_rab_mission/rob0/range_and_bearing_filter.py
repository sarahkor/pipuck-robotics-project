"""Rolling filters and measurement summaries for Pi-puck R&B readings.

Only readings with both nonzero range and nonzero peak are added to the
rolling windows. Bearings are summarized in degrees, while range and
peak values remain in the board's raw units. Sample freshness is tracked
separately from snapshot construction.
"""

import time
from collections import deque
from statistics import median
from typing import Optional

from range_and_bearing import RangeAndBearingReading


def wrap_degrees(angle: float) -> float:
    """Normalize an angle to [-180, 180)."""
    return ((float(angle) + 180.0) % 360.0) - 180.0


def circular_median(angles) -> float:
    """Compute a median after unwrapping angles around the last sample.

    This handles a cluster of angles straddling -180/+180 degrees.
    It is not a general circular-median optimizer for arbitrary
    multimodal angle distributions.
    """
    if not angles:
        raise ValueError("angles must not be empty")

    reference = float(angles[-1])
    unwrapped = [
        reference + wrap_degrees(float(angle) - reference)
        for angle in angles
    ]
    return wrap_degrees(median(unwrapped))


class FilteredRangeAndBearingReading(object):
    """Filtered snapshot built from several valid hardware readings."""

    def __init__(
        self,
        bearing_degrees: float,
        range_median: float,
        peak_median: float,
        weak_ratio: float,
        strong_ratio: float,
        sample_count: int,
        age_seconds: float,
        latest_data: int,
    ) -> None:
        """Store rolling-window summaries and the newest received payload.

        Range and peak statistics retain their respective hardware units.
        age_seconds is elapsed time since the last accepted sample.
        """
        self.bearing_degrees = float(bearing_degrees)
        self.range_median = float(range_median)
        self.peak_median = float(peak_median)
        self.weak_ratio = float(weak_ratio)
        self.strong_ratio = float(strong_ratio)
        self.sample_count = int(sample_count)
        self.age_seconds = float(age_seconds)
        self.latest_data = int(latest_data) & 0xFFFF

    @property
    def angle(self) -> float:
        """Return the filtered bearing in degrees."""
        return self.bearing_degrees

    def __repr__(self) -> str:
        """Format the filtered measurement statistics for diagnostics."""
        return (
            "FilteredRangeAndBearingReading("
            "bearing_degrees={:.2f}, range_median={:.1f}, "
            "peak_median={:.1f}, weak_ratio={:.2f}, "
            "strong_ratio={:.2f}, sample_count={}, age_seconds={:.3f})"
        ).format(
            self.bearing_degrees,
            self.range_median,
            self.peak_median,
            self.weak_ratio,
            self.strong_ratio,
            self.sample_count,
            self.age_seconds,
        )


class RangeAndBearingFilter(object):
    """Rolling robust filter for bearing and raw signal measurements.

    The filter intentionally does not claim that range_raw is centimetres.
    """

    def __init__(
        self,
        window_size: int = 9,
        min_samples: int = 5,
        stale_after: float = 1.0,
        weak_threshold: int = 200,
        strong_threshold: int = 500,
    ) -> None:
        """Configure rolling sample windows and signal-quality thresholds.

        Args:
            window_size: Maximum number of recent accepted samples.
            min_samples: Samples required before snapshot() can return data.
            stale_after: Maximum age, in seconds, for is_fresh().
            weak_threshold: Raw range below which a sample counts as weak.
            strong_threshold: Raw range at or above which it counts as strong.
        """
        window_size = int(window_size)
        min_samples = int(min_samples)

        if window_size < 1:
            raise ValueError("window_size must be at least 1")
        if not 1 <= min_samples <= window_size:
            raise ValueError("min_samples must be between 1 and window_size")
        if stale_after <= 0:
            raise ValueError("stale_after must be positive")

        self.window_size = window_size
        self.min_samples = min_samples
        self.stale_after = float(stale_after)
        self.weak_threshold = int(weak_threshold)
        self.strong_threshold = int(strong_threshold)

        self._angles = deque(maxlen=window_size)
        self._ranges = deque(maxlen=window_size)
        self._peaks = deque(maxlen=window_size)
        self._last_valid_time = None  # type: Optional[float]
        self._latest_data = 0

    def clear(self) -> None:
        """Discard all recorded measurements and reset freshness state."""
        self._angles.clear()
        self._ranges.clear()
        self._peaks.clear()
        self._last_valid_time = None
        self._latest_data = 0

    def add(
        self,
        reading: Optional[RangeAndBearingReading],
        now: Optional[float] = None,
    ) -> bool:
        """Add one hardware reading.

        Returns True only when the reading contributed a usable signal sample.
        Payload validation belongs to range_and_bearing_protocol.py.
        """
        if reading is None or not reading.has_signal_measurement:
            return False

        if now is None:
            now = time.monotonic()

        self._angles.append(reading.bearing_degrees)
        self._ranges.append(reading.range_raw)
        self._peaks.append(reading.max_peak)
        self._last_valid_time = float(now)
        self._latest_data = reading.data
        return True

    def is_fresh(self, now: Optional[float] = None) -> bool:
        """Check whether the newest valid sample is within stale_after.

        Freshness uses monotonic time by default. A snapshot may still be
        returned after this check becomes false; callers must check it.
        """
        if self._last_valid_time is None:
            return False
        if now is None:
            now = time.monotonic()
        return float(now) - self._last_valid_time <= self.stale_after

    def snapshot(
        self,
        now: Optional[float] = None,
    ) -> Optional[FilteredRangeAndBearingReading]:
        """Build filtered statistics when at least min_samples are available.

        Returns:
            A snapshot containing medians, signal ratios, and the age
            of the last accepted sample; otherwise None.

        This method does not reject stale samples. Call is_fresh() (or
        examine age_seconds) when freshness is required.
        """
        if len(self._ranges) < self.min_samples:
            return None

        if now is None:
            now = time.monotonic()

        assert self._last_valid_time is not None
        ranges = list(self._ranges)
        peaks = list(self._peaks)
        angles = list(self._angles)

        weak_count = sum(
            1 for value in ranges if value < self.weak_threshold
        )
        strong_count = sum(
            1 for value in ranges if value >= self.strong_threshold
        )
        count = len(ranges)

        return FilteredRangeAndBearingReading(
            bearing_degrees=circular_median(angles),
            range_median=median(ranges),
            peak_median=median(peaks),
            weak_ratio=weak_count / float(count),
            strong_ratio=strong_count / float(count),
            sample_count=count,
            age_seconds=float(now) - self._last_valid_time,
            latest_data=self._latest_data,
        )
