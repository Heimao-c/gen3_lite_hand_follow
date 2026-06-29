#!/usr/bin/env python3
import math
import time


class LowPassFilter:
    def __init__(self, alpha, initial_value=None):
        self.alpha = float(alpha)
        self.value = initial_value

    def reset(self, value=None):
        self.value = value

    def filter(self, value, alpha=None):
        a = self.alpha if alpha is None else float(alpha)
        if self.value is None:
            self.value = value
            return value
        self.value = a * value + (1.0 - a) * self.value
        return self.value


class OneEuroFilter:
    """One Euro filter for smooth, low-latency scalar signals."""

    def __init__(self, min_cutoff=1.0, beta=0.02, d_cutoff=1.0):
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        self.x_filter = LowPassFilter(1.0)
        self.dx_filter = LowPassFilter(1.0)
        self.last_time = None
        self.last_raw = None

    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def reset(self):
        self.x_filter.reset()
        self.dx_filter.reset()
        self.last_time = None
        self.last_raw = None

    def filter(self, value, stamp=None):
        now = time.time() if stamp is None else float(stamp)
        if self.last_time is None:
            self.last_time = now
            self.last_raw = float(value)
            self.x_filter.reset(float(value))
            self.dx_filter.reset(0.0)
            return float(value)

        dt = max(1.0e-3, now - self.last_time)
        dx = (float(value) - self.last_raw) / dt
        edx = self.dx_filter.filter(dx, self._alpha(self.d_cutoff, dt))
        cutoff = self.min_cutoff + self.beta * abs(edx)
        result = self.x_filter.filter(float(value), self._alpha(cutoff, dt))

        self.last_time = now
        self.last_raw = float(value)
        return result


class VectorOneEuroFilter:
    def __init__(self, min_cutoff=1.0, beta=0.02, d_cutoff=1.0, size=3):
        self.filters = [
            OneEuroFilter(min_cutoff=min_cutoff, beta=beta, d_cutoff=d_cutoff)
            for _ in range(size)
        ]

    def reset(self):
        for f in self.filters:
            f.reset()

    def filter(self, values, stamp=None):
        return [f.filter(v, stamp=stamp) for f, v in zip(self.filters, values)]


class SlewRateLimiter:
    """Limits the first derivative of a vector command."""

    def __init__(self, max_rate, size=3):
        self.max_rate = float(max_rate)
        self.last = [0.0] * size
        self.last_time = None

    def reset(self, value=None):
        self.last = list(value) if value is not None else [0.0] * len(self.last)
        self.last_time = None

    def limit(self, values, stamp=None):
        now = time.time() if stamp is None else float(stamp)
        if self.last_time is None:
            self.last_time = now
            self.last = list(values)
            return list(values)

        dt = max(1.0e-3, now - self.last_time)
        max_delta = self.max_rate * dt
        limited = []
        for prev, target in zip(self.last, values):
            delta = max(-max_delta, min(max_delta, target - prev))
            limited.append(prev + delta)
        self.last = limited
        self.last_time = now
        return limited
