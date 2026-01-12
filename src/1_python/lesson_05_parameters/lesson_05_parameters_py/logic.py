from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class StreamEvent(str, Enum):
    INITIAL = "initial"
    RESET = "reset"
    OUT_OF_ORDER = "out_of_order"
    OK = "ok"


@dataclass
class StreamDecision:
    event: StreamEvent
    count: int
    expected_after: int
    message: str


class TelemetryStreamValidator:
    """
    Validates a monotonically increasing counter stream with reset tolerance.

    Policy:
      - First message initializes expected.
      - If count drops to a small value (<= reset_max_value) and is < expected -> treat as reset.
      - If count < expected (and not a reset) -> out-of-order/stale sample.
      - Otherwise accept and advance expected.
    """

    __slots__ = ("_initialized", "_expected", "_reset_max_value")

    def __init__(self, *, reset_max_value: int = 1) -> None:
        self._initialized = False
        self._expected = 0
        self._reset_max_value = int(reset_max_value)

    @property
    def expected(self) -> int:
        return self._expected

    @property
    def reset_max_value(self) -> int:
        return self._reset_max_value

    def set_reset_max_value(self, value: int) -> None:
        self._reset_max_value = int(value)

    def on_count(self, count: int) -> StreamDecision:
        c = int(count)

        if not self._initialized:
            self._initialized = True
            self._expected = c + 1
            return StreamDecision(
                event=StreamEvent.INITIAL,
                count=c,
                expected_after=self._expected,
                message=f"Received (initial): {c}",
            )

        if c <= self._reset_max_value and c < self._expected:
            self._expected = c + 1
            return StreamDecision(
                event=StreamEvent.RESET,
                count=c,
                expected_after=self._expected,
                message=f"Detected counter reset. Re-syncing at: {c}",
            )

        if c < self._expected:
            return StreamDecision(
                event=StreamEvent.OUT_OF_ORDER,
                count=c,
                expected_after=self._expected,
                message=f"Out-of-order/invalid: {c} < {self._expected}",
            )

        self._expected = c + 1
        return StreamDecision(
            event=StreamEvent.OK,
            count=c,
            expected_after=self._expected,
            message=f"Received: {c}",
        )
