from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Generic, Optional, TypeVar


T = TypeVar("T")


@dataclass(frozen=True)
class ValidationResult(Generic[T]):
    ok: bool
    value: Optional[T] = None
    reason: str = ""


def validate_timer_period_s(value: Any) -> ValidationResult[float]:
    """
    Validate a timer period in seconds.
    Accepts int/float-like values and enforces > 0.0.
    """
    try:
        period_s = float(value)
    except (TypeError, ValueError):
        return ValidationResult(ok=False, reason="timer_period_s must be a number")

    if period_s <= 0.0:
        return ValidationResult(ok=False, reason="timer_period_s must be > 0.0")

    return ValidationResult(ok=True, value=period_s)


def validate_reset_max_value(value: Any) -> ValidationResult[int]:
    """
    Validate reset_max_value for stream reset tolerance.
    Enforces an integer >= 0.
    """
    try:
        reset_max = int(value)
    except (TypeError, ValueError):
        return ValidationResult(ok=False, reason="reset_max_value must be an integer")

    if reset_max < 0:
        return ValidationResult(ok=False, reason="reset_max_value must be >= 0")

    return ValidationResult(ok=True, value=reset_max)
