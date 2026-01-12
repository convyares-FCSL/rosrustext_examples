from lesson_05_parameters_py.logic import StreamEvent, TelemetryStreamValidator
from lesson_05_parameters_py.utils import validate_reset_max_value, validate_timer_period_s


def test_timer_period_accepts_float():
    r = validate_timer_period_s(0.5)
    assert r.ok is True
    assert r.value == 0.5


def test_timer_period_accepts_int():
    r = validate_timer_period_s(2)
    assert r.ok is True
    assert r.value == 2.0


def test_timer_period_rejects_zero():
    r = validate_timer_period_s(0)
    assert r.ok is False
    assert ">" in r.reason


def test_timer_period_rejects_negative():
    r = validate_timer_period_s(-1)
    assert r.ok is False


def test_timer_period_rejects_non_numeric():
    r = validate_timer_period_s("nope")
    assert r.ok is False
    assert "number" in r.reason


def test_reset_max_value_accepts_int():
    r = validate_reset_max_value(3)
    assert r.ok is True
    assert r.value == 3


def test_reset_max_value_rejects_negative():
    r = validate_reset_max_value(-1)
    assert r.ok is False
    assert ">=" in r.reason


def test_stream_initial_sets_expected():
    v = TelemetryStreamValidator(reset_max_value=1)
    d = v.on_count(10)
    assert d.event == StreamEvent.INITIAL
    assert d.expected_after == 11


def test_stream_out_of_order_is_detected():
    v = TelemetryStreamValidator(reset_max_value=1)
    v.on_count(10)  # expected 11
    d = v.on_count(9)
    assert d.event == StreamEvent.OUT_OF_ORDER
    assert d.expected_after == 11


def test_stream_reset_is_detected():
    v = TelemetryStreamValidator(reset_max_value=1)
    v.on_count(10)  # expected 11
    d = v.on_count(1)  # small value drop treated as reset
    assert d.event == StreamEvent.RESET
    assert d.expected_after == 2


def test_stream_ok_advances_expected():
    v = TelemetryStreamValidator(reset_max_value=1)
    v.on_count(1)  # expected 2
    d = v.on_count(2)
    assert d.event == StreamEvent.OK
    assert d.expected_after == 3
