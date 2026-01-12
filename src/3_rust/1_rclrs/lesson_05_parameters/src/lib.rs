// utils.rs (or lib.rs in lesson_05_parameters_rclrs)

use lesson_interfaces::msg::MsgCount;

// --- Publisher Logic ---

/// Pure publisher business logic.
///
/// Owns the counter state and produces the next telemetry message.
/// No ROS dependencies beyond the message type, so it remains testable and reusable.
pub struct TelemetryPublisherCore {
    count: i64,
}

impl TelemetryPublisherCore {
    pub fn new() -> Self {
        Self { count: 0 }
    }

    /// Generates the next message and advances internal state.
    pub fn next_message(&mut self) -> MsgCount {
        let msg = MsgCount { count: self.count };
        self.count += 1;
        msg
    }
}

// --- Subscriber Logic ---

/// High-level stream classification for a received sample.
#[derive(Debug, PartialEq)]
pub enum StreamEvent {
    Valid,
    Reset,
    OutOfOrder,
}

/// Decision returned by the validator.
///
/// This keeps validation "pure" (no logging side effects) and allows the node layer
/// to decide how to surface events (warn/info, metrics, etc).
#[derive(Debug, PartialEq)]
pub struct StreamDecision {
    pub event: StreamEvent,
    pub message: String,
}

/// Pure stream validator.
///
/// Tracks expected sequence and tolerates small resets (e.g. publisher restart)
/// according to `reset_max_value`.
pub struct TelemetryStreamValidator {
    expected: i64,
    reset_max_value: i64,
    initialized: bool,
}

impl TelemetryStreamValidator {
    pub fn new(reset_max_value: i64) -> Self {
        Self {
            expected: 0,
            reset_max_value,
            initialized: false,
        }
    }

    /// Updates reset tolerance at runtime (control plane).
    pub fn set_reset_max_value(&mut self, val: i64) {
        self.reset_max_value = val;
    }

    /// Processes a new count sample and returns a classification decision.
    ///
    /// Behaviour:
    /// - First sample initializes the stream.
    /// - Exact match with `expected` is valid and advances the stream.
    /// - A small backwards jump (>= 0 and <= reset_max_value) is treated as a reset.
    /// - Everything else is treated as out-of-order and re-syncs expectation.
    pub fn on_count(&mut self, count: i64) -> StreamDecision {
        // Initialization: accept first sample as the start of the stream.
        if !self.initialized {
            self.expected = count + 1;
            self.initialized = true;
            return StreamDecision {
                event: StreamEvent::Valid,
                message: format!("Stream initialized at {}", count),
            };
        }

        // Common case: in-order sample.
        if count == self.expected {
            self.expected += 1;
            return StreamDecision {
                event: StreamEvent::Valid,
                message: format!("Received {}", count),
            };
        }

        // Reset tolerance: accept small restart counters (e.g. publisher reset).
        if count < self.expected && count >= 0 && count <= self.reset_max_value {
            let old_expected = self.expected;
            self.expected = count + 1;
            return StreamDecision {
                event: StreamEvent::Reset,
                message: format!(
                    "Stream reset detected: expected {}, received {}",
                    old_expected, count
                ),
            };
        }

        // Out-of-order / discontinuity: re-sync expectation to the new sample.
        let old_expected = self.expected;
        self.expected = count + 1;
        StreamDecision {
            event: StreamEvent::OutOfOrder,
            message: format!("Out of order! Received {}, expected {}", count, old_expected),
        }
    }
}

// --- Time / Configuration Helpers ---

/// Converts a period expressed in seconds (as a floating point parameter value)
/// into a quantized millisecond period.
///
/// This is intentionally strict and side-effect free:
/// - Rejects non-finite values and non-positive periods.
/// - Quantizes to milliseconds using rounding to avoid churn from tiny float noise.
///
/// The node layer decides how to surface errors (logging, parameter rejection, etc).
pub fn period_s_to_ms_strict(period_s: f64) -> Result<u64, String> {
    if !period_s.is_finite() || period_s <= 0.0 {
        return Err("timer_period_s must be finite and > 0.0".to_string());
    }

    let ms_f = (period_s * 1000.0).round();
    if !ms_f.is_finite() || ms_f <= 0.0 || ms_f > (u64::MAX as f64) {
        return Err("timer_period_s is out of supported range".to_string());
    }

    Ok(ms_f as u64)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_stream_normal() {
        let mut v = TelemetryStreamValidator::new(1);
        assert_eq!(v.on_count(10).event, StreamEvent::Valid);
        assert_eq!(v.on_count(11).event, StreamEvent::Valid);
    }

    #[test]
    fn test_stream_reset() {
        let mut v = TelemetryStreamValidator::new(5);
        v.on_count(100);
        let d = v.on_count(1);
        assert_eq!(d.event, StreamEvent::Reset);
    }

    #[test]
    fn test_stream_out_of_order() {
        let mut v = TelemetryStreamValidator::new(1);
        v.on_count(10);
        let d = v.on_count(20);
        assert_eq!(d.event, StreamEvent::OutOfOrder);
    }

    #[test]
    fn test_period_s_to_ms_strict_rejects_invalid() {
        assert!(period_s_to_ms_strict(f64::NAN).is_err());
        assert!(period_s_to_ms_strict(f64::INFINITY).is_err());
        assert!(period_s_to_ms_strict(0.0).is_err());
        assert!(period_s_to_ms_strict(-1.0).is_err());
    }

    #[test]
    fn test_period_s_to_ms_strict_quantizes() {
        // 1.2344s -> 1234ms
        assert_eq!(period_s_to_ms_strict(1.2344).unwrap(), 1234);
        // 1.2346s -> 1235ms
        assert_eq!(period_s_to_ms_strict(1.2346).unwrap(), 1235);
    }
}
