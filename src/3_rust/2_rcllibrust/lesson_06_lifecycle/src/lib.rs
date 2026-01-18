pub mod lifecycle;

use serde::{Deserialize, Serialize};
use roslibrust::RosMessageType;

// --- MSG COUNT ---
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MsgCount {
    pub count: i64,
}

impl RosMessageType for MsgCount {
    const ROS_TYPE_NAME: &'static str = "lesson_interfaces/msg/MsgCount";
    const MD5SUM: &'static str = "ac0b03cb6e6a12df380a13eb731215bb";
}

// --- PUBLISHER LOGIC ---

#[derive(Debug)]
pub struct TelemetryPublisherCore {
    count: i64,
}

impl TelemetryPublisherCore {
    pub fn new() -> Self {
        Self { count: 0 }
    }

    pub fn next_message(&mut self) -> MsgCount {
        let msg = MsgCount { count: self.count };
        self.count += 1;
        msg
    }
}

// --- SUBSCRIBER LOGIC ---

#[derive(Debug, PartialEq)]
pub enum StreamEvent {
    Valid,
    Reset,
    OutOfOrder,
}

#[derive(Debug, PartialEq)]
pub struct StreamDecision {
    pub event: StreamEvent,
    pub message: String,
}

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

    pub fn set_reset_max_value(&mut self, val: i64) {
        self.reset_max_value = val;
    }

    pub fn on_count(&mut self, count: i64) -> StreamDecision {
        if !self.initialized {
            self.expected = count + 1;
            self.initialized = true;
            return StreamDecision {
                event: StreamEvent::Valid,
                message: format!("Stream initialized at {}", count),
            };
        }

        if count == self.expected {
            self.expected += 1;
            return StreamDecision {
                event: StreamEvent::Valid,
                message: format!("Received {}", count),
            };
        }

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

        let old_expected = self.expected;
        self.expected = count + 1;
        StreamDecision {
            event: StreamEvent::OutOfOrder,
            message: format!("Out of order! Received {}, expected {}", count, old_expected),
        }
    }
}

// --- UTILS ---

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
