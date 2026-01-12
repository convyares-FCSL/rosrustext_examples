//! Lesson 05 – Pure business logic & Message Definitions.

use serde::{Deserialize, Serialize};
use roslibrust::RosMessageType;

// --- MESSAGE DEFINITION ---
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MsgCount {
    pub count: i64,
}

// IMPLEMENTATION REQUIRED BY ROSLIBRUST 0.18+
impl RosMessageType for MsgCount {
    // This string must match the topic type expected by ROS 2 nodes
    const ROS_TYPE_NAME: &'static str = "lesson_interfaces/msg/MsgCount";
    // MD5 is required by the trait, but Rosbridge (JSON) often ignores it.
    // We provide a placeholder (or the real one if known) to satisfy the compiler.
    const MD5SUM: &'static str = "ac0b03cb6e6a12df380a13eb731215bb"; 
}

// --- BUSINESS LOGIC ---

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StreamEvent {
    Valid,
    Reset,
    OutOfOrder,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StreamDecision {
    pub event: StreamEvent,
    pub message: String,
}

#[derive(Debug)]
pub struct TelemetryStreamValidator {
    expected: i64,
    reset_max_value: i64,
}

impl TelemetryStreamValidator {
    pub fn new(reset_max_value: i64) -> Self {
        Self {
            expected: 0,
            reset_max_value: reset_max_value.max(0),
        }
    }

    pub fn set_reset_max_value(&mut self, v: i64) {
        self.reset_max_value = v.max(0);
    }

    pub fn on_count(&mut self, count: i64) -> StreamDecision {
        if self.expected == 0 && count != 0 {
            self.expected = count + 1;
            return StreamDecision {
                event: StreamEvent::Valid,
                message: format!("Start stream: count={count}"),
            };
        }
        if count == self.expected {
            self.expected += 1;
            return StreamDecision {
                event: StreamEvent::Valid,
                message: format!("OK: count={count}"),
            };
        }
        if count >= 0 && count <= self.reset_max_value {
            self.expected = count + 1;
            return StreamDecision {
                event: StreamEvent::Reset,
                message: format!("RESET detected: count={count}"),
            };
        }
        let expected = self.expected;
        self.expected = count + 1;
        StreamDecision {
            event: StreamEvent::OutOfOrder,
            message: format!("OUT-OF-ORDER: expected={expected}, got={count}"),
        }
    }
}

pub fn period_s_to_ms_strict(period_s: f64) -> Result<u64, String> {
    if !period_s.is_finite() || period_s <= 0.0 {
        return Err("Period must be > 0".into());
    }
    let ms = period_s * 1000.0;
    if ms < 1.0 {
        return Err("Period too small".into());
    }
    Ok(ms.round() as u64)
}

#[derive(Debug)]
pub struct TelemetryPublisherCore {
    count: i64,
}

impl TelemetryPublisherCore {
    pub fn new() -> Self {
        Self { count: 0 }
    }
    pub fn next_count(&mut self) -> i64 {
        self.count += 1;
        self.count
    }
}