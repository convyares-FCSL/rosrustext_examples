use rclrs::{DeclarationError, MandatoryParameter, Node, ParameterVariant};
use std::fmt::Debug;

/// Declares a parameter (or reuses an existing declaration) and returns a typed handle.
///
/// ROS 2 applies parameter overrides (YAML / CLI) before node creation, therefore
/// `param.get()` always reflects the effective runtime value.
/// The origin of the value (default vs override) is not inferred.
pub fn declare_parameter<T>(
    node: &Node,
    name: &str,
    default_value: T,
) -> Result<MandatoryParameter<T>, DeclarationError>
where
    T: ParameterVariant + Clone + Debug + 'static,
{
    let param = node
        .declare_parameter(name)
        .default(default_value)
        .mandatory()?;

    Ok(param)
}

// -----------------------------------------------------------------------------
// RclrsError helpers
// -----------------------------------------------------------------------------

use rclrs::{RclReturnCode, RclrsError};

/// Constructs an `RclrsError::RclError` with the provided return code.
///
/// This centralizes error construction policy and avoids repetition at call sites.
pub fn rcl_error(code: RclReturnCode) -> RclrsError {
    RclrsError::RclError { code, msg: None }
}

/// Constructs a generic RCL error using `RclReturnCode::Error`.
pub fn rcl_error_generic() -> RclrsError {
    rcl_error(RclReturnCode::Error)
}

/// Extension trait providing ergonomic conversion from `Result<T, E>`
/// into `Result<T, RclrsError>`.
///
/// This is intended for mapping rclrs API failures where the original
/// error type is not propagated.
pub trait IntoRclrsError<T> {
    fn rcl(self, code: RclReturnCode) -> Result<T, RclrsError>;
    fn rcl_generic(self) -> Result<T, RclrsError>;
}

impl<T, E> IntoRclrsError<T> for Result<T, E> {
    fn rcl(self, code: RclReturnCode) -> Result<T, RclrsError> {
        self.map_err(|_| rcl_error(code))
    }

    fn rcl_generic(self) -> Result<T, RclrsError> {
        self.rcl(RclReturnCode::Error)
    }
}
