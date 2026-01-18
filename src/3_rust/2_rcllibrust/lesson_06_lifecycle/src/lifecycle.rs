use serde::{Deserialize, Serialize};
use roslibrust::{RosMessageType, RosServiceType};

// --- LIFECYCLE MESSAGES ---

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransitionMsg {
    pub id: u8,
    pub label: String,
}

impl RosMessageType for TransitionMsg {
    const ROS_TYPE_NAME: &'static str = "lifecycle_msgs/msg/Transition";
    const MD5SUM: &'static str = "";
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StateMsg {
    pub id: u8,
    pub label: String,
}

impl RosMessageType for StateMsg {
    const ROS_TYPE_NAME: &'static str = "lifecycle_msgs/msg/State";
    const MD5SUM: &'static str = "";
}

// --- PARAMETER MESSAGES ---

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ParameterValue {
    #[serde(rename = "type")]
    pub type_: u8,
    pub bool_value: bool,
    pub integer_value: i64,
    pub double_value: f64,
    pub string_value: String,
    pub byte_array_value: Vec<u8>,
    pub bool_array_value: Vec<bool>,
    pub integer_array_value: Vec<i64>,
    pub double_array_value: Vec<f64>,
    pub string_array_value: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ParameterMsg {
    pub name: String,
    pub value: ParameterValue,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ParameterEvent {
    pub node: String,
    pub new_parameters: Vec<ParameterMsg>,
    pub changed_parameters: Vec<ParameterMsg>,
    pub deleted_parameters: Vec<ParameterMsg>,
}

impl RosMessageType for ParameterEvent {
    const ROS_TYPE_NAME: &'static str = "rcl_interfaces/msg/ParameterEvent";
    const MD5SUM: &'static str = "";
}

// --- SERVICES ---

// ChangeState
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChangeStateRequest {
    pub transition: TransitionMsg,
}
impl RosMessageType for ChangeStateRequest {
    const ROS_TYPE_NAME: &'static str = "lifecycle_msgs/srv/ChangeState";
    const MD5SUM: &'static str = ""; 
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChangeStateResponse {
    pub success: bool,
}
impl RosMessageType for ChangeStateResponse {
    const ROS_TYPE_NAME: &'static str = "lifecycle_msgs/srv/ChangeState";
    const MD5SUM: &'static str = ""; 
}

pub struct ChangeState;
impl RosServiceType for ChangeState {
    const ROS_SERVICE_NAME: &'static str = "lifecycle_msgs/srv/ChangeState";
    type Request = ChangeStateRequest;
    type Response = ChangeStateResponse;
}

// GetState
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GetStateRequest {}
impl RosMessageType for GetStateRequest {
    const ROS_TYPE_NAME: &'static str = "lifecycle_msgs/srv/GetState";
    const MD5SUM: &'static str = ""; 
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GetStateResponse {
    pub current_state: StateMsg,
}
impl RosMessageType for GetStateResponse {
    const ROS_TYPE_NAME: &'static str = "lifecycle_msgs/srv/GetState";
    const MD5SUM: &'static str = ""; 
}

pub struct GetState;
impl RosServiceType for GetState {
    const ROS_SERVICE_NAME: &'static str = "lifecycle_msgs/srv/GetState";
    type Request = GetStateRequest;
    type Response = GetStateResponse;
}

// SetParameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SetParametersResult {
    pub successful: bool,
    pub reason: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SetParametersRequest {
    pub parameters: Vec<ParameterMsg>,
}
impl RosMessageType for SetParametersRequest {
    const ROS_TYPE_NAME: &'static str = "rcl_interfaces/srv/SetParameters";
    const MD5SUM: &'static str = ""; 
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SetParametersResponse {
    pub results: Vec<SetParametersResult>,
}
impl RosMessageType for SetParametersResponse {
    const ROS_TYPE_NAME: &'static str = "rcl_interfaces/srv/SetParameters";
    const MD5SUM: &'static str = ""; 
}

pub struct SetParameters;
impl RosServiceType for SetParameters {
    const ROS_SERVICE_NAME: &'static str = "rcl_interfaces/srv/SetParameters";
    type Request = SetParametersRequest;
    type Response = SetParametersResponse;
}

// --- CONSTANTS ---

pub const STATE_UNCONFIGURED: u8 = 1;
pub const STATE_INACTIVE: u8 = 2;
pub const STATE_ACTIVE: u8 = 3;
pub const STATE_FINALIZED: u8 = 4;

pub const TRANSITION_CONFIGURE: u8 = 1;
pub const TRANSITION_CLEANUP: u8 = 2;
pub const TRANSITION_ACTIVATE: u8 = 3;
pub const TRANSITION_DEACTIVATE: u8 = 4;
pub const TRANSITION_SHUTDOWN: u8 = 100;

// --- HELPER LOGIC ---

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LifecycleState {
    Unconfigured,
    Inactive,
    Active,
    Finalized,
}

impl LifecycleState {
    pub fn id(&self) -> u8 {
        match self {
            Self::Unconfigured => STATE_UNCONFIGURED,
            Self::Inactive => STATE_INACTIVE,
            Self::Active => STATE_ACTIVE,
            Self::Finalized => STATE_FINALIZED,
        }
    }

    pub fn label(&self) -> &'static str {
        match self {
            Self::Unconfigured => "unconfigured",
            Self::Inactive => "inactive",
            Self::Active => "active",
            Self::Finalized => "finalized",
        }
    }
    
    pub fn to_msg(&self) -> StateMsg {
        StateMsg {
            id: self.id(),
            label: self.label().to_string(),
        }
    }
}
