use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug, Default, Clone)]
pub struct AdvertiseOptions {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub latch: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub queue_size: Option<usize>,
}

pub fn get_options(profile: &str) -> AdvertiseOptions {
    defaults_for_profile(profile)
}

fn defaults_for_profile(profile: &str) -> AdvertiseOptions {
    let mut options = AdvertiseOptions::default();
    match profile {
        "transient_local" => {
            options.latch = Some(true);
        }
        _ => {}
    }
    options
}
