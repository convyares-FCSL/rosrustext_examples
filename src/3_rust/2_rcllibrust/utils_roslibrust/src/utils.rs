// utils_roslibrust/src/utils.rs

use serde::de::DeserializeOwned;
use serde_yaml::{Mapping, Value};
use std::fs::File;
use std::io::Read;
use std::path::{Path, PathBuf};

/// Minimal config object: holds the merged `ros__parameters` mapping.
#[derive(Debug, Clone)]
pub struct Config {
    params: Value, // always Value::Mapping
}

#[derive(Debug)]
pub enum ConfigError {
    Io(std::io::Error),
    Yaml(serde_yaml::Error),
    InvalidFormat(&'static str),
}

impl From<std::io::Error> for ConfigError {
    fn from(e: std::io::Error) -> Self {
        ConfigError::Io(e)
    }
}

impl From<serde_yaml::Error> for ConfigError {
    fn from(e: serde_yaml::Error) -> Self {
        ConfigError::Yaml(e)
    }
}

impl Config {
    /// Load and merge multiple ROS2-style params YAML files.
    pub fn from_files<P: AsRef<Path>>(paths: &[P]) -> Result<Self, ConfigError> {
        let mut merged_params = Value::Mapping(Mapping::new());

        for p in paths {
            let doc = read_yaml(p.as_ref())?;
            let extracted = extract_ros_parameters(doc)?;
            deep_merge(&mut merged_params, extracted);
        }

        if !matches!(merged_params, Value::Mapping(_)) {
            return Err(ConfigError::InvalidFormat(
                "internal error: merged params not a mapping",
            ));
        }

        Ok(Self {
            params: merged_params,
        })
    }

    /// Generic typed fetch. Uses dot-path keys like `topics.telemetry`.
    pub fn get<T: DeserializeOwned>(&self, key: &str) -> Option<T> {
        let v = get_by_dot_path(&self.params, key)?;
        serde_yaml::from_value::<T>(v.clone()).ok()
    }

    /// Typed fetch with default.
    pub fn get_or<T: DeserializeOwned>(&self, key: &str, default: T) -> T {
        self.get(key).unwrap_or(default)
    }
}

/// Helper: Parses `--params-file <path>` arguments from std::env::args.
pub fn parse_params_files_from_args() -> Vec<PathBuf> {
    let args: Vec<String> = std::env::args().collect();
    let mut files = Vec::new();
    let mut i = 0;
    while i < args.len() {
        if args[i] == "--params-file" && i + 1 < args.len() {
            files.push(PathBuf::from(&args[i + 1]));
            i += 1;
        }
        i += 1;
    }
    files
}

// ------------------------- internal helpers -------------------------

fn read_yaml(path: &Path) -> Result<Value, ConfigError> {
    let mut f = File::open(path)?;
    let mut s = String::new();
    f.read_to_string(&mut s)?;
    let v: Value = serde_yaml::from_str(&s)?;
    Ok(v)
}

fn extract_ros_parameters(doc: Value) -> Result<Value, ConfigError> {
    let mut out = Value::Mapping(Mapping::new());
    match doc {
        Value::Mapping(root) => {
            if let Some(p) = root.get(&Value::String("ros__parameters".to_string())) {
                deep_merge(&mut out, p.clone());
                return ensure_mapping(out);
            }
            for (_k, v) in root {
                if let Value::Mapping(node_block) = v {
                    if let Some(p) = node_block.get(&Value::String("ros__parameters".to_string())) {
                        deep_merge(&mut out, p.clone());
                    }
                }
            }
            ensure_mapping(out)
        }
        _ => Err(ConfigError::InvalidFormat(
            "params file root must be a YAML mapping",
        )),
    }
}

fn ensure_mapping(v: Value) -> Result<Value, ConfigError> {
    if matches!(v, Value::Mapping(_)) {
        Ok(v)
    } else {
        Err(ConfigError::InvalidFormat(
            "ros__parameters must be a YAML mapping",
        ))
    }
}

fn deep_merge(dst: &mut Value, src: Value) {
    match (dst, src) {
        (Value::Mapping(dst_map), Value::Mapping(src_map)) => {
            for (k, v) in src_map {
                match dst_map.get_mut(&k) {
                    Some(existing) => deep_merge(existing, v),
                    None => {
                        dst_map.insert(k, v);
                    }
                }
            }
        }
        (dst_scalar, src_scalar) => {
            *dst_scalar = src_scalar;
        }
    }
}

fn get_by_dot_path<'a>(root: &'a Value, key: &str) -> Option<&'a Value> {
    let mut cur = root;
    if key.is_empty() {
        return Some(cur);
    }
    for part in key.split('.') {
        match cur {
            Value::Mapping(m) => {
                cur = m.get(&Value::String(part.to_string()))?;
            }
            _ => return None,
        }
    }
    Some(cur)
}