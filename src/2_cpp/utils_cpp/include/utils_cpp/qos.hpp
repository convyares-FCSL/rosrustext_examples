#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"

#include "utils_cpp/utils.hpp"

// Forward declaration to avoid dragging lifecycle headers into utils_cpp public headers.
namespace rclcpp_lifecycle { class LifecycleNode; }

namespace qos
{

inline constexpr char DEFAULT_PROFILE_NAME[] = "telemetry";

struct ProfileDefaults
{
  std::string reliability;
  std::string durability;
  int depth;
};

// Internal dictionary matching Python's _PROFILES
inline ProfileDefaults get_defaults_for_key(const std::string & key)
{
  if (key == "commands")            { return {"reliable",    "volatile",        1}; }
  if (key == "state_latched")       { return {"reliable",    "transient_local", 1}; }
  if (key == "events")              { return {"reliable",    "volatile",        50}; }
  if (key == "reliable_data")       { return {"reliable",    "volatile",        10}; }
  if (key == "static_data_latched") { return {"reliable",    "transient_local", 1}; }
  // Default / "telemetry"
  return {"best_effort", "volatile", 10};
}

// Small helper to avoid repeating “params + logger” plumbing everywhere.
struct NodeCtx
{
  rclcpp::node_interfaces::NodeParametersInterface & params;
  rclcpp::Logger logger;
};

inline NodeCtx make_ctx(rclcpp::Node & node)
{
  return NodeCtx{*node.get_node_parameters_interface(), node.get_logger()};
}

// Declared here; defined inline below after LifecycleNode is complete in the TU that includes it.
// This keeps qos.hpp free of lifecycle header includes.
inline NodeCtx make_ctx(rclcpp_lifecycle::LifecycleNode & node);

// Internal loader (Core)
inline rclcpp::QoS _load_profile(
  rclcpp::node_interfaces::NodeParametersInterface & params,
  const rclcpp::Logger & logger,
  std::string profile_name)
{
  std::string key = profile_name;
  
  // 1. Normalize: trim whitespace and lowercase (Python parity)
  const auto strBegin = key.find_first_not_of(" \t");
  if (strBegin == std::string::npos) {
      key = "";
  } else {
      const auto strEnd = key.find_last_not_of(" \t");
      key = key.substr(strBegin, strEnd - strBegin + 1);
  }
  std::transform(key.begin(), key.end(), key.begin(), ::tolower);

  // 2. Validate Key: If unknown, coerce to DEFAULT_PROFILE (affects parameter path!)
  // Python: key = DEFAULT_PROFILE if key not in _PROFILES
  bool is_valid = (key == "telemetry" || key == "commands" || key == "state_latched" ||
                   key == "events" || key == "reliable_data" || key == "static_data_latched");
  
  if (!is_valid) {
    key = DEFAULT_PROFILE_NAME; // "telemetry"
  }

  // 3. Load Params using valid key base
  ProfileDefaults defs = get_defaults_for_key(key);
  const std::string base = "qos.profiles." + key;

  const std::string rel = utils_cpp::get_or_declare_param(
    params, logger, base + ".reliability", std::string(defs.reliability), "qos");

  const std::string dur = utils_cpp::get_or_declare_param(
    params, logger, base + ".durability", std::string(defs.durability), "qos");

  const int depth = utils_cpp::get_or_declare_param(
    params, logger, base + ".depth", defs.depth, "qos");

  // 4. Construct QoS with Python-exact coercion (No Validation Warnings)
  rclcpp::QoS qos(depth);

  // if reliability == "best_effort": BEST_EFFORT else: RELIABLE
  if (rel == "best_effort") {
    qos.best_effort();
  } else {
    qos.reliable();
  }

  // if durability == "transient_local": TRANSIENT_LOCAL else: VOLATILE
  if (dur == "transient_local") {
    qos.transient_local();
  } else {
    qos.durability_volatile();
  }

  return qos;
}

// --- Public API (Core) ---

inline rclcpp::QoS from_parameters(
  rclcpp::node_interfaces::NodeParametersInterface & params,
  const rclcpp::Logger & logger)
{
  const std::string profile = utils_cpp::get_or_declare_param(
    params, logger, "qos.default_profile", std::string(DEFAULT_PROFILE_NAME), "qos.default_profile");
  return _load_profile(params, logger, profile);
}

inline rclcpp::QoS telemetry(
  rclcpp::node_interfaces::NodeParametersInterface & params,
  const rclcpp::Logger & logger)
{
  return _load_profile(params, logger, "telemetry");
}

inline rclcpp::QoS commands(
  rclcpp::node_interfaces::NodeParametersInterface & params,
  const rclcpp::Logger & logger)
{
  return _load_profile(params, logger, "commands");
}

inline rclcpp::QoS state_latched(
  rclcpp::node_interfaces::NodeParametersInterface & params,
  const rclcpp::Logger & logger)
{
  return _load_profile(params, logger, "state_latched");
}

inline rclcpp::QoS events(
  rclcpp::node_interfaces::NodeParametersInterface & params,
  const rclcpp::Logger & logger)
{
  return _load_profile(params, logger, "events");
}

inline rclcpp::QoS reliable_data(
  rclcpp::node_interfaces::NodeParametersInterface & params,
  const rclcpp::Logger & logger)
{
  return _load_profile(params, logger, "reliable_data");
}

inline rclcpp::QoS static_data_latched(
  rclcpp::node_interfaces::NodeParametersInterface & params,
  const rclcpp::Logger & logger)
{
  return _load_profile(params, logger, "static_data_latched");
}

// --- Adapters (thin wrappers) ---

inline rclcpp::QoS from_parameters(rclcpp::Node & node)
{
  auto ctx = make_ctx(node);
  return from_parameters(ctx.params, ctx.logger);
}

inline rclcpp::QoS telemetry(rclcpp::Node & node)
{
  auto ctx = make_ctx(node);
  return telemetry(ctx.params, ctx.logger);
}

inline rclcpp::QoS commands(rclcpp::Node & node)
{
  auto ctx = make_ctx(node);
  return commands(ctx.params, ctx.logger);
}

inline rclcpp::QoS state_latched(rclcpp::Node & node)
{
  auto ctx = make_ctx(node);
  return state_latched(ctx.params, ctx.logger);
}

inline rclcpp::QoS events(rclcpp::Node & node)
{
  auto ctx = make_ctx(node);
  return events(ctx.params, ctx.logger);
}

inline rclcpp::QoS reliable_data(rclcpp::Node & node)
{
  auto ctx = make_ctx(node);
  return reliable_data(ctx.params, ctx.logger);
}

inline rclcpp::QoS static_data_latched(rclcpp::Node & node)
{
  auto ctx = make_ctx(node);
  return static_data_latched(ctx.params, ctx.logger);
}

// Lifecycle adapters: declared in header, defined inline here without including lifecycle header.
// This works because it's only instantiated if the including TU knows LifecycleNode is complete.
// If your build complains about incomplete type, move these two overloads to a small qos_lifecycle.hpp
// that includes rclcpp_lifecycle/lifecycle_node.hpp.
inline rclcpp::QoS from_parameters(rclcpp_lifecycle::LifecycleNode & node)
{
  auto ctx = make_ctx(node);
  return from_parameters(ctx.params, ctx.logger);
}

inline rclcpp::QoS telemetry(rclcpp_lifecycle::LifecycleNode & node)
{
  auto ctx = make_ctx(node);
  return telemetry(ctx.params, ctx.logger);
}

inline rclcpp::QoS commands(rclcpp_lifecycle::LifecycleNode & node)
{
  auto ctx = make_ctx(node);
  return commands(ctx.params, ctx.logger);
}

inline rclcpp::QoS state_latched(rclcpp_lifecycle::LifecycleNode & node)
{
  auto ctx = make_ctx(node);
  return state_latched(ctx.params, ctx.logger);
}

inline rclcpp::QoS events(rclcpp_lifecycle::LifecycleNode & node)
{
  auto ctx = make_ctx(node);
  return events(ctx.params, ctx.logger);
}

inline rclcpp::QoS reliable_data(rclcpp_lifecycle::LifecycleNode & node)
{
  auto ctx = make_ctx(node);
  return reliable_data(ctx.params, ctx.logger);
}

inline rclcpp::QoS static_data_latched(rclcpp_lifecycle::LifecycleNode & node)
{
  auto ctx = make_ctx(node);
  return static_data_latched(ctx.params, ctx.logger);
}

}  // namespace qos

namespace qos
{
inline NodeCtx make_ctx(rclcpp_lifecycle::LifecycleNode & node)
{
  // We rely on LifecycleNode exposing the same parameters interface getter.
  return NodeCtx{*node.get_node_parameters_interface(), node.get_logger()};
}
}  // namespace qos
