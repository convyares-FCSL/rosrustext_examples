// lesson_03_subscriber.cpp

#include <cstdlib>
#include <memory>
#include <inttypes.h>

#include "lesson_03_subscriber/lesson_03_subscriber.hpp"
#include "utils_cpp/qos.hpp"
#include "utils_cpp/topics.hpp"

using lesson_interfaces::msg::MsgCount;

// ---- MessageListener ----

MessageListener::MessageListener(rclcpp::Logger logger, std::uint64_t reset_max_value) : logger_(std::move(logger)), reset_max_value_(reset_max_value) {}

void MessageListener::on_message(const MsgCount & msg) {
  const std::uint64_t count = static_cast<std::uint64_t>(msg.count);

  // Late joiners: first observed message defines the baseline.
  if (!initialized_) {
    init_from(count, true);
    return;
  }

  // Reset detection: publisher restart / manual injection.
  if (count <= reset_max_value_ && count < expected_count_) {
    init_from(count, false, true);
    return;
  }

  // Out-of-order / stale detection.
  if (count < expected_count_) {
    RCLCPP_WARN(logger_, "Out-of-order/invalid: %" PRIu64 " < %" PRIu64, count, expected_count_ );
    return;
  }

  RCLCPP_INFO(logger_, "Received: %" PRIu64, count);
  expected_count_ = count + 1;
}

void MessageListener::init_from(std::uint64_t msg_count, bool initial, bool reset)
{
  expected_count_ = msg_count + 1;
  initialized_ = true;

  if (initial) {
    RCLCPP_INFO(logger_, "Received (initial): %" PRIu64, msg_count);
  } else if (reset) {
    RCLCPP_WARN(logger_, "Detected counter reset. Re-syncing at: %" PRIu64, msg_count);
  } else {
    RCLCPP_INFO(logger_, "Received: %" PRIu64, msg_count);
  }
}

// ---- Lesson03Node ----

// Constructor
Lesson03Node::Lesson03Node(const rclcpp::NodeOptions & options) : rclcpp::Node("lesson_03_node", options)
{
  listener_ = std::make_unique<MessageListener>(this->get_logger(), 1);

  setup_subscriber();

  RCLCPP_INFO(this->get_logger(), "Lesson 03 node started (subscriber). Ctrl+C to exit.");
}

// Helper to create subscriber
void Lesson03Node::setup_subscriber()
{
  auto topic_name = topics::chatter(*this);
  auto qos_profile = qos::telemetry(*this);

  subscriber_ = this->create_subscription<MsgCount>( topic_name, qos_profile,
    [this](MsgCount::SharedPtr msg) { listener_->on_message(*msg); } // Callback must remain non-blocking; delegate to listener.
  );
}

// Main entry point
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<Lesson03Node>();
    rclcpp::spin(node);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("lesson_03_node_cpp"), "Exception in main: %s", e.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
