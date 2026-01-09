#include "lesson_cpp_template/lesson_node.hpp"

// Include helpers to encourage the "Composition" pattern
#include "lesson_cpp_template/qos.hpp"
#include "lesson_cpp_template/topics.hpp"
#include "lesson_cpp_template/services.hpp"

namespace lesson_cpp_template
{

LessonNode::LessonNode(const rclcpp::NodeOptions & options)
: rclcpp::Node(DEFAULT_NODE_NAME, options)
{
  // 1. Setup Parameters
  init_parameters();

  // 2. Setup Resources
  // << FILL IN HERE >>: Create Publishers/Subscribers using helper constants
  // publisher_ = this->create_publisher<MsgType>(topics::INPUT_TOPIC, qos::default_qos());

  // << FILL IN HERE >>: Create Timers
  // timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&LessonNode::on_timer, this));

  RCLCPP_INFO(this->get_logger(), "Node '%s' started.", this->get_name());
}

void LessonNode::init_parameters()
{
  // << FILL IN HERE >>: Declare and Get Parameters
  // this->declare_parameter<double>("my_param", 1.0);
}

/*
// << FILL IN HERE >>: Implement Callbacks
void LessonNode::on_timer() { ... }
*/

}  // namespace lesson_cpp_template


// --- Main Entry Point ---

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<lesson_cpp_template::LessonNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Fatal error: %s", e.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}