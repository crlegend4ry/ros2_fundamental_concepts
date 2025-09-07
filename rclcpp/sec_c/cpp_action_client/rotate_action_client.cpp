#include <memory>
#include <chrono>
#include <cstdlib>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_interfaces/action/rotate.hpp"

using namespace std::chrono_literals;
using Rotate = action_interfaces::action::Rotate;

class ActionClient : public rclcpp::Node
{
public:
  ActionClient(double angle_deg) : Node("rotate_action_client") {
    client_ = rclcpp_action::create_client<Rotate>(this, "rotate");
    angle_rad = angle_deg * M_PI / 180.0;
    send_goal();
  }

private:
  void send_goal() {
    if (!client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available.");
      return;
    }

    auto goal_msg = Rotate::Goal();
    goal_msg.angle = angle_rad;

    RCLCPP_INFO(this->get_logger(), "Sending goal: %.2f deg (%.2f rad)",
                angle_rad * 180.0 / M_PI, angle_rad);

    auto options = rclcpp_action::Client<Rotate>::SendGoalOptions();
    options.feedback_callback = [this](auto, const std::shared_ptr<const Rotate::Feedback> feedback) {
      double remaining_rad = feedback->remaining_angle;
      double remaining_deg = remaining_rad * 180.0 / M_PI;
      RCLCPP_INFO(this->get_logger(), "Remaining angle: %.2f deg (%.2f rad)",
                  remaining_deg, remaining_rad);
    };

    options.result_callback = [this](const auto &result) {
      if (result.result->success) RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
      else RCLCPP_INFO(this->get_logger(), "Goal aborted.");
      rclcpp::shutdown();
    };

    client_->async_send_goal(goal_msg, options);
  }

  rclcpp_action::Client<Rotate>::SharedPtr client_;

  double angle_rad;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  if (argc < 2 or argc > 2) {
    std::cout << "Usage: ros2 run cpp_action_client rotate_action_client <angle_in_deg>" << std::endl;
    return 1;
  }

  double angle_deg = std::stod(argv[1]);
  auto action_client = std::make_shared<ActionClient>(angle_deg);
  rclcpp::spin(action_client);
  rclcpp::shutdown();
  return 0;
}