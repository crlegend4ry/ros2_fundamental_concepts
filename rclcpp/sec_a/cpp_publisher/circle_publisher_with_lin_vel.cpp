#include <chrono>
#include <functional>
#include <cinttypes>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class Publisher : public rclcpp::Node {
  public:
    Publisher(double v) : Node("circle_publisher"),
                          wait_timeout_(500),
                          radius(0.5),
                          linear_vel(v)
    {
      vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(
                      "cmd_vel",
                      rclcpp::QoS(10).reliable());

      using rclcpp::contexts::get_global_default_context;
      get_global_default_context()->add_pre_shutdown_callback(
                                    [this]() {
                                      this->timer_->cancel();
                                      this->wait_for_all_acked();
                                    });

      timer_ = this->create_wall_timer(50ms, std::bind(&Publisher::timer_callback, this));
    }

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
    std::chrono::milliseconds wait_timeout_;

    double radius;
    double linear_vel;

    void wait_for_all_acked() {
      stop_robot();

      if (vel_pub->wait_for_all_acked(wait_timeout_)) {
        RCLCPP_INFO(this->get_logger(),
                          "All subscribers received the published messages.");
      } else {
        RCLCPP_INFO(this->get_logger(),
                          "Some subscribers did not receive the messages within %" PRId64 " ms.",
                          static_cast<int64_t>(wait_timeout_.count()));
      }
    }

    void timer_callback() {
      double angular_vel = linear_vel / radius;
      auto cmd = geometry_msgs::msg::Twist();
      cmd.linear.x = linear_vel;
      cmd.angular.z = angular_vel;

      RCLCPP_INFO(this->get_logger(), 
                        "Publishing[cmd_vel] >>> Linear(m/s) > 'x: %.2f' | Angular(rad/s) > 'z: %.2f'",
                        cmd.linear.x, cmd.angular.z);

      vel_pub->publish(cmd);
    }

    void stop_robot() {
      auto stop_cmd = geometry_msgs::msg::Twist();
      vel_pub->publish(stop_cmd);
      // std::cout << "Stop robot" << std::endl;
    }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  if (argc != 2) {
    std::cout << "Usage: ros2 run cpp_publisher circle_publisher <linear_vel_x>" << std::endl;
    return 1;
  }

  double v = std::stod(argv[1]);
  auto publisher = std::make_shared<Publisher>(v);
  rclcpp::spin(publisher);
  rclcpp::shutdown();
  return 0;
}