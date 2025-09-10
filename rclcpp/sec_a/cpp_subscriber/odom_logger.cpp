#include <functional>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class Subscriber : public rclcpp::Node {
    public:
        Subscriber() : Node("odom_logger") {
            odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, 
                                std::bind(&Subscriber::odom_callback, this, std::placeholders::_1));
        }

    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            double x = msg->pose.pose.position.x;
            double y = msg->pose.pose.position.y;
            double yaw;
        
            auto q = msg->pose.pose.orientation;
            yaw = std::atan2(2.0 * ((q.w * q.z) + (q.x * q.y)), 
                                1.0 - (2.0 * ((q.y * q.y) + (q.z * q.z))));

            RCLCPP_INFO(this->get_logger(), 
                                "Subscribing[odom] >>> Position(m) > 'x: %.2f, y: %.2f' | Angle(rad) > 'yaw: %.2f'",
                                x, y, yaw);
        }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto subscriber = std::make_shared<Subscriber>();
    rclcpp::spin(subscriber);
    rclcpp::shutdown();
    return 0;
}