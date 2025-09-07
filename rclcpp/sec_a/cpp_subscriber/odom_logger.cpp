#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class Subscriber : public rclcpp::Node
{
public:
    Subscriber() : Node("odom_logger") {
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, 
            std::bind(&Subscriber::odom_callback, this, std::placeholders::_1));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double roll, pitch, yaw;
        
        auto q = msg->pose.pose.orientation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 m(quat);
        m.getRPY(roll, pitch, yaw);

        RCLCPP_INFO(this->get_logger(), "Subscribing[odom] >>> Position(m) > 'x: %.2f, y: %.2f' | Angle(rad) > 'yaw: %.2f'",
                    x, y, yaw);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto subscriber = std::make_shared<Subscriber>();
    rclcpp::spin(subscriber);
    rclcpp::shutdown();
    return 0;
}