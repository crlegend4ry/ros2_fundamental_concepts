#include <cmath>
#include <memory>
#include <chrono>
#include <vector>

#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

class ServiceServer : public rclcpp::Node {
    public:
        ServiceServer() : Node("square_service_server"),
                          waypoint_index(0),
                          goal_x(0.0),
                          goal_y(0.0),
                          yaw(0.0),
                          linear_speed(0.22),
                          angular_speed(0.3),
                          goal_tolerance(0.03),
                          goal_reached(true)
        {
            service_ = this->create_service<std_srvs::srv::Empty>("square_service",
                                std::bind(&ServiceServer::handle_service, this,
                                    std::placeholders::_1,
                                    std::placeholders::_2,
                                    std::placeholders::_3));

            vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10,
                                std::bind(&ServiceServer::odom_callback, this, 
                                    std::placeholders::_1));

            timer_ = this->create_wall_timer(50ms, std::bind(&ServiceServer::move_robot, this));
        }

        void set_waypoints(const std::vector<std::pair<double, double>> &points) {
            waypoints = points;
        }

    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_;
        rclcpp::TimerBase::SharedPtr timer_;

        nav_msgs::msg::Odometry::SharedPtr odom_msg;

        std::vector<std::pair<double, double>> waypoints;
        size_t waypoint_index;

        double goal_x, goal_y, yaw;
        double linear_speed, angular_speed;
        double goal_tolerance;
        bool goal_reached;

        void handle_service(
            const std::shared_ptr<rmw_request_id_t>,
            const std::shared_ptr<std_srvs::srv::Empty::Request>,
            const std::shared_ptr<std_srvs::srv::Empty::Response>)
        {
            if (!odom_msg) {
                RCLCPP_WARN(this->get_logger(), "No odometry yet.");
                return;
            }

            if (waypoints.empty()) {
                RCLCPP_WARN(this->get_logger(), "No waypoints defined.");
                return;
            }

            waypoint_index = 0;
            goal_x = waypoints[waypoint_index].first;
            goal_y = waypoints[waypoint_index].second;
            goal_reached = false;

            RCLCPP_INFO(this->get_logger(), "Starting square path");
        }

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            odom_msg = msg;
            
            auto q = msg->pose.pose.orientation;
            yaw = std::atan2(2.0 * ((q.w * q.z) + (q.x * q.y)), 
                                1.0 - (2.0 * ((q.y * q.y) + (q.z * q.z))));
        }

        void move_robot() {
            if (!odom_msg || goal_reached)
                return;

            auto cmd = geometry_msgs::msg::Twist();

            double current_x = odom_msg->pose.pose.position.x;
            double current_y = odom_msg->pose.pose.position.y;

            double dx = goal_x - current_x;
            double dy = goal_y - current_y;
            double distance = std::sqrt((dx * dx) + (dy * dy));

            double target_yaw = std::atan2(dy, dx);
            double yaw_error = normalize_angle(target_yaw - yaw);

            /* For debugging output */
            
            // printf(">>> Linear Distance <<<\n");
            // printf("gx, gy: '%.2lf, %.2lf' | cx, cy: '%.2lf, %.2lf'", goal_x, goal_y, current_x, current_y);
            // printf(" | dx, dy: '%.2lf, %.2lf'\n", dx, dy);
            // printf("distance: %.2lf\n\n", distance);

            // printf(">>> Angular Distance <<<\n");
            // printf("target_yaw: %.2lf, yaw: %.2lf, yaw_error: %.2lf\n\n", yaw, target_yaw, yaw_error);

            if (distance > goal_tolerance) {
                if (std::abs(yaw_error) > 0.1) {
                    // std::cout << "Rotate" << std::endl;
                    if (yaw_error > 0) cmd.angular.z = angular_speed;
                    else cmd.angular.z = -angular_speed;
                } else {
                    // std::cout << "Move straight" << std::endl;
                    cmd.linear.x = linear_speed;
                }
            } else {
                waypoint_index++;
                if (waypoint_index < waypoints.size()) {
                    // std::cout << "Next waypoint" << std::endl;
                    goal_x = waypoints[waypoint_index].first;
                    goal_y = waypoints[waypoint_index].second;
                } else {
                    RCLCPP_INFO(this->get_logger(), "Completed square path");
                    goal_reached = true;
                    stop_robot();
                }
            }
            
            vel_pub->publish(cmd);
        }

        void stop_robot() {
            auto stop_cmd = geometry_msgs::msg::Twist();
            vel_pub->publish(stop_cmd);
            // std::cout << "Stop robot" << std::endl;
        }

        double normalize_angle(double angle) {
            angle = std::fmod(angle + M_PI, 2 * M_PI);
            if (angle < 0) angle += 2 * M_PI;
            return angle - M_PI;
        }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServiceServer>();

    node->set_waypoints({{0.5, 0.0},
                         {0.5, 0.5},
                         {0.0, 0.5},
                         {0.0, 0.0}});

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}