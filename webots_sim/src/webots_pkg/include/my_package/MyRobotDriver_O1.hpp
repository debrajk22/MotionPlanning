#ifndef WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP
#define WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP

#include "rclcpp/macros.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"

#include <vector>

namespace my_robot_driver_O1
{
    class MyRobotDriver : public webots_ros2_driver::PluginInterface
    {
    public:
        void step() override;
        void init(webots_ros2_driver::WebotsNode *node,
                  std::unordered_map<std::string, std::string> &parameters) override;

    private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
        geometry_msgs::msg::Twist cmd_vel_msg;

        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr o2_data_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr o3_data_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr o4_data_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr o5_data_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr b1_data_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr b2_data_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr b3_data_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr b4_data_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr b5_data_;

        std::vector<bool> data_received;
        struct RobotInfo {
            float x;
            float y;
            float theta;
        };
        std::vector<RobotInfo> robots_;

        WbDeviceTag wheel_1;
        WbDeviceTag wheel_2;
        WbDeviceTag wheel_3;
        WbDeviceTag wheel_4;

        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr o1_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr obs_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        WbDeviceTag gps;

        WbDeviceTag imu;

        // WbDeviceTag enc_1;
        // WbDeviceTag enc_2;
        // WbDeviceTag enc_3;
        // WbDeviceTag enc_4;
    };
} // namespace my_robot_driver
#endif
