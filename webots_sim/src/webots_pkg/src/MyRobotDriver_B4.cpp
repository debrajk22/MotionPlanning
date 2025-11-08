#include "my_package/MyRobotDriver_B4.hpp"

#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <functional>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/position_sensor.h>


// #define DISTANCE_OF_WHEELS_FROM_CENTER 0.1 // Diagonal/2
#define DISTANCE_OF_WHEELS_FROM_CENTER 0.35355 // Diagonal/2
#define WHEEL_RADIUS 0.05 // Adjust to your robot's wheel radius
#define ROOT_2 1.41421356237 // Pre-calculated value of sqrt(2)
#define SAMPLING_PERIOD 25 // GPS sampling time in ms

namespace my_robot_driver_B4 {

  void MyRobotDriver::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters) {

    // Initialize all four motors
    wheel_1 = wb_robot_get_device("wheel1");
    wheel_2 = wb_robot_get_device("wheel2");
    wheel_3 = wb_robot_get_device("wheel3");
    wheel_4 = wb_robot_get_device("wheel4");

    // Set the motors to velocity control mode
    wb_motor_set_position(wheel_1, INFINITY);
    wb_motor_set_velocity(wheel_1, 0.0);
    
    wb_motor_set_position(wheel_2, INFINITY);
    wb_motor_set_velocity(wheel_2, 0.0);
    
    wb_motor_set_position(wheel_3, INFINITY);
    wb_motor_set_velocity(wheel_3, 0.0);
    
    wb_motor_set_position(wheel_4, INFINITY);
    wb_motor_set_velocity(wheel_4, 0.0);

    // Initialize GPS
    gps = wb_robot_get_device("GPS");
    wb_gps_enable(gps, SAMPLING_PERIOD);

    // Initialize IMU
    imu = wb_robot_get_device("IMU");
    wb_inertial_unit_enable(imu, SAMPLING_PERIOD);

    // Initialise encoders
    // enc_1 = wb_robot_get_device("pw1");
    // enc_2 = wb_robot_get_device("pw2");
    // enc_3 = wb_robot_get_device("pw3");
    // enc_4 = wb_robot_get_device("pw4");

    // wb_position_sensor_enable(enc_1, SAMPLING_PERIOD);
    // wb_position_sensor_enable(enc_2, SAMPLING_PERIOD);
    // wb_position_sensor_enable(enc_3, SAMPLING_PERIOD);
    // wb_position_sensor_enable(enc_4, SAMPLING_PERIOD);
    
    // Subscribe to cmd_vel to receive velocity commands
    cmd_vel_subscription_ = node->create_subscription<geometry_msgs::msg::Twist>(
        "/b4/cmd_vel", rclcpp::SensorDataQoS().reliable(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          this->cmd_vel_msg.linear = msg->linear;
          this->cmd_vel_msg.angular = msg->angular;
        }
    );

    // Publish GPS, IMU and encoder data
    b4_publisher_ = node->create_publisher<std_msgs::msg::Float32MultiArray>("b4_data", 10);

    timer_ = node->create_wall_timer(std::chrono::milliseconds(SAMPLING_PERIOD), [this]() { // Timer for periodic publishing
      wb_robot_step(SAMPLING_PERIOD);                       // Step simulation in Webots
      const double *gps_position = wb_gps_get_values(gps);  // Retrieve GPS data
      const double *imu_position = wb_inertial_unit_get_roll_pitch_yaw(imu);
      // double enc_1_position = wb_position_sensor_get_value(enc_1);
      // double enc_2_position = wb_position_sensor_get_value(enc_2);
      // double enc_3_position = wb_position_sensor_get_value(enc_3);
      // double enc_4_position = wb_position_sensor_get_value(enc_4);

      auto msg = std_msgs::msg::Float32MultiArray();        // Fill ROS2 message
      msg.data = {static_cast<float>(gps_position[0]), static_cast<float>(gps_position[1]), static_cast<float>(imu_position[2])};
      // msg.data = {static_cast<float>(gps_position[0]), static_cast<float>(gps_position[1]), static_cast<float>(imu_position[2]), static_cast<float>(enc_1_position), static_cast<float>(enc_2_position), static_cast<float>(enc_3_position), static_cast<float>(enc_4_position)};
      b4_publisher_->publish(msg);                         // Publish GPS data
    });
  }

  void MyRobotDriver::step() {
    // Get the desired forward, sideways, and rotational velocities
    auto vx = cmd_vel_msg.linear.x;  // Linear velocity in x-direction (forward)
    auto vy = cmd_vel_msg.linear.y;  // Linear velocity in y-direction (sideways)
    auto omega = cmd_vel_msg.angular.z; // Angular velocity around z-axis (rotation)

    double theta = *(wb_inertial_unit_get_roll_pitch_yaw(imu) + 2); // Get the current orientation of the robot
    double vs = vx * cos(theta) + vy * sin(theta); // Calculate the forward velocity
    double vp = -vx * sin(theta) + vy * cos(theta); // Calculate the sideways velocity

    // Compute individual wheel velocities based on omni-wheel kinematics with 45-degree orientation
    auto v1 = (1 / WHEEL_RADIUS) * ((-vs + vp) / ROOT_2 + DISTANCE_OF_WHEELS_FROM_CENTER * omega);
    auto v2 = (1 / WHEEL_RADIUS) * ((-vs - vp) / ROOT_2 + DISTANCE_OF_WHEELS_FROM_CENTER * omega);
    auto v3 = (1 / WHEEL_RADIUS) * ((vs - vp) / ROOT_2 + DISTANCE_OF_WHEELS_FROM_CENTER * omega);
    auto v4 = (1 / WHEEL_RADIUS) * ((vs + vp) / ROOT_2 + DISTANCE_OF_WHEELS_FROM_CENTER * omega);

    // Set the velocity for each motor
    wb_motor_set_velocity(wheel_1, v1);
    wb_motor_set_velocity(wheel_2, v2);
    wb_motor_set_velocity(wheel_3, v3);
    wb_motor_set_velocity(wheel_4, v4);
  }
} // namespace my_robot_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_robot_driver_B4::MyRobotDriver, webots_ros2_driver::PluginInterface)

