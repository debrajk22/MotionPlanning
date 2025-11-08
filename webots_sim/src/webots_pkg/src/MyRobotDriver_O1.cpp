#include "my_package/MyRobotDriver_O1.hpp"

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

namespace my_robot_driver_O1 {

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
        "/self_cmd_vel", rclcpp::SensorDataQoS().reliable(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          this->cmd_vel_msg.linear = msg->linear;
          this->cmd_vel_msg.angular = msg->angular;
        }
    );

    data_received = std::vector<bool>(9, false);
    robots_ = std::vector<RobotInfo>(9);

    // Subscribers for other robots' data
    o2_data_ = node->create_subscription<std_msgs::msg::Float32MultiArray>(
      "o2_data", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        robots_[0].x = msg->data[0];
        robots_[0].y = msg->data[1];
        robots_[0].theta = msg->data[2];
        data_received[0] = true;
    });
    o3_data_ = node->create_subscription<std_msgs::msg::Float32MultiArray>(
      "o3_data", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        robots_[1].x = msg->data[0];
        robots_[1].y = msg->data[1];
        robots_[1].theta = msg->data[2];
        data_received[1] = true;
    });
    o4_data_ = node->create_subscription<std_msgs::msg::Float32MultiArray>(
      "o4_data", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        robots_[2].x = msg->data[0];
        robots_[2].y = msg->data[1];
        robots_[2].theta = msg->data[2];
        data_received[2] = true;
    });
    o5_data_ = node->create_subscription<std_msgs::msg::Float32MultiArray>(
      "o5_data", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        robots_[3].x = msg->data[0];
        robots_[3].y = msg->data[1];
        robots_[3].theta = msg->data[2];
        data_received[3] = true;
    });
    b1_data_ = node->create_subscription<std_msgs::msg::Float32MultiArray>(
      "b1_data", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        robots_[4].x = msg->data[0];
        robots_[4].y = msg->data[1];
        robots_[4].theta = msg->data[2];
        data_received[4] = true;
    });
    b2_data_ = node->create_subscription<std_msgs::msg::Float32MultiArray>(
      "b2_data", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        robots_[5].x = msg->data[0];
        robots_[5].y = msg->data[1];
        robots_[5].theta = msg->data[2];
        data_received[5] = true;
    });
    b3_data_ = node->create_subscription<std_msgs::msg::Float32MultiArray>(
      "b3_data", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        robots_[6].x = msg->data[0];
        robots_[6].y = msg->data[1];
        robots_[6].theta = msg->data[2];
        data_received[6] = true;
    });
    b4_data_ = node->create_subscription<std_msgs::msg::Float32MultiArray>(
      "b4_data", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        robots_[7].x = msg->data[0];
        robots_[7].y = msg->data[1];
        robots_[7].theta = msg->data[2];
        data_received[7] = true;
    });
    b5_data_ = node->create_subscription<std_msgs::msg::Float32MultiArray>(
      "b5_data", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        robots_[8].x = msg->data[0];
        robots_[8].y = msg->data[1];
        robots_[8].theta = msg->data[2];
        data_received[8] = true;
    });

    // Publish GPS, IMU and encoder data
    o1_publisher_ = node->create_publisher<std_msgs::msg::Float32MultiArray>("self_position", 10);
    obs_publisher_ = node->create_publisher<std_msgs::msg::Float32MultiArray>("obstacles", 10);

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
      o1_publisher_->publish(msg);                         // Publish GPS data

      bool received_all = true;
      for (const auto& received : data_received) received_all &= received;
      if (!received_all) return;

      std_msgs::msg::Float32MultiArray obs_msg;
      obs_msg.data.push_back(robots_.size());
      for (const auto& robot : robots_) {
          obs_msg.data.push_back(robot.x);
          obs_msg.data.push_back(robot.y);
          obs_msg.data.push_back(robot.theta);
      }
      obs_publisher_->publish(obs_msg);
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
PLUGINLIB_EXPORT_CLASS(my_robot_driver_O1::MyRobotDriver, webots_ros2_driver::PluginInterface)

