#include <iostream>
#include <cmath>
#include <vector>
#include <thread>
#include <mutex>

#include "Coordinate.hpp"
#include "PathPlanner.hpp"
#include "Visualize.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std;

Visualize *window = new Visualize(1600);
double runTime = 0.01;
double A = 22, B = 14;
optimalPlanner plannerType = PLANNER_RRTSTAR;
planningObjective objectiveType = OBJECTIVE_PATHLENGTH;

long long iteration_count = 0;
int flag = 0;
double obs_size = 0.7;
double max_angle = 0.6;

std::vector<Point2D> obs, path;
Point2D selfPos(0, 0, 0), finalPos(0, 0, 0), ballPos(0, 0, 0);

int findclosestpoint(std::vector<Point2D> &path, Point2D &selfPos) {
    if (path.size() == 0)
        return 0;
    double min_dist = sqrtl((path[0].x - selfPos.x) * (path[0].x - selfPos.x) + (path[0].y - selfPos.y) * (path[0].y - selfPos.y));
    int idx = 0;
    for (int i = 1; i < path.size(); i++) { // excluding last point and start
        double curr_dist = sqrtl((path[i].x - selfPos.x) * (path[i].x - selfPos.x) + (path[i].y - selfPos.y) * (path[i].y - selfPos.y));
        if (curr_dist < min_dist) {
            min_dist = curr_dist;
            idx = i;
        }
    }
    return idx;
}

// Function to check if a point lies within the extended region of a line
bool check_path(const Point2D &path_point, const Point2D &obs_point) {
    double dist = sqrt((obs_point.x - path_point.x) * (obs_point.x - path_point.x) + (obs_point.y - path_point.y) * (obs_point.y - path_point.y));
    return (dist > obs_size); // Check if the distance is within the extended region
}

class ListenerNode : public rclcpp::Node {
public:
    ListenerNode() : Node("listener_node") { // Create subscribers for two topics
        obs_subscriber = this->create_subscription<std_msgs::msg::Float32MultiArray>("/obstacles", 10, std::bind(&ListenerNode::obstacle_callback, this, std::placeholders::_1));
        self_subcriber = this->create_subscription<std_msgs::msg::Float32MultiArray>("/self_position", 10, std::bind(&ListenerNode::self_callback, this, std::placeholders::_1));
        ball_subscriber = this->create_subscription<std_msgs::msg::Float32MultiArray>("/ball_data", 10, std::bind(&ListenerNode::ball_callback, this, std::placeholders::_1));
        decision_target_subscriber = this->create_subscription<std_msgs::msg::Float32MultiArray>("/decision_target_data", 10, std::bind(&ListenerNode::decision_target_callback, this, std::placeholders::_1));
        path_publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("/planned_path", 10);
    }

    // Callback funtions
    void obstacle_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        obs_mutex.lock();
        obs.clear();
        for (int i = 1; i < 3 * msg->data[0] + 1 ; i += 3)
        {
            obs.push_back(Point2D(msg->data[i], msg->data[i + 1], msg->data[i + 2]));
        }
        obs_received = true;
        obs_mutex.unlock();
    }
    void self_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        self_mutex.lock();
        selfPos = Point2D(msg->data[0], msg->data[1], msg->data[2]);
        self_received = true;
        self_mutex.unlock();
        listening();
    }
    void ball_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        ball_mutex.lock();
        ballPos = Point2D(msg->data[0], msg->data[1], 0);
        ball_received = true;
        ball_mutex.unlock();
    }
    void decision_target_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        target_mutex.lock();
        finalPos = Point2D(msg->data[0], msg->data[1], msg->data[2]);
        target_received = true;
        target_mutex.unlock();
    }

    void listening() {
        if (!self_received || !obs_received || !ball_received || !target_received)
            return;

        // check if any obstacle has came in between the previously planned path
        for (int i = 0; i < path.size() && flag == 0; i++) {
            for (auto curr_obs : obs) {
                if (!check_path(path[i], curr_obs)) {
                    flag = 1;
                    break;
                }
            }
        }

        if (flag || iteration_count == 0 || (path.size() > 0 && (finalPos.x != path.back().x || finalPos.y != path.back().y))) {
            try {
                path.clear();
                path = plan(runTime, A, B, obs, plannerType, objectiveType, selfPos, finalPos, ballPos);
            }
            catch (const std::exception &e) {
                std::cerr << e.what() << '\n';
                std::cout << "Could not find path" << std::endl;

                path.clear();
                path.push_back(selfPos);
            }
            flag = 0;

            if (sqrt((path.back().x - finalPos.x) * (path.back().x - finalPos.x) + (path.back().y - finalPos.y) * (path.back().y - finalPos.y)) > 0.1) {
                path.clear();
                path.push_back(selfPos);
            }

            publish_path(path);
        }

        iteration_count++;
        window->visualizeGame(path, selfPos, findclosestpoint(path, selfPos), selfPos.theta, obs, ballPos);
    }

    bool self_received{false}, obs_received{false}, ball_received{false}, target_received{false}; // Variables to store received messages
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr self_subcriber, obs_subscriber, ball_subscriber, decision_target_subscriber; // Subscribers
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr path_publisher; // Publisher
    std::mutex self_mutex, obs_mutex, ball_mutex, target_mutex; // Mutexes

    void publish_path(std::vector<Point2D> &path) {
        std_msgs::msg::Float32MultiArray message;
        message.data.push_back(path.size());
        for (int i = 0; i < path.size(); i++) {
            message.data.push_back(path[i].x);
            message.data.push_back(path[i].y);
        }
        path_publisher->publish(message);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ListenerNode>());
    rclcpp::shutdown();
    return 0;
}
