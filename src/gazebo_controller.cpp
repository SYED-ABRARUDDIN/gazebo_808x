/**
 * @file gazebo_controller.cpp
 * @author Abraruddin Syed (syed029@umd.edu)
 * @brief The node that controls the turtlebot's movement based on laser scan
 * @version 1
 * @date 2023-11-29
 *
 * @copyright Copyright (c) 2023 Abraruddin syed

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 *
 */
#include <chrono>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;

/**
 * @brief The class to define the node where commands to the Turtlebot would be
 * given based on the laser scan readings
 */
class ControllerNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for the ControllerNode class to initialize the publisher and
   * subscriber
   */
  ControllerNode() : Node("controller_gazebo") {
    publisher_ = this->create_publisher
        <geometry_msgs::msg::Twist>("/cmd_vel", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&ControllerNode::subscriber_callback, this, _1));
  }

 private:
  /**
   * @brief Subscriber callback that processes laser scan readings and
   * decides whether to move forward or turn based on detected obstacles
   *
   * @param msg The laser scan readings from the Turtlebot's lidar sensor
   */
  void subscriber_callback(const sensor_msgs::msg::LaserScan& msg) {
    std::vector<float> range_values = msg.ranges;

    geometry_msgs::msg::Twist pub_vel;

    float stopping_threshold = 0.5;
    bool turn = false;

    for (size_t i = 0; i < range_values.size(); ++i) {
      if (range_values[i] < stopping_threshold) {
        turn = true;
        break;  // Exit the loop early if an obstacle is detected
      }
    }

    if (turn) {
      pub_vel.angular.z = 0.5;  // Set angular velocity for turning
    } else {
      pub_vel.linear.x = 0.3;   // Set linear velocity to move forward
    }

    publisher_->publish(pub_vel);  // Publish the velocity command
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
