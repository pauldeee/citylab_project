#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl/publisher.h"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <asm-generic/errno.h>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

class Patrol : public rclcpp::Node {

public:
  Patrol() : Node("robot_patrol") {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&Patrol::scanCallback, this, std::placeholders::_1));
    odom_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&Patrol::publishOdom, this));
  }

private:
  double direction_;

  geometry_msgs::msg::Twist twist_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr odom_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  void publishOdom() {
    // set constant forward velocity.
    twist_.linear.x = 0.1;
    // rotate based on direction
    twist_.angular.z = direction_ / 2.0;
    odom_pub_->publish(twist_);
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    int start = std::round((msg->ranges.size() - 1) * 0.25); // -pi/2
    int end = std::round((msg->ranges.size() - 1) * 0.75);   // +pi/2
    float max_range = 0;
    int idx = 0;

    for (int i = start; i <= end; i++) {
      if (!std::isinf(msg->ranges[i]) && msg->ranges[i] > max_range) {
        max_range = msg->ranges[i];
        idx = i;
      }
    }
    direction_ = msg->angle_min + idx * msg->angle_increment;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  return 0;
}
