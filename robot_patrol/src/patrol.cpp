#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <memory>
#include <tuple>
#include <vector>

class Patrol : public rclcpp::Node {

public:
  Patrol() : Node("robot_patrol") {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&Patrol::scanCallback, this, std::placeholders::_1));
    twist_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(200),
                                     std::bind(&Patrol::publishOdom, this));
  }

private:
  double direction_;

  geometry_msgs::msg::Twist twist_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  void publishOdom() {
    // set constant forward velocity.
    twist_.linear.x = 0.1;
    // rotate based on direction
    twist_.angular.z = direction_ / 2.0;
    twist_pub_->publish(twist_);
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    int start = std::round((msg->ranges.size() - 1) * 0.25); // -pi/2
    int end = std::round((msg->ranges.size() - 1) * 0.75);   // +pi/2

    // minimum range for considering laser ray as "clear"
    float min_range = 0.5; // meters

    double curr_area = 0;
    int curr_idx_start = -1;

    std::vector<std::tuple<int, int, double>> laserSections;

    for (int i = start; i <= end; i++) {
      if (!std::isinf(msg->ranges[i]) &&
          msg->ranges[i] > min_range) { // if range is valid

        if (curr_idx_start == -1) { // set starting idx
          curr_idx_start = i;
        }

        // sum area of laser ray
        curr_area += 0.5 * std::pow(msg->ranges[i], 2) * msg->angle_increment;

        if (i == end ||
            (i + 1 <= end && std::isinf(msg->ranges[i + 1]))) { // check for end
          laserSections.emplace_back(curr_idx_start, i, curr_area);
          curr_idx_start = -1;
          curr_area = 0;
        }

      } else if (curr_idx_start != -1) {
        laserSections.emplace_back(curr_idx_start, i - 1, curr_area);
        curr_idx_start = -1;
        curr_area = 0;
      }
    }

    // now find largest area and set direction_ to the center idx value
    double max_area;

    for (const auto &section : laserSections) {
      int start_idx = std::get<0>(section);
      int end_idx = std::get<1>(section);
      double area = std::get<2>(section);

      if (area > max_area) {
        max_area = area;
        direction_ = msg->angle_min + (start_idx + (end_idx - start_idx) / 2) *
                                          msg->angle_increment;
      }
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  return 0;
}
