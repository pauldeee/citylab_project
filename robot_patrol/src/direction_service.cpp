#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/utilities.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include <functional>
#include <iostream>
#include <memory>
#include <ostream>

using GetDirection = robot_patrol::srv::GetDirection;

class DirectionService : public rclcpp::Node {

public:
  DirectionService() : Node("direction_service") {
    srv_ = create_service<GetDirection>(
        "direction_service",
        std::bind(&DirectionService::getDirectionCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
  }

private:
  rclcpp::Service<GetDirection>::SharedPtr srv_;

  void getDirectionCallback(const std::shared_ptr<GetDirection::Request> req,
                            const std::shared_ptr<GetDirection::Response> res) {

    auto &msg = req->laser_data;

    // lidar spins CCW -- 0th index is to the rear
    int one_quater_scan = std::round(msg.ranges.size() * 0.25);
    int start = one_quater_scan;
    int end = one_quater_scan * 3;

    // cut valid laser area into thirds
    int size_of_third = (end - start) / 3;
    int right = start + size_of_third;
    int left = right + size_of_third;

    // minimum range for considering laser ray as "clear"
    float min_range = 0.5; // meters

    // split valid measurements into slices and measure their area
    double curr_area = 0;
    int curr_idx_start = -1;

    std::vector<std::tuple<int, int, double>> laserSections;

    for (int i = start; i <= end; i++) {
      if (!std::isinf(msg.ranges[i]) &&
          msg.ranges[i] > min_range) { // if range is valid
        if (curr_idx_start == -1) {    // set starting idx
          curr_idx_start = i;
        }
        // sum area of laser ray
        curr_area += 0.5 * std::pow(msg.ranges[i], 2) * msg.angle_increment;
        if (i == end ||
            (i + 1 <= end && std::isinf(msg.ranges[i + 1]))) { // check for end
          laserSections.emplace_back(curr_idx_start, i, curr_area);
          curr_idx_start = -1;
          curr_area = 0;
        }

      } else if (curr_idx_start != -1) { // invalid masurement, reset
        laserSections.emplace_back(curr_idx_start, i - 1, curr_area);
        curr_idx_start = -1;
        curr_area = 0;
      }
    }

    // now find largest area and set direction to the center idx value
    double max_area = 0;
    double direction = 0;

    for (const auto &section : laserSections) {
      int start_idx = std::get<0>(section);
      int end_idx = std::get<1>(section);
      double area = std::get<2>(section);

      if (area > max_area) {
        max_area = area;
        direction = start_idx + (end_idx - start_idx) / 2;
      }
    }

    // determine if robot should go left, right, forward
    if (direction < right) {
      res->direction = "right";
    } else if (direction >= right && direction < left) {
      res->direction = "forward";
    } else {
      res->direction = "left";
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
  return 0;
}