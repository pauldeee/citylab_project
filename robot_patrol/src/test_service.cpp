#include "rclcpp/client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>

using GetDirection = robot_patrol::srv::GetDirection;
using namespace std::chrono_literals;

class TestService : public rclcpp::Node {

public:
  TestService() : Node("test_service_node") {
    client_ = this->create_client<GetDirection>("direction_service");
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&TestService::scanCallback, this, std::placeholders::_1));
  }

private:
  rclcpp::Client<GetDirection>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  void handleServiceResponse(
      const rclcpp::Client<GetDirection>::SharedFuture future) {
    try {
      auto result = future.get();
      RCLCPP_INFO(this->get_logger(), "Received direction: %s",
                  result->direction.c_str());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
    }
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    auto request = std::make_shared<GetDirection::Request>();
    request->laser_data = *msg;

    if (!client_->service_is_ready()) {
      RCLCPP_WARN(this->get_logger(), "Service not available");
      return;
    }

    auto send_request_future = client_->async_send_request(
        request, std::bind(&TestService::handleServiceResponse, this,
                           std::placeholders::_1));
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestService>());
  rclcpp::shutdown();
  return 0;
}