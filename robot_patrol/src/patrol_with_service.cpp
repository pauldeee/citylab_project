#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <memory>
#include <tuple>
#include <vector>

using GetDirection = robot_patrol::srv::GetDirection;

class Patrol : public rclcpp::Node {

public:
  Patrol() : Node("robot_patrol_with_service") {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&Patrol::scanCallback, this, std::placeholders::_1));
    twist_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(200),
                                     std::bind(&Patrol::publishOdom, this));
    client_ = this->create_client<robot_patrol::srv::GetDirection>(
        "direction_service");
  }

private:
  sensor_msgs::msg::LaserScan::SharedPtr last_laser_;

  geometry_msgs::msg::Twist twist_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;

  void responseCallback(rclcpp::Client<GetDirection>::SharedFuture future) {
    auto response = future.get();

    RCLCPP_INFO(this->get_logger(), "Received direction: %s",
                response->direction.c_str());

    // set constant forward velocity.
    twist_.linear.x = 0.1;

    if (response->direction == "left") {
      twist_.angular.z = 0.5;
    } else if (response->direction == "forward") {
      twist_.angular.z = 0.0;
    } else {
      twist_.angular.z = -0.5;
    }

    // publish twist
    twist_pub_->publish(twist_);
  }
  void publishOdom() {
    // service request
    auto request = std::make_shared<GetDirection::Request>();
    request->laser_data = *last_laser_;

    // send the service request and attach a callback
    auto response_future = client_->async_send_request(
        request,
        std::bind(&Patrol::responseCallback, this, std::placeholders::_1));
  }
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    last_laser_ = msg;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}
