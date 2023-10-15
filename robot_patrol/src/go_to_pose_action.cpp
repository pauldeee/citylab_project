#include "geometry_msgs/msg/detail/pose2_d__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_action/types.hpp"
#include "robot_patrol/action/go_to_pose.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <functional>
#include <memory>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class GoToPose : public rclcpp::Node {

public:
  using GoToPoseAction = robot_patrol::action::GoToPose;
  using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPoseAction>;

  explicit GoToPose(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("go_to_pose", options) {

    using namespace std::placeholders;
    this->actionServer_ = rclcpp_action::create_server<GoToPoseAction>(
        this, "go_to_pose", std::bind(&GoToPose::handleGoal, this, _1, _2),
        std::bind(&GoToPose::handleCancel, this, _1),
        std::bind(&GoToPose::handleAccepted, this, _1));
    twistPub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    odomSub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/odom", 10, std::bind(&GoToPose::odomCallback, this, _1));
  }

private:
  rclcpp_action::Server<GoToPoseAction>::SharedPtr actionServer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twistPub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;

  geometry_msgs::msg::Pose2D desired_pose_;
  geometry_msgs::msg::Pose2D current_pos_;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pos_.x = msg->pose.pose.orientation.x;
    current_pos_.y = msg->pose.pose.orientation.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    current_pos_.theta = yaw;
  }

  rclcpp_action::GoalResponse
  handleGoal(const rclcpp_action::GoalUUID &uuid,
             std::shared_ptr<const GoToPoseAction::Goal> goal) {

    RCLCPP_INFO(this->get_logger(), "Received goal request with x, y %d",
                goal->goal_pos.x, goal->goal_pos.y);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handleCancel(const std::shared_ptr<GoalHandleGoToPose> goalHandle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goalHandle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleGoToPose> goalHandle) {
    using namespace std::placeholders;

    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&GoToPose::execute, this, _1), goalHandle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleGoToPose> goalHandle) {
    const auto goal = goalHandle->get_goal();
    auto feedback = std::make_shared<GoToPoseAction::Feedback>();
    auto &message = feedback->current_pos;
    auto result = std::make_shared<GoToPoseAction::Result>();
    auto twist = geometry_msgs::msg::Twist();
    rclcpp::Rate loop_rate(0.2);

    // handle goal execution here.

    result->status = "The robot has reached the goal.";
    goalHandle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto actionServer = std::make_shared<GoToPose>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(actionServer);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}