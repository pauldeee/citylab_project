#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_action/types.hpp"
#include "robot_patrol/action/go_to_pose.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
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
    odomSub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&GoToPose::odomCallback, this, _1));

    currGoalHandle = nullptr;
  }

private:
  rclcpp_action::Server<GoToPoseAction>::SharedPtr actionServer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twistPub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;

  geometry_msgs::msg::Pose2D desired_pose_;
  geometry_msgs::msg::Pose2D current_pos_;

  std::shared_ptr<GoalHandleGoToPose> currGoalHandle;
  std::mutex goal_handle_mutex_;

  rclcpp::TimerBase::SharedPtr timer_;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pos_.x = msg->pose.pose.position.x;
    current_pos_.y = msg->pose.pose.position.y;

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

    RCLCPP_INFO(this->get_logger(),
                "Received goal request with x: %f, y: %f, theta: %f",
                goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handleCancel(const std::shared_ptr<GoalHandleGoToPose> goalHandle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goalHandle;
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    twistPub_->publish(twist);

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleGoToPose> goalHandle) {
    using namespace std::placeholders;
    std::lock_guard<std::mutex> lock(goal_handle_mutex_);
    currGoalHandle = goalHandle;

    if (timer_) {
      timer_->cancel();
      timer_ = nullptr;
    }

    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&GoToPose::publishFeedback, this));

    std::thread{std::bind(&GoToPose::execute, this, _1), goalHandle}.detach();
  }

  void publishFeedback() {
    std::lock_guard<std::mutex> lock(goal_handle_mutex_);
    if (!currGoalHandle) {
      return; // don't publish feedback if there's no active goal
    }
    auto feedback = std::make_shared<GoToPoseAction::Feedback>();
    feedback->current_pos.x = current_pos_.x;
    feedback->current_pos.y = current_pos_.y;
    feedback->current_pos.theta = current_pos_.theta;
    currGoalHandle->publish_feedback(feedback);
  }

  void execute(const std::shared_ptr<GoalHandleGoToPose> goalHandle) {
    const auto goal = goalHandle->get_goal();
    auto result = std::make_shared<GoToPoseAction::Result>();

    // FIRST -- turn towards goal point
    double point_yaw = atan2(goal->goal_pos.y - current_pos_.y,
                             goal->goal_pos.x - current_pos_.x);
    double yaw_diff = point_yaw - current_pos_.theta;
    turn(yaw_diff);

    // SECOND -- move towards goal point
    move(goal->goal_pos.x, goal->goal_pos.y);

    // THIRD -- face goal theta
    double goal_theta_radians = goal->goal_pos.theta * M_PI / 180.0;
    turn(goal_theta_radians);

    // END
    result->status = "The robot has reached the goal.";
    goalHandle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    std::lock_guard<std::mutex> lock(goal_handle_mutex_);
    timer_->cancel();
    if (currGoalHandle == goalHandle)
      currGoalHandle = nullptr;
  }

  void move(double x, double y) {
    rclcpp::Rate loop_rate(30);
    auto twist = geometry_msgs::msg::Twist();
    const double DISTANCE_TOLERANCE = 0.1;
    const double MAX_SPEED = 0.2;
    const double ACCEL = 0.01;

    double current_speed = 0.0;
    double distance =
        std::sqrt(pow((x - current_pos_.x), 2) + pow(y - current_pos_.y, 2));

    while (distance > DISTANCE_TOLERANCE) {
      double stopping_distance = (current_speed * current_speed) / (2 * ACCEL);

      // accel or decel
      if (distance > stopping_distance) {
        current_speed += ACCEL;
        if (current_speed > MAX_SPEED) {
          current_speed = MAX_SPEED;
        }
      } else {
        current_speed -= ACCEL;
        if (current_speed < 0.0) {
          current_speed = 0.0;
        }
      }

      // publish
      twist.linear.x = current_speed;
      twistPub_->publish(twist);
      distance = // update distance
          std::sqrt(pow((x - current_pos_.x), 2) + pow(y - current_pos_.y, 2));
      loop_rate.sleep();
    }

    // stop
    twist.linear.x = 0.0;
    twistPub_->publish(twist);
  }

  void turn(double target_angle) {
    auto twist = geometry_msgs::msg::Twist();
    rclcpp::Rate loop_rate(30);
    const double ANGLE_TOLERANCE = 0.025;
    const double MAX_TURN_SPEED = 0.5;

    while (true) {
      double angle_diff = target_angle - current_pos_.theta;
      if (angle_diff > M_PI)
        angle_diff -= 2.0 * M_PI;
      if (angle_diff < -M_PI)
        angle_diff += 2.0 * M_PI;

      if (fabs(angle_diff) <= ANGLE_TOLERANCE)
        break;

      double speed = std::min(MAX_TURN_SPEED, fabs(angle_diff));
      twist.angular.z = speed * (angle_diff > 0 ? 1 : -1);
      twistPub_->publish(twist);
      loop_rate.sleep();
    }

    twist.angular.z = 0.0; // stop
    twistPub_->publish(twist);
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