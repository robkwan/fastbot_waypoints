#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "fastbot_waypoints/action/waypoint.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

class FastbotActionServer : public rclcpp::Node {
public:
  using Waypoint = fastbot_waypoints::action::Waypoint;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Waypoint>;

  FastbotActionServer() : Node("fastbot_action_server") {
    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/fastbot/cmd_vel", 10);

    callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/fastbot/odom", 10,
        std::bind(&FastbotActionServer::odom_callback, this,
                  std::placeholders::_1),
        options);

    action_server_ = rclcpp_action::create_server<Waypoint>(
        this, "fastbot_as",
        std::bind(&FastbotActionServer::handle_goal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&FastbotActionServer::handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&FastbotActionServer::handle_accepted, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Fastbot Waypoint Action Server Started.");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp_action::Server<Waypoint>::SharedPtr action_server_;

  geometry_msgs::msg::Point initial_position_;
  geometry_msgs::msg::Point current_position_;
  double current_yaw_ = 0.0;

  Waypoint::Feedback feedback_;
  std::string state_ = "idle";

  const double yaw_precision_ = 0.2; // ~6 degrees
  const double dist_precision_ = 0.1;

  double normalize_angle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &,
              std::shared_ptr<const Waypoint::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal: [%.2f, %.2f]",
                goal->position.x, goal->position.y);

    // Store the initial position
    initial_position_ =
        current_position_; // Set initial position to current position

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandle>) {
    RCLCPP_INFO(this->get_logger(), "Goal canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    std::thread{std::bind(&FastbotActionServer::execute, this, goal_handle)}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<Waypoint::Result>();

    feedback_ = Waypoint::Feedback(); // Reset feedback state
    state_ = "idle";                  // Reset FSM

    geometry_msgs::msg::Point target = goal->position;

    rclcpp::Rate loop_rate(25);
    bool success = true;

    while (rclcpp::ok()) {
      // Calculate the current position relative to the initial position
      // double relative_x = current_position_.x - initial_position_.x;
      // double relative_y = current_position_.y - initial_position_.y;

      // Calculate the distance to the target
      double dx =
          target.x - current_position_.x; // Target x minus current relative x
      double dy =
          target.y -
          current_position_.y; // Target y minus current relative y double
                               // pos_err = std::sqrt(dx * dx + dy * dy);
      double pos_err = std::sqrt(dx * dx + dy * dy); // Calculate distance

      double desired_yaw = std::atan2(dy, dx);
      double yaw_error = normalize_angle(desired_yaw - current_yaw_);

      if (std::fabs(yaw_error) > M_PI / 2.0) {
        RCLCPP_WARN(this->get_logger(), "Large yaw error: %.2f rad (%.1f°)",
                    yaw_error, yaw_error * 180.0 / M_PI);
      }

      geometry_msgs::msg::Twist cmd;

      if (pos_err > dist_precision_ && success) {
        if (goal_handle->is_canceling()) {
          pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
          return;
        }

        if (std::fabs(yaw_error) > yaw_precision_) {
          cmd.angular.z = 0.5 * yaw_error;
          cmd.linear.x = 0.0;
          state_ = "fix yaw";
        } else {
          double linear_speed = std::min(0.8, 0.5 * pos_err);
          cmd.linear.x = linear_speed;
          // cmd.angular.z = (std::fabs(yaw_error) > 0.05)
          //                     ? (yaw_error > 0 ? 0.1 : -0.1)
          //                     : 0.0;
          cmd.angular.z = 0.0;
          state_ = "go to point";
        }

        pub_cmd_vel_->publish(cmd);

        feedback_.position = current_position_;
        feedback_.state = state_;
        goal_handle->publish_feedback(
            std::make_shared<Waypoint::Feedback>(feedback_));

        RCLCPP_INFO(this->get_logger(),
                    "[%s] pos_err: %.2f, yaw_err: %.2f, curr_yaw: %.2f, "
                    "des_yaw: %.2f",
                    state_.c_str(), pos_err, yaw_error, current_yaw_,
                    desired_yaw);
      } else {

        if (std::fabs(yaw_error) > yaw_precision_) {
          cmd.angular.z = 0.7 * yaw_error;
          cmd.linear.x = 0.0;
          state_ = "fix yaw";

          pub_cmd_vel_->publish(cmd);

          feedback_.position = current_position_;
          feedback_.state = state_;
          goal_handle->publish_feedback(
              std::make_shared<Waypoint::Feedback>(feedback_));

          RCLCPP_INFO(this->get_logger(),
                      "[%s] pos_err: %.2f, yaw_err: %.2f, curr_yaw: %.2f, "
                      "des_yaw: %.2f",
                      state_.c_str(), pos_err, yaw_error, current_yaw_,
                      desired_yaw);
        } else {
          break; // both pos_err and yaw_err get satisfied.
        }
      }

      loop_rate.sleep();
    }

    pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_position_ = msg->pose.pose.position;
    auto q = msg->pose.pose.orientation;
    double roll, pitch, yaw;
    tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    current_yaw_ = normalize_angle(yaw);
    RCLCPP_INFO(this->get_logger(),
                "Odom: x=%.2f, y=%.2f, yaw=%.2f rad (%.1f°)",
                current_position_.x, current_position_.y, current_yaw_,
                current_yaw_ * 180.0 / M_PI);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FastbotActionServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
