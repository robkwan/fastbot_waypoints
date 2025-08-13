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

  FastbotActionServer() : Node("fastbot_action_server_node") {
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

    odom_first = false;

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
  bool odom_first;
  Waypoint::Feedback feedback_;
  std::string state_ = "idle";

  const double yaw_precision_ = 0.05; // ~6 degrees
  const double dist_precision_ = 0.05;

  //  double normalize_angle(double angle) {
  //    return std::atan2(std::sin(angle), std::cos(angle));
  //  }

  double normalize_angle(double angle) {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle < -M_PI)
      angle += 2.0 * M_PI;
    return angle;
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &,
              std::shared_ptr<const Waypoint::Goal> goal) {

    int cnt = 0;
    while (!odom_first) {
      cnt++;
      if (cnt > 5000) {
        RCLCPP_INFO(this->get_logger(),
                    "No odom data yet to start action server correctly.");
      }
    }

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

    feedback_ = Waypoint::Feedback();
    state_ = "idle";

    geometry_msgs::msg::Point target = goal->position;

    double dx_init = target.x - current_position_.x;
    double dy_init = target.y - current_position_.y;
    double desired_yaw_init =
        normalize_angle(std::atan2(dy_init, dx_init) + M_PI / 2);

    RCLCPP_INFO(this->get_logger(), "desired_yaw_init = %.2f",
                desired_yaw_init);

    rclcpp::Rate loop_rate(25);
    bool yaw_fixed = false;
    double yaw_error;

    const double yaw_precision_low = 0.04;
    const double yaw_precision_high = 0.07;
    const double dist_precision_ = 0.05;

    while (rclcpp::ok()) {
      double dx = target.x - current_position_.x;
      double dy = target.y - current_position_.y;
      double pos_err = std::sqrt(dx * dx + dy * dy);

      // double desired_yaw_dynamic = std::atan2(dy, dx);
      double desired_yaw_dynamic =
          normalize_angle(std::atan2(dy, dx) + M_PI / 2);

      geometry_msgs::msg::Twist cmd;

      if (pos_err > dist_precision_) {
        // Use initial yaw until close to goal
        yaw_error = normalize_angle(desired_yaw_init - current_yaw_);
        double abs_yaw_error = std::fabs(yaw_error);

        if (!yaw_fixed) {
          if (abs_yaw_error < yaw_precision_low) {
            yaw_fixed = true;
            RCLCPP_INFO(this->get_logger(), "Yaw fixed!");
          }
          state_ = "fix yaw";
          cmd.linear.x = 0.0;
          cmd.angular.z = 0.5 * yaw_error;
          RCLCPP_INFO(this->get_logger(), "Fixing yaw: error=%.3f", yaw_error);
        } else {
          state_ = "go to point";
          double linear_speed = 0.5 * pos_err;
          if (pos_err < 0.3) {
            linear_speed *= 0.5;
          }
          cmd.linear.x = std::clamp(linear_speed, 0.1, 0.8);
          cmd.angular.z = std::clamp(0.5 * yaw_error, -1.0, 1.0);

          RCLCPP_INFO(this->get_logger(), "desired_yaw_init = %.2f",
                      desired_yaw_init);
          RCLCPP_INFO(this->get_logger(), "Yaw error = %.2f, Pos error = %.2f",
                      yaw_error, pos_err);
          RCLCPP_INFO(this->get_logger(),
                      "Odom: x=%.2f, y=%.2f, yaw=%.2f rad (%.1f°)",
                      current_position_.x, current_position_.y, current_yaw_,
                      current_yaw_ * 180.0 / M_PI);
        }
      } else {
        // Final yaw alignment when close to goal
        yaw_error = normalize_angle(desired_yaw_dynamic - current_yaw_);
        double abs_yaw_error = std::fabs(yaw_error);

        if (abs_yaw_error > yaw_precision_low) {
          state_ = "final yaw align";
          cmd.linear.x = 0.0;
          cmd.angular.z = 0.5 * yaw_error;
          RCLCPP_INFO(this->get_logger(), "Aligning final yaw...");
        } else {
          break; // Goal reached
        }
      }

      pub_cmd_vel_->publish(cmd);

      if (goal_handle->is_canceling()) {
        pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      feedback_.position = current_position_;
      feedback_.state = state_;
      goal_handle->publish_feedback(
          std::make_shared<Waypoint::Feedback>(feedback_));

      loop_rate.sleep();
    }

    pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    RCLCPP_INFO(this->get_logger(),
                "Odom: x=%.2f, y=%.2f, yaw=%.2f rad (%.1f°)",
                current_position_.x, current_position_.y, current_yaw_,
                current_yaw_ * 180.0 / M_PI);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!odom_first) {
      odom_first = true;
      RCLCPP_INFO(this->get_logger(), "Fastbot Odometry data ready.");
    }

    current_position_ = msg->pose.pose.position;
    auto q = msg->pose.pose.orientation;
    double roll, pitch, yaw;
    tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    current_yaw_ = normalize_angle(yaw);
    /*RCLCPP_INFO(this->get_logger(),
                "Odom: x=%.2f, y=%.2f, yaw=%.2f rad (%.1f°)",
                current_position_.x, current_position_.y, current_yaw_,
                current_yaw_ * 180.0 / M_PI);*/
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
