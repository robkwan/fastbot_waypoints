#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <gtest/gtest.h>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sstream>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "fastbot_waypoints/action/waypoint.hpp"
#include "rcl/node.h"

#define ENABLE_GOOD_TESTS 1

using namespace std::chrono_literals;
using Waypoint = fastbot_waypoints::action::Waypoint;

double normalize_angle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

class WaypointTest : public ::testing::Test {
protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Waypoint>::SharedPtr client_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;

  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("waypoint_test_node");
    client_ = rclcpp_action::create_client<Waypoint>(node_, "fastbot_as");
    ASSERT_TRUE(client_->wait_for_action_server(5s));

    sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/fastbot/odom", 10,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) { latest_odom_ = msg; });

    // Warm up odom
    rclcpp::Rate rate(10);
    for (int i = 0; i < 20 && rclcpp::ok(); ++i) {
      rclcpp::spin_some(node_);
      rate.sleep();
    }
    ASSERT_NE(latest_odom_, nullptr) << "Did not receive odometry data";
  }

  void TearDown() override { rclcpp::shutdown(); }

  geometry_msgs::msg::Point get_position() {
    return latest_odom_->pose.pose.position;
  }

  double get_yaw() {
    auto q = latest_odom_->pose.pose.orientation;
    double roll, pitch, yaw;
    tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 matrix(quaternion);
    matrix.getRPY(roll, pitch, yaw);
    return normalize_angle(yaw);
  }

  // Custom error logger
  void LogError(const std::string &message) {
    std::cerr << "ERROR: " << message << std::endl; // Log to stderr
  }

  // Function to check if a value is within tolerance void
  void CheckWithinTolerance(double value, double expected, double tolerance) {
    if (value < expected - tolerance || value > expected + tolerance) {
      std::ostringstream oss;
      oss << "ERROR: Value " << value << " is outside the tolerance range of "
          << expected - tolerance << " to " << expected + tolerance;
      std::cerr << oss.str() << std::endl; // Log to stderr
    }
  }

  void send_goal(double x, double y) {
    auto goal_msg = Waypoint::Goal();
    goal_msg.position.x = x;
    goal_msg.position.y = y;

    auto future_result = client_->async_send_goal(goal_msg);
    ASSERT_EQ(rclcpp::spin_until_future_complete(node_, future_result),
              rclcpp::FutureReturnCode::SUCCESS);
    auto goal_handle = future_result.get();
    ASSERT_TRUE(goal_handle);

    auto result_future = client_->async_get_result(goal_handle);
    ASSERT_EQ(rclcpp::spin_until_future_complete(node_, result_future),
              rclcpp::FutureReturnCode::SUCCESS);
    ASSERT_TRUE(result_future.get().result->success);
  }
};

#if ENABLE_GOOD_TESTS

// Test 1: Final position check within tolerance
TEST_F(WaypointTest, TestFinalPositionWithinTolerance) {
  double goal_x = 0.4, goal_y = -0.6;
  send_goal(goal_x, goal_y);

  geometry_msgs::msg::Point pos = get_position();
  double pos_tol = 0.1; // Allowable position error
  EXPECT_NEAR(pos.x, goal_x, pos_tol) << "X position out of tolerance";
  EXPECT_NEAR(pos.y, goal_y, pos_tol) << "Y position out of tolerance";
}

// Test 2: Final yaw check within tolerance
TEST_F(WaypointTest, TestFinalYawWithinTolerance) {
  double goal_x = 0.2, goal_y = -0.75;
  send_goal(goal_x, goal_y);

  geometry_msgs::msg::Point pos = get_position();
  double expected_yaw = std::atan2(goal_y - pos.y, goal_x - pos.x);
  double actual_yaw = get_yaw();

  double yaw_error = normalize_angle(expected_yaw - actual_yaw);
  double yaw_tol = 0.5; // Allowable yaw error
  // CheckWithinTolerance(actual_yaw, expected_yaw, 0.1);

  EXPECT_LE(std::abs(yaw_error), yaw_tol) << "Yaw angle out of tolerance";
}

#else
/*
TEST_F(WaypointTest, SimulateDivideByZero) {
  int x = 0;
  int y = 5 / x; // Undefined behavior = potential crash
  (void)y;
}*/
/*
TEST(WaypointTest, SimulateThrowError) {
  throw std::runtime_error("Uncaught exception error!");
}
*/
/*
TEST(WaypointTest, SimulateExitFailure) {
  std::exit(EXIT_FAILURE); // Test will error, not fail
}
*/
// Test 3: Failed X outside room range
TEST_F(WaypointTest, TestTimeOutCase) {
  try {
    double goal_x = -0.1, goal_y = -0.75;
    send_goal(goal_x, goal_y);

    geometry_msgs::msg::Point pos = get_position();
    double pos_tol = 0.1;
    EXPECT_NEAR(pos.x, goal_x, pos_tol);
  } catch (const std::exception &e) {
    // FAIL() << "Exception caught: " << e.what();
    // throw std::runtime_error("Simulated uncaught error");
    std::exit(EXIT_FAILURE); // Test will error, not fail
  }
}

TEST(WaypointTest, SkippedTest) {
  GTEST_SKIP() << "Skipping this test for demonstration";
}
/*
TEST_F(WaypointTest, TestNotReady) {
  RCUTILS_LOG_WARN("Skipping test due to missing precondition.");
  SUCCEED();
}
*/

#endif // ENABLE_GOOD_TESTS

// Main function to set the gtest filter and run tests
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
