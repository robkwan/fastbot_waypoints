#include <chrono>
#include <cmath>
#include <condition_variable>
#include <geometry_msgs/msg/point.hpp>
#include <gtest/gtest.h>
#include <iostream>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "fastbot_waypoints/action/waypoint.hpp"

#define ENABLE_GOOD_TESTS 0

using namespace std::chrono_literals;
using Waypoint = fastbot_waypoints::action::Waypoint;

double normalize_angle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

class WaypointTest : public ::testing::Test {
protected:
  static rclcpp::Node::SharedPtr node_;
  static rclcpp_action::Client<Waypoint>::SharedPtr client_;
  static rclcpp::CallbackGroup::SharedPtr action_cb_group_;
  static rclcpp::CallbackGroup::SharedPtr sub_cb_group_;
  static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec_;
  static std::thread spin_thread_;
  static bool executor_initialized_;

  // Non-static subscription and odometry storage
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_; // Declare sub_
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  std::mutex odom_mutex_; // Mutex for thread safety

  // Thread-safe result handling
  std::mutex result_mutex_;
  std::condition_variable result_cv_;
  bool goal_succeeded_ = false; // Track if goal succeeded
  bool got_result_ = false;
  rclcpp_action::ClientGoalHandle<Waypoint>::WrappedResult last_result_;

  static void SetUpTestSuite() {
    if (!executor_initialized_) {
      node_ = rclcpp::Node::make_shared("waypoint_test_node_unique");

      action_cb_group_ = node_->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive);
      sub_cb_group_ = node_->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive);

      client_ = rclcpp_action::create_client<Waypoint>(node_, "fastbot_as",
                                                       action_cb_group_);
      ASSERT_TRUE(client_->wait_for_action_server(5s));

      exec_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
      exec_->add_node(node_);
      spin_thread_ = std::thread([exec = exec_]() { exec->spin(); });
      executor_initialized_ = true;
    }
  }

  // New non-static method to set up subscriptions
  void setup_subscription() {
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = sub_cb_group_;

    // Define QoS profile
    rclcpp::QoS qos_profile(rclcpp::KeepLast(50)); // Keep last 10 messages
    qos_profile.reliable();                        // Use reliable communication
    // You can also set other QoS options here if needed
    // e.g., qos_profile.durability(rclcpp::DurabilityPolicy::TransientLocal);

    // Create subscription with QoS profile
    auto self = this; // Use 'this' correctly
    sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/fastbot/odom", qos_profile,
        [self](const nav_msgs::msg::Odometry::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(self->odom_mutex_);
          self->latest_odom_ = msg;
        },
        sub_options);
  }

  static void TearDownTestSuite() {
    if (executor_initialized_) {
      exec_->cancel();
      if (spin_thread_.joinable())
        spin_thread_.join();
      exec_.reset();
      client_.reset();
      node_.reset();
      executor_initialized_ = false;
    }
  }

  void SetUp() override {
    setup_subscription(); // Set up the subscription for this instance
    got_result_ = false;
    goal_succeeded_ = false; // Reset goal status
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < 2s) {
      {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (latest_odom_)
          break;
      }
      std::this_thread::sleep_for(10ms);
    }

    // Verify odometry is available
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      ASSERT_NE(latest_odom_, nullptr) << "No odometry received";
    }
  }

  void TearDown() override { latest_odom_.reset(); }

  geometry_msgs::msg::Point get_position() {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!latest_odom_) {
      RCLCPP_ERROR(node_->get_logger(), "Odometry data unavailable");
      // Return a default position (e.g., origin)
      return geometry_msgs::msg::Point();
    }
    return latest_odom_->pose.pose.position;
  }

  double get_yaw() {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    auto q = latest_odom_->pose.pose.orientation;
    double roll, pitch, yaw;
    tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 matrix(quaternion);
    matrix.getRPY(roll, pitch, yaw);
    return normalize_angle(yaw);
  }

  void send_goal(double x, double y) {
    auto goal_msg = Waypoint::Goal();
    goal_msg.position.x = x;
    goal_msg.position.y = y;

    ASSERT_TRUE(client_->wait_for_action_server(2s))
        << "Action server not available";

    rclcpp_action::Client<Waypoint>::SendGoalOptions send_goal_options;

    send_goal_options.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<Waypoint>::WrappedResult
                   &result) {
          std::lock_guard<std::mutex> lock(result_mutex_);
          last_result_ = result;
          got_result_ = true;
          goal_succeeded_ =
              (result.code == rclcpp_action::ResultCode::SUCCEEDED);
          result_cv_.notify_one();
        };

    {
      std::lock_guard<std::mutex> lock(result_mutex_);
      got_result_ = false;
      goal_succeeded_ = false; // Reset local goal status
      last_result_.code = rclcpp_action::ResultCode::UNKNOWN;
    }

    client_->async_send_goal(goal_msg, send_goal_options);

    std::unique_lock<std::mutex> lock(result_mutex_);
    if (!result_cv_.wait_for(lock, 45s, [this] { return got_result_; })) {
      auto pos = get_position();
      RCLCPP_ERROR(node_->get_logger(),
                   "Timeout! Current: (%.2f, %.2f) | Goal: (%.2f, %.2f)", pos.x,
                   pos.y, x, y);
      FAIL() << "Goal result timeout";
    }

    ASSERT_EQ(last_result_.code, rclcpp_action::ResultCode::SUCCEEDED)
        << "Action server returned failure code: "
        << static_cast<int>(last_result_.code);
  }
};

// Initialize static members
rclcpp::Node::SharedPtr WaypointTest::node_ = nullptr;
rclcpp_action::Client<Waypoint>::SharedPtr WaypointTest::client_ = nullptr;
rclcpp::CallbackGroup::SharedPtr WaypointTest::action_cb_group_ = nullptr;
rclcpp::CallbackGroup::SharedPtr WaypointTest::sub_cb_group_ = nullptr;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> WaypointTest::exec_ =
    nullptr;
std::thread WaypointTest::spin_thread_;
bool WaypointTest::executor_initialized_ = false;

#if ENABLE_GOOD_TESTS

TEST_F(WaypointTest, TestFinalPositionWithinTolerance) {
  double goal_x = 0.4, goal_y = -0.6;
  send_goal(goal_x, goal_y);

  geometry_msgs::msg::Point pos = get_position();
  double pos_tol = 0.1; // Allowable position error
  EXPECT_NEAR(pos.x, goal_x, pos_tol) << "X position out of tolerance";
  EXPECT_NEAR(pos.y, goal_y, pos_tol) << "Y position out of tolerance";
}

TEST_F(WaypointTest, TestFinalYawWithinTolerance) {
  double goal_x = 0.2, goal_y = -0.75;
  send_goal(goal_x, goal_y);

  geometry_msgs::msg::Point pos = get_position();
  double expected_yaw = std::atan2(goal_y - pos.y, goal_x - pos.x) + M_PI / 2;
  double actual_yaw = get_yaw();

  double yaw_error = std::abs(normalize_angle(expected_yaw - actual_yaw));
  double yaw_tol = 0.5; // Allowable yaw error
  EXPECT_LE(yaw_error, yaw_tol) << "Yaw angle out of tolerance";
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

#endif

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}