#include "pid_maze_solver/goals.hpp"
#include "pid_maze_solver/goals_yaml.hpp"

#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"
#include <filesystem>

#include <limits>
#include <stdexcept>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>

using Quaternion = geometry_msgs::msg::Quaternion;
using Odometry = nav_msgs::msg::Odometry;
using Twist = geometry_msgs::msg::Twist;

using namespace std::chrono_literals;

namespace PidMazeSolver {

double getYaw(const Quaternion &quaternion) {
  tf2::Quaternion q{quaternion.x, quaternion.y, quaternion.z, quaternion.w};
  double pitch{};
  double roll{};
  double yaw{};
  tf2::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
  return yaw;
}

class PidMazeSolver : public rclcpp::Node {
public:
  PidMazeSolver(const std::string &node_name = kNodeName,
                const rclcpp::NodeOptions &options = rclcpp::NodeOptions{})
      : Node{node_name, options}, odom_sub_{this->create_subscription<Odometry>(
                                      kOdometryTopicName, 1,
                                      std::bind(&PidMazeSolver::odomSubCb, this,
                                                std::placeholders::_1))},
        twist_pub_{
            this->create_publisher<Twist>(kCommandVelocityTopicName, 1)} {
    RCLCPP_INFO(this->get_logger(), "%s node started.", node_name.c_str());
  }

  bool followTrajectory(GoalSequence goals);

private:
  struct State {
    double x;
    double y;
    double theta;
    double dx;
    double dy;
    double dtheta;
  };

  constexpr static char kCommandVelocityTopicName[]{"cmd_vel"};
  constexpr static char kNodeName[]{"distance_controller"};
  constexpr static char kOdometryTopicName[]{"/odometry/filtered"};

  constexpr static double kAngleTolerance{0.02};    // [rad]
  constexpr static double kPositionTolerance{0.01}; // [m]
  constexpr static auto kControlCycle{100ms};
  constexpr static double kHeadingIGain{0.01};
  constexpr static double kHeadingPGain{2.0};
  constexpr static double kHeadingDGain{0.3};
  constexpr static double kPositionIGain{0.05};
  constexpr static double kPositionPGain{1.7};
  constexpr static double kPositionDGain{0.7};

  double pidStepPosition(double e, double de, double ie);
  double pidStepHeading(double e, double de, double ie);

  void odomSubCb(const std::shared_ptr<const Odometry> msg);
  bool goToGoal(const Point2D &goal);

  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_{};
  rclcpp::Publisher<Twist>::SharedPtr twist_pub_{};

  std::optional<struct State> state_cur_{std::nullopt};
};

double PidMazeSolver::pidStepPosition(double e, double de, double ie) {
  return kPositionPGain * e + kPositionDGain * de + kPositionIGain * ie;
}

double PidMazeSolver::pidStepHeading(double e, double de, double ie) {
  return kHeadingPGain * e + kHeadingDGain * de + kHeadingIGain * ie;
}

void PidMazeSolver::odomSubCb(const std::shared_ptr<const Odometry> msg) {
  state_cur_ = {msg->pose.pose.position.x,          msg->pose.pose.position.y,
                getYaw(msg->pose.pose.orientation), msg->twist.twist.linear.x,
                msg->twist.twist.linear.y,          msg->twist.twist.angular.z};
}

bool PidMazeSolver::goToGoal(const Point2D &point) {
  RCLCPP_INFO(this->get_logger(), "Received new goal: x=%f y=%f", point.x,
              point.y);

  while (!state_cur_) {
    RCLCPP_WARN(this->get_logger(), "Odometry not received yet, waiting...");
    rclcpp::sleep_for(1s);
  }

  double ie_s{0.0}, ie_theta{0.0};
  double e_s{std::numeric_limits<double>::infinity()};
  bool heading_reached{false};

  while (e_s > kPositionTolerance) {
    Twist v_d{};

    // heading loop
    auto theta_goal{
        std::atan2(point.y - state_cur_->y, point.x - state_cur_->x)};
    double e_theta{std::atan2(std::sin(theta_goal - state_cur_->theta),
                              std::cos(theta_goal - state_cur_->theta))};
    double de_theta{/* dtheta_goal = 0 */ -state_cur_->dtheta};
    v_d.angular.z = pidStepHeading(e_theta, de_theta, ie_theta);
    ie_theta += std::chrono::duration<double>{kControlCycle}.count() * e_theta;
    heading_reached |= std::abs(e_theta) <= kAngleTolerance;

    // position loop
    if (heading_reached) {
      double e_x{point.x - state_cur_->x};
      double e_y{point.y - state_cur_->y};
      e_s = std::sqrt(e_x * e_x + e_y * e_y);

      double de_x{/* dx_goal = 0 */ -state_cur_->dx};
      double de_y{/* dy_goal = 0 */ -state_cur_->dy};
      double de_s{(e_x * de_x + e_y * de_y) /
                  (e_s + 0.001)}; // derivative of norm + normalization

      v_d.linear.x = pidStepPosition(e_s, de_s, ie_s);
      ie_s += std::chrono::duration<double>{kControlCycle}.count() * e_x;
    }

    twist_pub_->publish(v_d);
    rclcpp::sleep_for(kControlCycle);
  }

  // goal reached
  twist_pub_->publish(Twist{});
  RCLCPP_INFO(this->get_logger(), "Reached waypoint.");
  return true;
}

bool PidMazeSolver::followTrajectory(GoalSequence goals) {
  RCLCPP_INFO(this->get_logger(), "Received new goal trajectory.");

  for (const auto &goal : goals) {
    if (!goToGoal(goal)) {
      RCLCPP_WARN(this->get_logger(), "Failed to follow goal trajectory.");
      return false;
    }
  }
  RCLCPP_INFO(this->get_logger(), "Reached goal.");
  return true;
}

std::string getGoalsYamlFilename() {
  const std::string package_share_directory{
      ament_index_cpp::get_package_share_directory("pid_maze_solver")};
  std::string waypoint_file_name{"sim_points_1.yaml"};
  const std::string yaml_file{package_share_directory + "/waypoints/" +
                              waypoint_file_name};
  return yaml_file;
}

GoalSequence loadSetpointsFromYaml(const std::string &yaml_file) {
  GoalSequence goals{};
  YAML::Node goals_yaml;

  try {
    goals_yaml = YAML::LoadFile(yaml_file);
    goals = goals_yaml["goals"].as<GoalSequence>();
  } catch (const YAML::Exception &e) {
    throw std::runtime_error{std::string{"Failed to read YAML file: "} +
                             e.what()};
  }

  return goals;
}

} // namespace PidMazeSolver

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node{std::make_shared<PidMazeSolver::PidMazeSolver>()};
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto executor_thread{std::thread([&executor]() { executor.spin(); })};

  const auto goals{PidMazeSolver::loadSetpointsFromYaml(
      PidMazeSolver::getGoalsYamlFilename())};
  node->followTrajectory(goals);

  if (executor_thread.joinable()) {
    executor_thread.join();
  }
  rclcpp::shutdown();
}