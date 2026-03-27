#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "bspline_opt/uniform_bspline.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "traj_utils/msg/bspline.hpp"

using ego_planner::UniformBspline;

namespace
{

double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double yawFromQuaternion(double x, double y, double z, double w)
{
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

class Go2TrajServer : public rclcpp::Node
{
public:
  Go2TrajServer() : Node("go2_traj_server")
  {
    const auto bspline_topic =
        this->declare_parameter<std::string>("bspline_topic", "planning/bspline");
    const auto odom_topic =
        this->declare_parameter<std::string>("odom_topic", "odom_world");
    const auto cmd_vel_topic =
        this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const auto control_rate_hz =
        this->declare_parameter<double>("control_rate_hz", 20.0);

    time_forward_ = this->declare_parameter<double>("time_forward", 0.35);
    min_linear_speed_ = this->declare_parameter<double>("min_linear_speed", 0.0);
    max_linear_x_ = this->declare_parameter<double>("max_linear_x", 0.5);
    max_linear_y_ = this->declare_parameter<double>("max_linear_y", 0.0);
    min_angular_speed_ = this->declare_parameter<double>("min_angular_speed", 0.0);
    max_angular_z_ = this->declare_parameter<double>("max_angular_z", 1.0);
    heading_kp_ = this->declare_parameter<double>("heading_kp", 1.5);
    goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.18);
    stop_yaw_tolerance_ = this->declare_parameter<double>("stop_yaw_tolerance", 0.2);

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 20,
        std::bind(&Go2TrajServer::odomCallback, this, std::placeholders::_1));
    bspline_sub_ = this->create_subscription<traj_utils::msg::Bspline>(
        bspline_topic, 10,
        std::bind(&Go2TrajServer::bsplineCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(control_rate_hz, 1.0)),
        std::bind(&Go2TrajServer::publishCommand, this));
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_odom_ = msg;
  }

  void bsplineCallback(const traj_utils::msg::Bspline::SharedPtr msg)
  {
    if (msg->pos_pts.empty() || msg->knots.size() < 2)
    {
      active_traj_ = false;
      RCLCPP_WARN(this->get_logger(), "Received empty B-spline, trajectory deactivated.");
      return;
    }

    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
    {
      pos_pts(0, i) = msg->pos_pts[i].x;
      pos_pts(1, i) = msg->pos_pts[i].y;
      pos_pts(2, i) = msg->pos_pts[i].z;
    }

    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i)
    {
      knots(i) = msg->knots[i];
    }

    position_traj_ = UniformBspline(pos_pts, msg->order, msg->knots[1] - msg->knots[0]);
    position_traj_.setKnot(knots);
    velocity_traj_ = position_traj_.getDerivative();
    traj_duration_ = position_traj_.getTimeSum();
    traj_start_time_ = rclcpp::Time(msg->start_time);
    active_traj_ = true;
    RCLCPP_INFO(
        this->get_logger(),
        "Accepted B-spline traj_id=%ld, ctrl_pts=%zu, duration=%.3f s",
        msg->traj_id,
        msg->pos_pts.size(),
        traj_duration_);
  }

  void publishZero()
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    last_logged_command_ = std::nullopt;
  }

  void publishCommand()
  {
    if (!active_traj_ || !latest_odom_)
    {
      publishZero();
      return;
    }

    const auto now = this->get_clock()->now();
    double t_cur = (now - traj_start_time_).seconds();
    t_cur = clamp(t_cur, 0.0, traj_duration_);

    const Eigen::Vector3d position = position_traj_.evaluateDeBoorT(t_cur);
    const Eigen::Vector3d velocity_world = velocity_traj_.evaluateDeBoorT(t_cur);

    const auto &odom = latest_odom_->pose.pose;
    const Eigen::Vector3d odom_pos(
        odom.position.x, odom.position.y, odom.position.z);
    const double yaw = yawFromQuaternion(
        odom.orientation.x,
        odom.orientation.y,
        odom.orientation.z,
        odom.orientation.w);

    Eigen::Vector3d future_position = position;
    if (traj_duration_ > 1e-3)
    {
      future_position =
          position_traj_.evaluateDeBoorT(std::min(t_cur + time_forward_, traj_duration_));
    }
    const Eigen::Vector3d heading_dir = future_position - odom_pos;
    const double desired_yaw =
        heading_dir.head<2>().norm() > 0.05 ? std::atan2(heading_dir.y(), heading_dir.x()) : yaw;
    const double heading_error = normalizeAngle(desired_yaw - yaw);

    geometry_msgs::msg::Twist cmd;
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    cmd.linear.x = cos_yaw * velocity_world.x() + sin_yaw * velocity_world.y();
    cmd.linear.y = -sin_yaw * velocity_world.x() + cos_yaw * velocity_world.y();
    cmd.angular.z = heading_kp_ * heading_error;

    const double distance_to_goal = (position - odom_pos).head<2>().norm();
    const bool trajectory_finished = t_cur >= traj_duration_ - 1e-2;
    if (trajectory_finished && distance_to_goal < goal_tolerance_ &&
        std::abs(heading_error) < stop_yaw_tolerance_)
    {
      publishZero();
      active_traj_ = false;
      RCLCPP_INFO(this->get_logger(), "Trajectory finished and goal tolerance satisfied, stopping.");
      return;
    }

    cmd.linear.x = clamp(cmd.linear.x, -max_linear_x_, max_linear_x_);
    cmd.linear.y = clamp(cmd.linear.y, -max_linear_y_, max_linear_y_);
    cmd.angular.z = clamp(cmd.angular.z, -max_angular_z_, max_angular_z_);

    const double planar_speed = std::hypot(cmd.linear.x, cmd.linear.y);
    if (planar_speed > 1e-6 && planar_speed < min_linear_speed_)
    {
      const double scale = min_linear_speed_ / planar_speed;
      cmd.linear.x = clamp(cmd.linear.x * scale, -max_linear_x_, max_linear_x_);
      cmd.linear.y = clamp(cmd.linear.y * scale, -max_linear_y_, max_linear_y_);
    }

    if (std::abs(cmd.angular.z) > 1e-6 && std::abs(cmd.angular.z) < min_angular_speed_)
    {
      cmd.angular.z = std::copysign(min_angular_speed_, cmd.angular.z);
      cmd.angular.z = clamp(cmd.angular.z, -max_angular_z_, max_angular_z_);
    }

    const std::array<double, 3> command_key{
        std::round(cmd.linear.x * 1000.0) / 1000.0,
        std::round(cmd.linear.y * 1000.0) / 1000.0,
        std::round(cmd.angular.z * 1000.0) / 1000.0};
    if (!last_logged_command_.has_value() || last_logged_command_.value() != command_key)
    {
      RCLCPP_INFO(
          this->get_logger(),
          "Publishing cmd_vel: vx=%.3f vy=%.3f wz=%.3f, goal_dist=%.3f, heading_err=%.3f",
          cmd.linear.x,
          cmd.linear.y,
          cmd.angular.z,
          distance_to_goal,
          heading_error);
      last_logged_command_ = command_key;
    }

    cmd_pub_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<traj_utils::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  UniformBspline position_traj_;
  UniformBspline velocity_traj_;
  rclcpp::Time traj_start_time_{0, 0, RCL_SYSTEM_TIME};
  double traj_duration_{0.0};
  bool active_traj_{false};

  double time_forward_{0.35};
  double min_linear_speed_{0.0};
  double max_linear_x_{0.5};
  double max_linear_y_{0.0};
  double min_angular_speed_{0.0};
  double max_angular_z_{1.0};
  double heading_kp_{1.5};
  double goal_tolerance_{0.18};
  double stop_yaw_tolerance_{0.2};
  std::optional<std::array<double, 3>> last_logged_command_;
};

}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2TrajServer>());
  rclcpp::shutdown();
  return 0;
}
