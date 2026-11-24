#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace
{
constexpr double kQuarterTurn = 1.5707963267948966;
constexpr double kLaneWidth = 0.36;
constexpr double kScanStop = 0.36;
constexpr double kSonarStop = 0.18;
constexpr double kFreshness = 0.5;

double angleError(double target, double current)
{
  return std::atan2(std::sin(target - current), std::cos(target - current));
}

bool fresh(const ros::Time& stamp, const ros::Time& now)
{
  return !stamp.isZero() && stamp <= now && (now - stamp).toSec() <= kFreshness;
}

class VacuumSweep
{
public:
  explicit VacuumSweep(ros::NodeHandle& node)
  {
    drive_ = node.advertise<geometry_msgs::Twist>("drive_controller/cmd_vel", 1);
    odometry_ = node.subscribe("drive_controller/odom", 1, &VacuumSweep::onOdometry, this);
    lidar_ = node.subscribe("scan", 1, &VacuumSweep::onScan, this);
    const std::array<const char*, 3> topics{{"sonar/left", "sonar/center", "sonar/right"}};
    for (std::size_t i = 0; i < topics.size(); ++i)
    {
      sonars_[i] = node.subscribe<sensor_msgs::Range>(
          topics[i], 1, [this, i](const sensor_msgs::Range::ConstPtr& msg) {
            range_stamps_[i] = msg->header.stamp;
            ranges_[i] = msg->range;
          });
    }
  }

  void update()
  {
    geometry_msgs::Twist command;
    const auto now = ros::Time::now();
    if (fresh(odom_stamp_, now) && fresh(scan_stamp_, now) && rangesReady(now))
      command = nextCommand();
    drive_.publish(command);
  }

private:
  enum class Phase { Straight, TurnAcross, Shift, TurnBack, EscapeTurn };

  void onOdometry(const nav_msgs::Odometry::ConstPtr& msg)
  {
    const auto& pose = msg->pose.pose;
    const auto& rotation = pose.orientation;
    const tf2::Quaternion quaternion(rotation.x, rotation.y, rotation.z, rotation.w);
    if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
        !std::isfinite(quaternion.length2()) || quaternion.length2() < 1e-6)
    {
      odom_stamp_ = ros::Time();
      return;
    }
    double roll, pitch;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw_);
    x_ = pose.position.x;
    y_ = pose.position.y;
    odom_stamp_ = msg->header.stamp;
  }

  void onScan(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    scan_stamp_ = ros::Time();
    scan_blocked_ = false;
    for (std::size_t i = 0; i < msg->ranges.size(); ++i)
    {
      const double angle = msg->angle_min + i * msg->angle_increment;
      if (std::abs(angle) > kQuarterTurn)
        continue;

      const double range = msg->ranges[i];
      if (std::isnan(range) || range <= 0.0)
        continue;

      scan_stamp_ = msg->header.stamp;
      if (std::isfinite(range) && range <= msg->range_max &&
          range * std::cos(angle) < kScanStop && std::abs(range * std::sin(angle)) < 0.21)
        scan_blocked_ = true;
    }
  }

  bool rangesReady(const ros::Time& now) const
  {
    for (std::size_t i = 0; i < ranges_.size(); ++i)
      if (!fresh(range_stamps_[i], now) || std::isnan(ranges_[i]))
        return false;
    return true;
  }

  bool blocked() const
  {
    return scan_blocked_ || *std::min_element(ranges_.begin(), ranges_.end()) <= kSonarStop;
  }

  geometry_msgs::Twist nextCommand()
  {
    geometry_msgs::Twist command;
    switch (phase_)
    {
      case Phase::Straight:
        if (blocked())
        {
          target_yaw_ = yaw_ + turn_sign_ * kQuarterTurn;
          phase_ = Phase::TurnAcross;
          ROS_INFO("Obstacle: first 90-degree turn %s", turn_sign_ < 0 ? "right" : "left");
        }
        else
          command.linear.x = 0.20;
        break;

      case Phase::TurnAcross:
      case Phase::TurnBack:
      case Phase::EscapeTurn:
      {
        const double error = angleError(target_yaw_, yaw_);
        if (std::abs(error) > 0.018)
        {
          command.angular.z = std::max(-0.65, std::min(0.65, 2.5 * error));
          break;
        }

        if (phase_ == Phase::TurnAcross)
        {
          shift_x_ = x_;
          shift_y_ = y_;
          phase_ = Phase::Shift;
        }
        else
        {
          if (phase_ == Phase::TurnBack)
            turn_sign_ = -turn_sign_;
          phase_ = Phase::Straight;
        }
        break;
      }

      case Phase::Shift:
      {
        const double progress = (x_ - shift_x_) * std::cos(target_yaw_) +
                                (y_ - shift_y_) * std::sin(target_yaw_);
        if (progress >= kLaneWidth - 0.005)
        {
          target_yaw_ += turn_sign_ * kQuarterTurn;
          phase_ = Phase::TurnBack;
        }
        else if (blocked())
        {
          target_yaw_ = yaw_ + turn_sign_ * kQuarterTurn * (4.0 / 3.0);
          phase_ = Phase::EscapeTurn;
          ROS_WARN("Lane change obstructed: turning back toward the open room");
        }
        else
        {
          command.linear.x = std::min(0.13, std::max(0.03, kLaneWidth - progress));
          command.angular.z = 1.5 * angleError(target_yaw_, yaw_);
        }
        break;
      }
    }
    return command;
  }

  ros::Publisher drive_;
  ros::Subscriber odometry_;
  ros::Subscriber lidar_;
  std::array<ros::Subscriber, 3> sonars_;
  std::array<ros::Time, 3> range_stamps_;
  std::array<double, 3> ranges_{};
  ros::Time odom_stamp_, scan_stamp_;
  double x_ = 0.0, y_ = 0.0, yaw_ = 0.0;
  double shift_x_ = 0.0, shift_y_ = 0.0, target_yaw_ = 0.0;
  double turn_sign_ = -1.0;
  bool scan_blocked_ = false;
  Phase phase_ = Phase::Straight;
};
}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vacuum_sweep");
  ros::NodeHandle node;
  VacuumSweep sweep(node);
  ros::WallRate cycle(30);
  while (ros::ok())
  {
    ros::spinOnce();
    sweep.update();
    cycle.sleep();
  }
  return 0;
}
