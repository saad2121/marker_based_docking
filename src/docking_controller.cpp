#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <docking_marker/DockTarget.h>
#include <cmath>

/* ================= Utility ================= */

double normalizeAngle(double a)
{
  while (a > M_PI)  a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(v, hi));
}

double yawFromQuat(const geometry_msgs::Quaternion& q)
{
  return atan2(2.0 * (q.w * q.z),
               1.0 - 2.0 * (q.z * q.z));
}

/* ================= Docking Controller ================= */

class DockingController
{
public:
  DockingController()
  {
    target_sub_ = nh_.subscribe("/dock_target", 1, &DockingController::targetCallback, this);

    amcl_sub_ = nh_.subscribe("/amcl_pose", 1, &DockingController::amclCallback, this);

    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    last_time_ = ros::Time::now();
  }

  /* ================= AMCL CALLBACK ================= */

  void amclCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double yaw = yawFromQuat(msg->pose.pose.orientation);

    double dx = x - predock_x_;
    double dy = y - predock_y_;
    double dist = std::hypot(dx, dy);

    double yaw_err = normalizeAngle(yaw - predock_yaw_);

    if (dist < predock_pos_tol_ &&
        fabs(yaw_err) < predock_yaw_tol_)
    {
      if (!predock_reached_)
        ROS_INFO("Pre-dock pose reached");

      predock_reached_ = true;
    }
    else
    {
      predock_reached_ = false;
    }
  }

  /* ================= DOCK TARGET CALLBACK ================= */

  void targetCallback(
    const docking_marker::DockTarget::ConstPtr& msg)
  {
    geometry_msgs::Twist cmd;

    /* -------- Pre-dock gating -------- */
    if (predock_reached_)
    {
      ROS_WARN_THROTTLE(2.0,"Predock reached - Docking start");
      dock_start_ = true;
      //ROS_INFO("dock start %d",dock_start_);
    }

    //ROS_INFO("valid=%d dist=%f", msg->valid, msg->distance);

    if (predock_reached_ && !msg->valid)
    {
      ROS_WARN_THROTTLE(1.0,
        "Marker not detected");
      return;
    }

    ros::Time now = ros::Time::now();
    double dt = (now - last_time_).toSec();
    last_time_ = now;

    if (dt <= 0.0)
      return;

    /* -------- Errors -------- */
    double angle_err = normalizeAngle(msg->angle);
    double dist_err  = msg->distance - final_distance_;

    if (fabs(angle_err) < angle_deadband_)
      angle_err = 0.0;

    /* ================= ANGULAR PD ================= */
    double ang_d = (angle_err - prev_angle_err_) / dt;
    prev_angle_err_ = angle_err;

    double angular_cmd =
      ang_kp_ * angle_err +
      ang_kd_ * ang_d;

    angular_cmd =
      clamp(angular_cmd,
            -max_ang_vel_,
             max_ang_vel_);

    /* ================= LINEAR PD ================= */
    double linear_cmd = 0.0;

    if (fabs(angle_err) < angle_control_thresh_ &&
        dist_err > dist_tolerance_)
    {
      double lin_d =
        (dist_err - prev_dist_err_) / dt;
      prev_dist_err_ = dist_err;

      linear_cmd =
        lin_kp_ * dist_err +
        lin_kd_ * lin_d;

      linear_cmd =
        clamp(linear_cmd,
              min_lin_vel_,
              max_lin_vel_);
    }
    else
    {
      prev_dist_err_ = 0.0;
    }

    /* ================= STOP ================= */
    if (fabs(dist_err) < dist_tolerance_ &&
        fabs(angle_err) < angle_deadband_)
    {
      cmd.linear.x  = 0.0;
      cmd.angular.z = 0.0;
      if (dock_start_) {
        ROS_INFO_THROTTLE(1.0, "Docking complete");
      }
      dock_start_ = false;
      //ROS_INFO("dock start %d",dock_start_);
    }
    else
    {
      if (dock_start_) {
      cmd.linear.x  = linear_cmd;
      cmd.angular.z = angular_cmd;
      }
    }

    //ROS_INFO("lin=%f ang=%f",cmd.linear.x,cmd.angular.z);
    
   
      cmd_pub_.publish(cmd);
  
    
  }

private:
  ros::NodeHandle nh_;

  ros::Subscriber target_sub_;
  ros::Subscriber amcl_sub_;
  ros::Publisher  cmd_pub_;

  /* ===== Pre-dock target ===== */
  double predock_x_   = 0.55;
  double predock_y_   = -0.87;
  double predock_yaw_ = 1.55;   // ≈ yaw from (0,0,0.693,0.72)

  double predock_pos_tol_ = 0.2;  // meters
  double predock_yaw_tol_ = 0.26;  // radians

  bool predock_reached_ = false;
  bool dock_start_ = false;

  /* ===== Docking params ===== */
  double final_distance_ = 0.20;

  double angle_deadband_        = 0.03;
  double angle_control_thresh_ = 0.04;
  double dist_tolerance_       = 0.03;

  double max_ang_vel_ = 0.6;
  double max_lin_vel_ = 0.15;
  double min_lin_vel_ = 0.02;

  /* ===== PID gains ===== */
  double ang_kp_ = 2.2;
  double ang_kd_ = 0.15;

  double lin_kp_ = 0.6;
  double lin_kd_ = 0.10;

  /* ===== State ===== */
  double prev_angle_err_ = 0.0;
  double prev_dist_err_  = 0.0;

  ros::Time last_time_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "docking_controller");
  DockingController node;
  ros::spin();
  return 0;
}
