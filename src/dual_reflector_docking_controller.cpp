#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <docking_marker/DockTarget.h>
#include <docking_marker/DockingStatus.h>
#include <cmath>
#include <deque>

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
    ros::NodeHandle pnh("~");

    /* ===== Load parameters ===== */

    // Predock
    pnh.param("predock/pos_x", predock_pos_x_, 1.9f);
    pnh.param("predock/pos_y", predock_pos_y_, -0.45f);
    pnh.param("predock/yaw",   predock_pos_angle_, 0.0f);

    pnh.param("predock/pos_tol",
              predock_reached_pos_tol_, 0.1f);
    pnh.param("predock/yaw_tol",
              predock_reached_orientation_tol_, float(M_PI/6));
    pnh.param("predock/dwell_time",
              predock_dwell_time_, 2.0);

    // Dock
    pnh.param("dock/offset_x", dock_offset_x, 0.04f);
    pnh.param("dock/pos_tol",  dock_pos_tol_, 0.03f);
    pnh.param("dock/ang_tol",  dock_ang_tol_, 0.008f);

    // PID
    pnh.param("pid/angular/kp", ang_kp, 0.8);
    pnh.param("pid/angular/kd", ang_kd, 0.04);

    pnh.param("pid/linear/kp", lin_kp, 0.45);
    pnh.param("pid/linear/kd", lin_kd, 0.04);

    // Angle filter
    pnh.param("angle_filter/window_size",
              angle_window_size_, 10);

    /* ===== ROS interfaces ===== */

    target_sub_ = nh_.subscribe(
      "/dock_target", 1,
      &DockingController::targetCallback, this);

    amcl_sub_ = nh_.subscribe(
      "/amcl_pose", 1,
      &DockingController::amclCallback, this);

    cmd_pub_ =
      nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    docking_status_pub_ =
      nh_.advertise<docking_marker::DockingStatus>(
        "/docking_status", 1, true);  // latched

    publishDockingStatus(0, 0.0, 0.0); // IDLE

    last_time_ = ros::Time::now();
  }

  /* ================= AMCL CALLBACK ================= */

  void amclCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double yaw = yawFromQuat(msg->pose.pose.orientation);

    double dx = x - predock_pos_x_;
    double dy = y - predock_pos_y_;
    double yaw_err =
      normalizeAngle(yaw - predock_pos_angle_);

    predock_reached_ =
      (fabs(dx) < predock_reached_pos_tol_ &&
       fabs(dy) < predock_reached_pos_tol_ &&
       fabs(yaw_err) < predock_reached_orientation_tol_);
  }

  /* ================= TARGET CALLBACK ================= */

  void targetCallback(
    const docking_marker::DockTarget::ConstPtr& msg)
  {
    /* -------- re-arm after leaving predock -------- */
    if (!predock_reached_ && dock_completed_)
    {
      dock_completed_ = false;
      publishDockingStatus(0, 0.0, 0.0); // IDLE
    }

    /* -------- predock dwell-time gating -------- */
    if (predock_reached_ &&
        !dock_start_ &&
        !dock_completed_)
    {
      publishDockingStatus(1, msg->distance, msg->angle); // PREDOCK

      if (!predock_timer_running_)
      {
        predock_enter_time_ = ros::Time::now();
        predock_timer_running_ = true;
      }
      else
      {
        double dwell =
          (ros::Time::now() -
           predock_enter_time_).toSec();

        if (dwell >= predock_dwell_time_)
        {
          ROS_WARN("Predock stable for %.1fs - Docking start",
                   predock_dwell_time_);
          dock_start_ = true;
          dock_completed_ = false;
          predock_timer_running_ = false;

          publishDockingStatus(2, msg->distance, msg->angle); // DOCKING
        }
      }
    }
    else
    {
      predock_timer_running_ = false;
    }

    if (!dock_start_ || !msg->valid)
      return;

    if (dock_start_ && !msg->valid) {
      marker_valid_false_count_ ++;
    }
    else if (dock_start_ && msg->valid) {
      marker_valid_false_count_ = 0;
    }

    if (marker_valid_false_count_ < 5) {
      dist_err = msg->distance - dock_offset_x;

      double raw_ang = normalizeAngle(msg->angle);
      updateAngleBuffer(raw_ang);
      ang_err = getAverageAngle();
    }else {
      dist_err = 0.0;
      ang_err = 0.0;
    }
    
    //ROS_INFO("distance error - %f, ang error - %f", dist_err, ang_err);

    if (fabs(dist_err) > 0.5 ||
        fabs(ang_err)  > 0.5)
      return;

    ros::Time now = ros::Time::now();
    double dt = (now - last_time_).toSec();
    last_time_ = now;
    if (dt <= 0.0) return;

    geometry_msgs::Twist cmd;

    /* -------- angular PD -------- */
    double ang_d =
      (ang_err - prev_ang_err_) / dt;
    prev_ang_err_ = ang_err;

    double angular_cmd =
      clamp(ang_kp * ang_err +
            ang_kd * ang_d,
            -0.1, 0.1);

    /* -------- linear PD -------- */
    double linear_cmd = 0.0;

    if (fabs(ang_err) < 0.1)
    {
      double lin_d =
        (dist_err - prev_dist_err_) / dt;
      prev_dist_err_ = dist_err;

      linear_cmd =
        clamp(lin_kp * dist_err +
              lin_kd * lin_d,
              0.02, 0.12);
    }
    else
    {
      prev_dist_err_ = 0.0;
    }

    /* -------- stop / move -------- */
    if (fabs(dist_err) < dock_pos_tol_ &&
        fabs(ang_err)  < dock_ang_tol_)
    {
      cmd.linear.x  = 0.0;
      cmd.angular.z = 0.0;

      ROS_INFO_THROTTLE(1.0, "Docking complete");

      dock_start_ = false;
      dock_completed_ = true;

      publishDockingStatus(3, msg->distance, ang_err); // DOCKED
    }
    else
    {
      cmd.linear.x  = linear_cmd;
      cmd.angular.z = angular_cmd;

      publishDockingStatus(2, msg->distance, ang_err); // DOCKING
    }

    cmd_pub_.publish(cmd);
  }

private:
  /* ================= Docking Status ================= */

  void publishDockingStatus(uint8_t state,
                            float distance,
                            float angle)
  {
    docking_marker::DockingStatus msg;
    msg.state    = state;
    msg.distance = distance;
    msg.angle    = angle;
    docking_status_pub_.publish(msg);
  }

  /* ================= Angle averaging ================= */

  void updateAngleBuffer(double angle)
  {
    angle_buffer_.push_back(angle);
    if (angle_buffer_.size() >
        static_cast<size_t>(angle_window_size_))
      angle_buffer_.pop_front();
  }

  double getAverageAngle()
  {
    if (angle_buffer_.empty())
      return 0.0;

    double s = 0.0, c = 0.0;
    for (double a : angle_buffer_)
    {
      s += sin(a);
      c += cos(a);
    }
    return atan2(s, c);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber target_sub_;
  ros::Subscriber amcl_sub_;
  ros::Publisher  cmd_pub_;
  ros::Publisher  docking_status_pub_;

  /* ===== Predock ===== */
  float predock_pos_x_;
  float predock_pos_y_;
  float predock_pos_angle_;
  float predock_reached_pos_tol_;
  float predock_reached_orientation_tol_;
  double predock_dwell_time_;
  bool  predock_reached_ = false;

  bool predock_timer_running_ = false;
  ros::Time predock_enter_time_;

  /* ===== Dock state ===== */
  bool dock_start_     = false;
  bool dock_completed_ = false;

  /* ===== Dock params ===== */
  float dock_offset_x;
  float dock_pos_tol_;
  float dock_ang_tol_;

  /* ===== PID ===== */
  double ang_kp, ang_kd;
  double lin_kp, lin_kd;

  double prev_ang_err_  = 0.0;
  double prev_dist_err_ = 0.0;

  int marker_valid_false_count_ = 0;
  double dist_err = 0.0;
  double ang_err = 0.0;
  ros::Time last_time_;

  /* ===== Angle filter ===== */
  int angle_window_size_;
  std::deque<double> angle_buffer_;
};

/* ================= MAIN ================= */

int main(int argc, char** argv)
{
  ros::init(argc, argv,
            "dual_reflector_docking_controller");
  DockingController node;
  ros::spin();
  return 0;
}
