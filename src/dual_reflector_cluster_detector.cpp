#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <docking_marker/DockTarget.h>
#include <cmath>
#include <vector>

/* ================= PARAMETERS ================= */

constexpr float MAX_CLUSTER_GAP = 0.06;   // meters
constexpr int   MIN_CLUSTER_PTS = 4;
constexpr int   MAX_CLUSTER_PTS = 20;

constexpr float MAX_RANGE = 0.6;           // meters
constexpr float REFLECTOR_SEPARATION = 0.238; // meters
constexpr float SEPARATION_TOL = 0.05;     // meters
constexpr float MAX_TARGET_ANGLE = M_PI / 6.0; // ±30 deg

constexpr double VALID_HOLD_TIME = 0.15;   // seconds

/* ================= UTIL ================= */

float normalizeAngle(float a)
{
  while (a > M_PI)  a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

struct Cluster
{
  float r;
  float angle;
};

/* ================= NODE ================= */

class DualReflectorDetector
{
public:
  DualReflectorDetector()
  {
    scan_sub_ = nh_.subscribe(
      "/scan", 1,
      &DualReflectorDetector::scanCallback, this);

    target_pub_ = nh_.advertise<docking_marker::DockTarget>(
      "/dock_target", 1);

    last_valid_time_ = ros::Time(0);
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
  {
    std::vector<Cluster> clusters;
    int start = -1;

    for (size_t i = 0; i < scan->ranges.size(); i++)
    {
      float r = scan->ranges[i];

      bool valid =
        std::isfinite(r) && r < MAX_RANGE;

      if (!valid)
      {
        finalizeCluster(scan, start, i - 1, clusters);
        start = -1;
        continue;
      }

      if (start < 0)
      {
        start = i;
        continue;
      }

      float prev_r = scan->ranges[i - 1];
      if (!std::isfinite(prev_r) || prev_r > MAX_RANGE)
      {
        finalizeCluster(scan, start, i - 1, clusters);
        start = i;
        continue;
      }

      float gap = fabs(r - prev_r);
      if (gap > MAX_CLUSTER_GAP)
      {
        finalizeCluster(scan, start, i - 1, clusters);
        start = i;
      }
    }

    finalizeCluster(scan,
                    start,
                    scan->ranges.size() - 1,
                    clusters);

    publishTarget(clusters);
  }

private:
  /* -------- cluster finalization -------- */

  void finalizeCluster(
    const sensor_msgs::LaserScan::ConstPtr& scan,
    int s, int e,
    std::vector<Cluster>& clusters)
  {
    if (s < 0 || e <= s)
      return;

    int count = e - s + 1;
    if (count < MIN_CLUSTER_PTS ||
        count > MAX_CLUSTER_PTS)
      return;

    float sum = 0.0f;
    for (int i = s; i <= e; i++)
      sum += scan->ranges[i];

    float mean_r = sum / count;
    float idx = 0.5f * (s + e);

    float angle =
      scan->angle_min +
      idx * scan->angle_increment;

    clusters.push_back(
      {mean_r, normalizeAngle(angle)});
  }

  void publishTarget(const std::vector<Cluster>& clusters)
{
  docking_marker::DockTarget msg;
  ros::Time now = ros::Time::now();

  bool left_found = false;
  bool right_found = false;
  Cluster left, right;

  /* -------- select closest cluster on each side -------- */
  for (const auto& c : clusters)
  {
    if (c.angle > 0)
    {
      if (!left_found || c.r < left.r)
      {
        left = c;
        left_found = true;
      }
    }
    else
    {
      if (!right_found || c.r < right.r)
      {
        right = c;
        right_found = true;
      }
    }
  }

  if (left_found && right_found)
  {
    float x1 = left.r  * cos(left.angle);
    float y1 = left.r  * sin(left.angle);
    float x2 = right.r * cos(right.angle);
    float y2 = right.r * sin(right.angle);

    float sep = hypot(x1 - x2, y1 - y2);
    if (fabs(sep - REFLECTOR_SEPARATION) < SEPARATION_TOL)
    {
      float mx = 0.5f * (x1 + x2);
      float my = 0.5f * (y1 + y2);

      float dist = hypot(mx, my);
      float ang  = normalizeAngle(atan2(my, mx));

      if (fabs(ang) < MAX_TARGET_ANGLE)
      {
        msg.valid = true;
        msg.distance = dist;
        msg.angle = ang;

        last_valid_msg_ = msg;
        last_valid_time_ = now;

       // ROS_INFO("valid - %d, distance - %f, angle - %f",msg.valid, msg.distance, msg.angle);

        target_pub_.publish(msg);
        return;
      }
    }
  }

  /* -------- temporal hold -------- */
  if ((now - last_valid_time_).toSec() < VALID_HOLD_TIME)
  {
    target_pub_.publish(last_valid_msg_);
  }
  else
  {
    msg.valid = false;
    msg.distance = 0.0;
    msg.angle = 0.0;
    target_pub_.publish(msg);
  }
}

private:
  ros::NodeHandle nh_;
  ros::Subscriber scan_sub_;
  ros::Publisher  target_pub_;

  docking_marker::DockTarget last_valid_msg_;
  ros::Time last_valid_time_;
};

/* ================= MAIN ================= */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dual_reflector_detector");
  DualReflectorDetector node;
  ros::spin();
  return 0;
}
