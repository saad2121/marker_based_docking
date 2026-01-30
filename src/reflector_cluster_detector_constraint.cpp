#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <docking_marker/DockTarget.h>
#include <cmath>

constexpr float MAX_CLUSTER_GAP = 0.06;        // meters
constexpr int   MIN_CLUSTER_PTS = 3;
constexpr int   MAX_CLUSTER_PTS = 10;

constexpr float MAX_DETECTION_DISTANCE = 0.9;  // meters
constexpr float MAX_DETECTION_ANGLE = M_PI / 6.0; // ±30 deg

float normalizeAngle(float a)
{
  while (a > M_PI)  a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

class ReflectorClusterDetector
{
public:
  ReflectorClusterDetector()
  {
    scan_sub_ = nh_.subscribe("/scan", 1, &ReflectorClusterDetector::scanCallback, this);

    target_pub_ = nh_.advertise<docking_marker::DockTarget>("/dock_target", 1);
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
  {
    int start = -1;
    bool found = false;
    float best_dist = 100.0;
    float best_angle = 0.0;

    for (size_t i = 0; i < scan->ranges.size(); i++)
    {
      float r = scan->ranges[i];

      if (!std::isfinite(r))
      {
        evaluateCluster(scan, start, i - 1,
                        found, best_dist, best_angle);
        start = -1;
        continue;
      }

      if (start < 0) start = i;

      if (i > start &&
          fabs(scan->ranges[i] - scan->ranges[i - 1]) > MAX_CLUSTER_GAP)
      {
        evaluateCluster(scan, start, i - 1,
                        found, best_dist, best_angle);
        start = i;
      }
    }

    evaluateCluster(scan, start,
                    scan->ranges.size() - 1,
                    found, best_dist, best_angle);

    docking_marker::DockTarget msg;
    msg.valid = found;
    msg.distance = best_dist;
    msg.angle = best_angle;
    target_pub_.publish(msg);
  }

private:
  void evaluateCluster(const sensor_msgs::LaserScan::ConstPtr& scan,
                       int s, int e,
                       bool& found,
                       float& best_dist,
                       float& best_angle)
  {
    if (s < 0 || e <= s) return;

    int count = e - s + 1;
    if (count < MIN_CLUSTER_PTS || count > MAX_CLUSTER_PTS)
      return;

    // Mean distance
    float sum = 0.0;
    for (int i = s; i <= e; i++)
      sum += scan->ranges[i];
    float mean_dist = sum / count;

    // Distance gate
    if (mean_dist > MAX_DETECTION_DISTANCE)
      return;

    // Angle calculation
    float center_idx = (s + e) * 0.5f;
    float angle = scan->angle_min +
                  center_idx * scan->angle_increment;
    angle = normalizeAngle(angle);

    // Heading gate
    if (fabs(angle) > MAX_DETECTION_ANGLE)
      return;

    // Choose closest valid reflector
    if (!found || mean_dist < best_dist)
    {
      found = true;
      best_dist = mean_dist;
      best_angle = angle;
    }
  }

  ros::NodeHandle nh_;
  ros::Subscriber scan_sub_;
  ros::Publisher target_pub_;
};
  
int main(int argc, char** argv)
{
  ros::init(argc, argv, "reflector_cluster_detector");
  ReflectorClusterDetector node;
  ros::spin();
  return 0;
}
