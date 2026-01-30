#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <docking_marker/DockTarget.h>
#include <cmath>
#include <vector>

constexpr float MAX_CLUSTER_RANGE_GAP = 0.04;      // meters
constexpr int   MIN_CLUSTER_PTS = 4;
constexpr int   MAX_CLUSTER_PTS = 10;

constexpr float REFLECTOR_SEPARATION = 0.30;       // meters
constexpr float SEPARATION_TOL = 0.06;             // ±6 cm

constexpr float MAX_DETECTION_DISTANCE = 1.0;      // meters
constexpr float MAX_DETECTION_ANGLE = M_PI / 6.0;  // ±30 deg

struct Cluster
{
  float x;
  float y;
};

float normalizeAngle(float a)
{
  while (a > M_PI)  a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

class DualReflectorDetector
{
public:
  DualReflectorDetector()
  {
    scan_sub_ = nh_.subscribe("/scan", 1,
      &DualReflectorDetector::scanCB, this);

    target_pub_ = nh_.advertise<docking_marker::DockTarget>(
      "/dock_target", 1);
  }

  void scanCB(const sensor_msgs::LaserScan::ConstPtr& scan)
  {
    std::vector<Cluster> clusters;
    int start = -1;

    // --- STEP 1: Range-based clustering
    for (size_t i = 0; i < scan->ranges.size(); i++)
    {
      float r = scan->ranges[i];

      if (!std::isfinite(r))
      {
        processCluster(scan, start, i - 1, clusters);
        start = -1;
        continue;
      }

      if (start < 0)
        start = i;

      if (i > start &&
          fabs(scan->ranges[i] - scan->ranges[i - 1]) > MAX_CLUSTER_RANGE_GAP)
      {
        processCluster(scan, start, i - 1, clusters);
        start = i;
      }
    }

    processCluster(scan, start,
                   scan->ranges.size() - 1,
                   clusters);

    publishMidpoint(clusters);
  }

private:
  void processCluster(const sensor_msgs::LaserScan::ConstPtr& scan,
                      int s, int e,
                      std::vector<Cluster>& clusters)
  {
    if (s < 0 || e <= s) return;

    int count = e - s + 1;
    if (count < MIN_CLUSTER_PTS || count > MAX_CLUSTER_PTS)
      return;

    float sum_r = 0.0;
    for (int i = s; i <= e; i++)
      sum_r += scan->ranges[i];
    float mean_r = sum_r / count;

    // Reject non-compact clusters (walls, edges)
    for (int i = s; i <= e; i++)
    {
      if (fabs(scan->ranges[i] - mean_r) > 0.03)
        return;
    }

    float center_idx = 0.5f * (s + e);
    float angle = scan->angle_min +
                  center_idx * scan->angle_increment;
    angle = normalizeAngle(angle);

    float x = mean_r * cos(angle);
    float y = mean_r * sin(angle);

    clusters.push_back({x, y});
  }

  void publishMidpoint(const std::vector<Cluster>& clusters)
  {
    docking_marker::DockTarget msg;
    msg.valid = false;

    if (clusters.size() < 2)
    {
      target_pub_.publish(msg);
      return;
    }

    // --- STEP 2: Pair clusters geometrically
    for (size_t i = 0; i < clusters.size(); i++)
    {
      for (size_t j = i + 1; j < clusters.size(); j++)
      {
        float dx = clusters[i].x - clusters[j].x;
        float dy = clusters[i].y - clusters[j].y;
        float sep = std::sqrt(dx * dx + dy * dy);

        if (fabs(sep - REFLECTOR_SEPARATION) > SEPARATION_TOL)
          continue;

        // --- STEP 3: Midpoint
        float mx = 0.5f * (clusters[i].x + clusters[j].x);
        float my = 0.5f * (clusters[i].y + clusters[j].y);

        float dist = std::sqrt(mx * mx + my * my);
        float angle = normalizeAngle(std::atan2(my, mx));

        // --- STEP 4: Gating
        if (dist > MAX_DETECTION_DISTANCE) continue;
        if (fabs(angle) > MAX_DETECTION_ANGLE) continue;

        msg.valid = true;
        msg.distance = dist;
        msg.angle = angle;

        target_pub_.publish(msg);
        return;
      }
    }

    target_pub_.publish(msg);
  }

  ros::NodeHandle nh_;
  ros::Subscriber scan_sub_;
  ros::Publisher target_pub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dual_reflector_detector");
  DualReflectorDetector node;
  ros::spin();
  return 0;
}
