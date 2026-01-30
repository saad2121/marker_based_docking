#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <docking_marker/DockTarget.h>
#include <cmath>

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
    const float max_gap = 0.06;   // meters
    const int min_pts = 3;
    const int max_pts = 8;

    int start = -1;
    bool found = false;
    float best_dist = 100.0;
    float best_angle = 0.0;

    for (size_t i = 0; i < scan->ranges.size(); i++)
    {
      float r = scan->ranges[i];

      if (!std::isfinite(r))
      {
        processCluster(scan, start, i - 1, best_dist, best_angle, found);
        start = -1;
        continue;
      }

      if (start < 0) start = i;

      if (i > start &&
          fabs(scan->ranges[i] - scan->ranges[i - 1]) > max_gap)
      {
        processCluster(scan, start, i - 1, best_dist, best_angle, found);
        start = i;
      }
    }

    processCluster(scan, start,scan->ranges.size() - 1,best_dist, best_angle, found);

    docking_marker::DockTarget msg;
    msg.valid = found;
    msg.distance = best_dist;
    msg.angle = best_angle;
    target_pub_.publish(msg);
  }

private:
  void processCluster(const sensor_msgs::LaserScan::ConstPtr& scan,
                      int s, int e,
                      float& best_dist,
                      float& best_angle,
                      bool& found)
  {
    if (s < 0 || e <= s) return;

    int count = e - s + 1;
    if (count < 3 || count > 8) return;

    float sum = 0.0;
    for (int i = s; i <= e; i++)
      sum += scan->ranges[i];

    float mean = sum / count;
    if (mean < best_dist)
    {
      float center = (s + e) * 0.5;
      best_angle = scan->angle_min +
                   center * scan->angle_increment;
      best_dist = mean;
      found = true;
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

