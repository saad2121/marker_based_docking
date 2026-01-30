#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <docking_marker/DockTarget.h>
#include <cmath>

double normalizeAngle(double a)
{
  while (a > M_PI)  a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

class DockingController
{
public:
  DockingController()
  {
    target_sub_ = nh_.subscribe("/dock_target", 1, &DockingController::targetCallback, this);

    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  }

  bool docking_started_ = false;

  void targetCallback(const docking_marker::DockTarget::ConstPtr& msg)
  {
    geometry_msgs::Twist cmd;
    bool motion_this_cycle = false;

    if (msg->valid)
    {
      float angle_err = normalizeAngle(msg->angle);
      float dist_err  = msg->distance - final_distance_;

      if (fabs(angle_err) > 0.05)
      {
        cmd.angular.z = 0.6 * angle_err;
        motion_this_cycle = true;
      }
      else if (dist_err > 0.02)
      {
        cmd.linear.x = (dist_err > 0.3) ? 0.15 : 0.05;
        motion_this_cycle = true;
      }
      else
      {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        ROS_INFO("Docked");
      }
    }
    else
    {
      if (!docking_started_)
      {
        ROS_WARN_THROTTLE(1.0, "Marker not detected");
        // cmd.angular.z = 0.2; // search
      }
      else
      {
        ROS_WARN_THROTTLE(1.0, "Marker lost during docking -> STOP");
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
      }
    }

    if (motion_this_cycle)
      docking_started_ = true;

    cmd_pub_.publish(cmd);

  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber target_sub_;
  ros::Publisher cmd_pub_;
  float final_distance_ = 0.20;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "docking_controller");
  DockingController node;
  ros::spin();
  return 0;
}

