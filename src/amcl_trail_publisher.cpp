#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <deque>

class AmclTrailPublisher
{
public:
  AmclTrailPublisher()
  {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param("max_points", max_points_, 20);
    pnh.param<std::string>("global_frame", global_frame_, "map");

    amcl_sub_ = nh.subscribe(
        "/amcl_pose", 10,
        &AmclTrailPublisher::amclCallback, this);

    path_pub_ = nh.advertise<nav_msgs::Path>(
        "amcl_trail", 1, true);  // latched publisher

    path_msg_.header.frame_id = global_frame_;

    ROS_INFO("AMCL Trail Publisher started (max points = %d)", max_points_);
  }

private:
  void amclCallback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    geometry_msgs::PoseStamped pose;
    pose.header = msg->header;
    pose.pose   = msg->pose.pose;

    trail_.push_back(pose);

    if (trail_.size() > max_points_)
      trail_.pop_front();

    path_msg_.header.stamp = ros::Time::now();
    path_msg_.poses.assign(trail_.begin(), trail_.end());

    path_pub_.publish(path_msg_);
  }

  ros::Subscriber amcl_sub_;
  ros::Publisher  path_pub_;

  std::deque<geometry_msgs::PoseStamped> trail_;
  nav_msgs::Path path_msg_;

  int max_points_;
  std::string global_frame_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "amcl_trail_publisher");
  AmclTrailPublisher node;
  ros::spin();
  return 0;
}
