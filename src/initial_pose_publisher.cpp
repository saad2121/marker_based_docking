#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "initial_pose_publisher");
  ros::NodeHandle nh("~");

  ros::Publisher pub =
    nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      "/initialpose", 1, true);   // latched publisher

  geometry_msgs::PoseWithCovarianceStamped msg;
  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time::now();

  // Load parameters
  nh.param("initial_pose/frame_id", msg.header.frame_id, std::string("map"));

  nh.param("initial_pose/position/x", msg.pose.pose.position.x, -2.0);
  nh.param("initial_pose/position/y", msg.pose.pose.position.y, -0.5);
  nh.param("initial_pose/position/z", msg.pose.pose.position.z, 0.0);

  nh.param("initial_pose/orientation/qx", msg.pose.pose.orientation.x, 0.0);
  nh.param("initial_pose/orientation/qy", msg.pose.pose.orientation.y, 0.0);
  nh.param("initial_pose/orientation/qz", msg.pose.pose.orientation.z, 0.0);
  nh.param("initial_pose/orientation/qw", msg.pose.pose.orientation.w, 1.0);

  double cov_xx, cov_yy, cov_yaw;
  nh.param("initial_pose/covariance/cov_xx", cov_xx, 0.25);
  nh.param("initial_pose/covariance/cov_yy", cov_yy, 0.25);
  nh.param("initial_pose/covariance/cov_yaw", cov_yaw, 0.07);

  // Fill covariance matrix
  msg.pose.covariance.assign(0.0);
  msg.pose.covariance[0]  = cov_xx;   // x
  msg.pose.covariance[7]  = cov_yy;   // y
  msg.pose.covariance[35] = cov_yaw;  // yaw

  // Give AMCL time to start
  ros::Duration(1.0).sleep();
  while (pub.getNumSubscribers() == 0 && ros::ok())
{
  ros::Duration(0.1).sleep();
}

  pub.publish(msg);
  ros::Duration(0.5).sleep();
pub.publish(msg); 
  ROS_INFO("Initial pose published to /initialpose");

  ros::spinOnce();
  return 0;
}
