#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib_msgs/GoalStatus.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <docking_marker/DockingStatus.h>

/* ===================== FSM States ===================== */
enum class State {
  START,
  GO_POINT_A,
  GO_POINT_B,
  GO_POINT_C,
  GO_PREDOCK,
  WAIT_DOCK,
  GO_UNDOCK
};

class MissionController {
public:
  MissionController()
    : nh_(), pnh_("~"), state_(State::START), mission_index_(0)
  {
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "/move_base_simple/goal", 1, true);

    nav_result_sub_ = nh_.subscribe(
        "/move_base/result", 1,
        &MissionController::navResultCB, this);

    dock_sub_ = nh_.subscribe(
        "/docking_status", 1,
        &MissionController::dockCB, this);

    loadPoses();

    start_timer_ = nh_.createTimer(
        ros::Duration(5),
        &MissionController::startMission,
        this,
        true);

    ROS_INFO("Mission Controller initialized");
  }

private:
  /* -------- Node Handles -------- */
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  /* -------- ROS Interfaces -------- */
  ros::Publisher  goal_pub_;
  ros::Subscriber nav_result_sub_;
  ros::Subscriber dock_sub_;
  ros::Timer      start_timer_;
  ros::Timer      delay_timer_;

  /* -------- FSM -------- */
  State state_;
  State next_state_;
  int mission_index_; 
  int docking_count_ = 0;  // 0=A, 1=B, 2=C

  /* -------- Mission Poses -------- */
  geometry_msgs::PoseStamped point_a_;
  geometry_msgs::PoseStamped point_b_;
  geometry_msgs::PoseStamped point_c_;
  geometry_msgs::PoseStamped predock_;
  geometry_msgs::PoseStamped undock_;
  geometry_msgs::PoseStamped delayed_goal_;

  /* =================== FSM LOGIC =================== */

  void startMission(const ros::TimerEvent&)
  {
    state_ = State::GO_POINT_A;
    sendGoal(point_a_);
    ROS_INFO("START -> Going to Point A");
  }

  void navResultCB(const move_base_msgs::MoveBaseActionResult& msg)
  {
    if (msg.status.status != actionlib_msgs::GoalStatus::SUCCEEDED)
      return;

    switch (state_) {

      case State::GO_POINT_A:
        next_state_   = State::GO_PREDOCK;
        delayed_goal_ = predock_;
        startDelayTimer(1.0);
        ROS_INFO("Point A reached ->  Going to predock");
        break;

      case State::GO_PREDOCK:
        state_ = State::WAIT_DOCK;
        ROS_INFO("Predock reached -> Waiting for docking");
        break;

      case State::GO_UNDOCK:
        mission_index_ = 1;  // always go to B after docking
        next_state_   = State::GO_POINT_B;
        delayed_goal_ = point_b_;
        startDelayTimer(1.0);
        ROS_INFO("Undocked  -> Going to Point B");
        break;

      case State::GO_POINT_B:
        mission_index_ = 2;
        next_state_   = State::GO_POINT_C;
        delayed_goal_ = point_c_;
        startDelayTimer(1.0);
        ROS_INFO("Point B reached -> Going to Point C");
        break;

      case State::GO_POINT_C:
        mission_index_ = 0;
        next_state_   = State::GO_POINT_A;
        delayed_goal_ = point_a_;
        startDelayTimer(1.0);
        ROS_INFO("Point C reached -> Going to Point A");
        break;

      default:
        break;
    }
  }

  void dockCB(const docking_marker::DockingStatus::ConstPtr& msg)
  {
    if (state_ == State::WAIT_DOCK && msg->state == 3) 
    { // DOCKED
      next_state_   = State::GO_UNDOCK;
      delayed_goal_ = undock_;
      startDelayTimer(1.0);
      ROS_INFO("Docking complete -> Going to Undock");
      docking_count_ ++;
      ROS_INFO("docking count - %d", docking_count_);
    }
  }

  /* =================== DELAY HANDLING =================== */

  void startDelayTimer(double delay_sec)
  {
    delay_timer_ = nh_.createTimer(
        ros::Duration(delay_sec),
        &MissionController::delayTimerCB,
        this,
        true);
  }

  void delayTimerCB(const ros::TimerEvent&)
  {
    state_ = next_state_;
    sendGoal(delayed_goal_);
   // ROS_INFO("Delayed transition → goal published");
  }

  /* =================== HELPERS =================== */

  void sendGoal(const geometry_msgs::PoseStamped& goal)
  {
    geometry_msgs::PoseStamped g = goal;
    g.header.stamp = ros::Time::now();
    goal_pub_.publish(g);
  }

  geometry_msgs::PoseStamped loadPose(const std::string& name)
  {
    geometry_msgs::PoseStamped p;
    std::string frame_id;
    nh_.param<std::string>("frame_id", frame_id, "map");

    double x, y, yaw;
    if (!nh_.getParam(name + "/x", x) ||
        !nh_.getParam(name + "/y", y) ||
        !nh_.getParam(name + "/yaw", yaw)) {
      ROS_FATAL("Missing parameters for pose: %s", name.c_str());
      ros::shutdown();
    }

    p.header.frame_id = frame_id;
    p.pose.position.x = x;
    p.pose.position.y = y;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    p.pose.orientation = tf2::toMsg(q);

    ROS_INFO("Loaded %s → x=%.2f y=%.2f yaw=%.2f", name.c_str(), x, y, yaw);
    return p;
  }

  void loadPoses()
  {
    point_a_ = loadPose("point_a");
    point_b_ = loadPose("point_b");
    point_c_ = loadPose("point_c");
    predock_ = loadPose("predock");
    undock_  = loadPose("undock");
  }
};

/* =================== MAIN =================== */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mission_controller");
  MissionController mc;
  ros::spin();
  return 0;
}
