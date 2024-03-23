#include "me5413_world/path_tracker_node.hpp"
#include "me5413_world/mpc_controller.hpp" // 假设这是MPC控制器的实现

namespace me5413_world {

// Dynamic Parameters
double SPEED_TARGET;
bool PARAMS_UPDATED;

void dynamicParamCallback(me5413_world::path_trackerConfig& config, uint32_t level) {
  // Common Params
  SPEED_TARGET = config.speed_target;

  PARAMS_UPDATED = true;
};

class PathTrackerNode {
public:
  PathTrackerNode() : tf2_listener_(tf2_buffer_), mpc_controller_() {
    f = boost::bind(&dynamicParamCallback, _1, _2);
    server.setCallback(f);

    this->sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &PathTrackerNode::robotOdomCallback, this);
    this->sub_local_path_ = nh_.subscribe("/me5413_world/planning/local_path", 1, &PathTrackerNode::localPathCallback, this);
    this->pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 1);

    // Initialization
    this->robot_frame_ = "base_link";
    this->world_frame_ = "world";
  };

private:
  void localPathCallback(const nav_msgs::Path::ConstPtr& path);
  void robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom);
  geometry_msgs::Twist computeMPCControl(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal);

  // MPC控制器对象
  MPCController mpc_controller_;
};

void PathTrackerNode::localPathCallback(const nav_msgs::Path::ConstPtr& path) {
  this->pose_world_goal_ = path->poses.back().pose; // 假设目标是路径的最后一个点
  this->pub_cmd_vel_.publish(computeMPCControl(this->odom_world_robot_, this->pose_world_goal_));
}

void PathTrackerNode::robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
  this->odom_world_robot_ = *odom.get();
}

geometry_msgs::Twist PathTrackerNode::computeMPCControl(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal) {
  // 使用MPC控制器计算控制输出
  auto control_outputs = mpc_controller_.calculateControl(odom_robot, pose_goal, SPEED_TARGET);
  return control_outputs;
}

} // namespace me5413_world

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_tracker_node");
  me5413_world::PathTrackerNode path_tracker_node;
  ros::spin(); // spin the ros node.
  return 0;
}
