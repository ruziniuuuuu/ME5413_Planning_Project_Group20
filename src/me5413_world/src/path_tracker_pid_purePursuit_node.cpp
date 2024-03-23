
#include "me5413_world/math_utils.hpp"
#include "me5413_world/path_tracker_pid_purePursuit_node.hpp"

namespace me5413_world
{

// Dynamic Parameters
    double SPEED_TARGET;
    double PID_Kp, PID_Ki, PID_Kd;
    double purePursuit_DistanceAhead; // Pure pursuit look ahead distance
    bool PARAMS_UPDATED;
    double yaw_angular_error = 3;

    void dynamicParamCallback(const me5413_world::path_tracker_pid_purePursuitConfig& config,uint32_t level)
    {
        // Common Params
        SPEED_TARGET = config.speed_target;
        // PID
        PID_Kp = config.PID_Kp;
        PID_Ki = config.PID_Ki;
        PID_Kd = config.PID_Kd;

        // Load ahead distance
        purePursuit_DistanceAhead = config.purePursuit_DistanceAhead;

        PARAMS_UPDATED = true;
    }


    PathTrackerPIDPurePursuit::PathTrackerPIDPurePursuit() : tf2_listener_(tf2_buffer_)
    {
        f = boost::bind(&dynamicParamCallback, _1, _2);
        server.setCallback(f);

        this->sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &PathTrackerPIDPurePursuit::robotOdomCallback, this);
        this->sub_local_path_ = nh_.subscribe("/me5413_world/planning/local_path", 1, &PathTrackerPIDPurePursuit::localPathCallback, this);
        this->pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 1);

        // Initialization
        this->robot_frame_ = "base_link";
        this->world_frame_ = "world";

        this->pid_ = control::PID(0.1, 1.0, -1.0, PID_Kp, PID_Ki, PID_Kd);
    }

//    void PathTrackerPIDPurePursuit::localPathCallback(const nav_msgs::Path::ConstPtr& path)
//    {
//        geometry_msgs::PoseStamped target_pose = selectTargetPose(path);

//        geometry_msgs::Twist cmd_vel = computeControlOutputs(this->odom_world_robot_, target_pose.pose);

//        this->pub_cmd_vel_.publish(cmd_vel);
//        this->pub_cmd_vel_.publish(computeControlOutputs(this->odom_world_robot_, this->pose_world_goal_));

//        return;
//    }

    void PathTrackerPIDPurePursuit::localPathCallback(const nav_msgs::Path::ConstPtr& path)
    {
        this->pub_cmd_vel_.publish(computeControlOutputs(this->odom_world_robot_, path));
        return;
    }


    void PathTrackerPIDPurePursuit::robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
    {
        this->world_frame_ = odom->header.frame_id;
        this->robot_frame_ = odom->child_frame_id;
        this->odom_world_robot_ = *odom;

        return;
    }


    geometry_msgs::PoseStamped PathTrackerPIDPurePursuit::purePursuitGoalPose(const tf2::Vector3& point_robot, const nav_msgs::Path::ConstPtr& path, double purePursuit_DistanceAhead)
    {
        int low = 0, high = path->poses.size() - 1, mid;
        double min_distance = std::numeric_limits<double>::max();
        int closest_idx = 0;

        while (low <= high) {
            mid = low + (high - low) / 2;
            tf2::Vector3 point_path;
            tf2::fromMsg(path->poses[mid].pose.position, point_path);
            double distance = tf2::tf2Distance(point_robot, point_path);
            if (distance < min_distance) {
                min_distance = distance;
                closest_idx = mid;
            }
            if (distance < purePursuit_DistanceAhead) {
                low = mid + 1;
            } else {
                high = mid - 1;
            }
        }

        int goal_idx = closest_idx;
        tf2::Vector3 point_path;
        tf2::fromMsg(path->poses[goal_idx].pose.position, point_path);
        double distance = tf2::tf2Distance(point_robot, point_path);
        while (distance < purePursuit_DistanceAhead && goal_idx < path->poses.size() - 1) {
            ++goal_idx;
            tf2::fromMsg(path->poses[goal_idx].pose.position, point_path);
            distance = tf2::tf2Distance(point_robot, point_path);
        }

        return path->poses[goal_idx];
    }

    double PathTrackerPIDPurePursuit::purePursuit_normalizeHeading(double angle)
    {
        angle = fmod(angle, 2.0 * M_PI); // Ensure the angle is within [0, 2π)
        if (angle < 0)
            angle += 2.0 * M_PI; // Adjust if negative
        if (angle >= M_PI)
            angle -= 2.0 * M_PI; // Shift to [-π, π) if necessary
        return angle;
    }

    // Function to calculate the control output with direct geometric angle for steering
    geometry_msgs::Twist PathTrackerPIDPurePursuit::computeControlOutputs(const nav_msgs::Odometry& odom_robot, const nav_msgs::Path::ConstPtr& path)
    {
        // Extract robot's orientation and position
        tf2::Quaternion q_robot;
        tf2::fromMsg(odom_robot.pose.pose.orientation, q_robot);
        const tf2::Matrix3x3 m_robot(q_robot);
        double roll, pitch, yaw_robot;
        m_robot.getRPY(roll, pitch, yaw_robot);
        tf2::Vector3 point_robot;
        tf2::fromMsg(odom_robot.pose.pose.position, point_robot);

        // Change velocity based on PID controller
        tf2::Vector3 robot_vel;
        tf2::fromMsg(odom_world_robot_.twist.twist.linear, robot_vel);
        const double velocity = robot_vel.length();
        geometry_msgs::Twist cmd_vel;
        if (PARAMS_UPDATED)
        {
            this->pid_.updateSettings(PID_Kp, PID_Ki, PID_Kd);
            PARAMS_UPDATED = false;
        }
        cmd_vel.linear.x = this->pid_.calculate(SPEED_TARGET, velocity);

        // Find the goal point and calculate the error
        geometry_msgs::PoseStamped goal_pose = purePursuitGoalPose(point_robot, path, purePursuit_DistanceAhead);
        double yaw_goal = atan2(goal_pose.pose.position.y - point_robot.y(), goal_pose.pose.position.x - point_robot.x());
        double yaw_error = yaw_goal - yaw_robot;

        // Normalize yaw_error to be within [-pi, pi)
        yaw_error = purePursuit_normalizeHeading(yaw_error);

        // Controller output angle control
        cmd_vel.angular.z = yaw_angular_error * yaw_error;

        return cmd_vel;
    }


} // namespace me5413_world

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_tracker_node");
    me5413_world::PathTrackerPIDPurePursuit path_tracker_node;
    ros::spin();  // spin the ros node.
    return 0;
}


