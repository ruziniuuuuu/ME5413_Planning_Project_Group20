#include "me5413_world/path_tracker_pid_stanley_improved_node.hpp"

namespace me5413_world {

// Dynamic Parameters
    double SPEED_TARGET;
    double PID_Kp, PID_Ki, PID_Kd;
    double STANLEY_K;
    double prev_STANLEY_K = 1.0;
    double prev_velocity = 0.0;
    bool PARAMS_UPDATED;

    const double PathTrackerNode::TURNING_SPEED = 0.2;
    const double PathTrackerNode::STANLEY_GAIN_TURNING = 0.6;
    const double PathTrackerNode::HEADING_ERROR_THRESHOLD = 0.05;
    const double PathTrackerNode::CROSS_TRACK_ERROR_THRESHOLD = 0.3;

    void dynamicParamCallback(me5413_world::path_tracker_pid_stanleyConfig& config, uint32_t level)
    {
        // Common Params
        SPEED_TARGET = config.speed_target;
        // PID
        PID_Kp = config.PID_Kp;
        PID_Ki = config.PID_Ki;
        PID_Kd = config.PID_Kd;
        // Stanley
        STANLEY_K = config.stanley_K;

        PARAMS_UPDATED = true;
    };

    PathTrackerNode::PathTrackerNode() : tf2_listener_(tf2_buffer_)
    {
        f = boost::bind(&dynamicParamCallback, _1, _2);
        server.setCallback(f);

        this->sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &PathTrackerNode::robotOdomCallback, this);
        this->sub_local_path_ = nh_.subscribe("/me5413_world/planning/local_path", 1, &PathTrackerNode::localPathCallback, this);
        this->pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 1);

        // Initialization
        this->robot_frame_ = "base_link";
        this->world_frame_ = "world";

        this->pid_ = control::PID(0.1, 1.0, -1.0, PID_Kp, PID_Ki, PID_Kd);
    };

    void PathTrackerNode::localPathCallback(const nav_msgs::Path::ConstPtr& path)
    {
        // Calculate absolute errors (wrt to world frame)
        this->pose_world_goal_ = path->poses[11].pose;
        this->pub_cmd_vel_.publish(computeControlOutputs(this->odom_world_robot_, this->pose_world_goal_));
        return;
    };

    void PathTrackerNode::robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
    {
        this->world_frame_ = odom->header.frame_id;
        this->robot_frame_ = odom->child_frame_id;
        this->odom_world_robot_ = *odom.get();
        return;
    };

    double PathTrackerNode::computeStanleyControl(const double heading_error, const double cross_track_error, const double velocity,double stanley_gain)
    {
        // Compute the Stanley control output based on heading error, cross-track error, and velocity
        // The control output is calculated as a negative feedback of heading error and the arctangent of cross-track error times gain over velocity.
        double stanley_output = -1.0 * (heading_error + std::atan2(stanley_gain * cross_track_error, std::max(velocity, 0.3)));
        stanley_output = std::min(std::max(stanley_output, -2.2), 2.2);

        // Check for and correct NaN values, which could occur due to invalid operations in the calculation.
        if (std::isnan(stanley_output)) {
            ROS_WARN("Stanley output is NaN, resetting to 0.");
            stanley_output = 0.0;
        }
        return stanley_output;
    };

    void PathTrackerNode::adjustStanleyKBasedOnVelocity(double velocity) {
        const double LOW_SPEED_THRESHOLD = 0.1;
        const double HIGH_SPEED_THRESHOLD = 0.6;
        const double STANLEY_K_MIN = 0.8;
        const double STANLEY_K_MAX = 1.5;
        // Calculate a dynamic smoothing factor based on the change in velocity.
        double dynamicSmoothFactor = 0.5 + std::abs(prev_velocity - velocity) * 0.5;
        dynamicSmoothFactor = std::min(std::max(dynamicSmoothFactor, 0.1), 0.9);

        // Adjust Stanley gain based on the current velocity.
        double newStanleyK;
        if (velocity < LOW_SPEED_THRESHOLD) {
            newStanleyK = STANLEY_K_MAX;
        } else if (velocity > HIGH_SPEED_THRESHOLD) {
            newStanleyK = STANLEY_K_MIN;
        } else {
            double ratio = (velocity - LOW_SPEED_THRESHOLD) / (HIGH_SPEED_THRESHOLD - LOW_SPEED_THRESHOLD);
            newStanleyK = STANLEY_K_MAX - ratio * (STANLEY_K_MAX - STANLEY_K_MIN);
        }

        // Smoothly update the Stanley gain using the dynamic factor and previous gain for smoother transitions.
        STANLEY_K = dynamicSmoothFactor * newStanleyK + (1 - dynamicSmoothFactor) * prev_STANLEY_K;
        prev_STANLEY_K = STANLEY_K;
        prev_velocity = velocity;
    }

    geometry_msgs::Twist PathTrackerNode::computeControlOutputs(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal)
    {
        // Heading Error
        tf2::Quaternion q_robot, q_goal;
        tf2::fromMsg(odom_robot.pose.pose.orientation, q_robot);
        tf2::fromMsg(pose_goal.orientation, q_goal);
        const tf2::Matrix3x3 m_robot = tf2::Matrix3x3(q_robot);
        const tf2::Matrix3x3 m_goal = tf2::Matrix3x3(q_goal);

        double roll, pitch, yaw_robot, yaw_goal;
        m_robot.getRPY(roll, pitch, yaw_robot);
        m_goal.getRPY(roll, pitch, yaw_goal);

        const double heading_error = unifyAngleRange(yaw_robot - yaw_goal);

        // Lateral Error
        tf2::Vector3 point_robot, point_goal;
        tf2::fromMsg(odom_robot.pose.pose.position, point_robot);
        tf2::fromMsg(pose_goal.position, point_goal);
        const tf2::Vector3 V_goal_robot = point_robot - point_goal;
        const double angle_goal_robot = std::atan2(V_goal_robot.getY(), V_goal_robot.getX());
        const double angle_diff = angle_goal_robot - yaw_goal;
        const double lat_error = V_goal_robot.length()*std::sin(angle_diff);

        // Velocity
        tf2::Vector3 robot_vel;
        tf2::fromMsg(this->odom_world_robot_.twist.twist.linear, robot_vel);
        const double velocity = robot_vel.length();

        // Adjust Stanley gain based on current velocity for dynamic response.
        adjustStanleyKBasedOnVelocity(velocity);

        // Implement logic for initial acceleration phase and conditional adjustment of speed and steering gain based on heading and lateral errors.
        static ros::Time start_time = ros::Time::now();
        ros::Duration time_elapsed = ros::Time::now() - start_time;
        const double acceleration_phase_duration = 5.0;

        geometry_msgs::Twist cmd_vel;
        if (time_elapsed.toSec() < acceleration_phase_duration && std::fabs(heading_error) > HEADING_ERROR_THRESHOLD) {
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = std::copysign(STANLEY_GAIN_TURNING, -heading_error);
        } else {
            double adjusted_speed_target = SPEED_TARGET;
            double adjusted_stanley_gain = STANLEY_K;
            // Adjust speed and Stanley gain if turning
            if (std::fabs(heading_error) > HEADING_ERROR_THRESHOLD || std::fabs(lat_error) > CROSS_TRACK_ERROR_THRESHOLD)
            {
                // Adjust target speed and Stanley gain based on the current navigation state.
                adjusted_speed_target = TURNING_SPEED; // Reduce speed when turning
                adjusted_stanley_gain = STANLEY_GAIN_TURNING; // Use adjusted Stanley gain when turning
            }
            if (PARAMS_UPDATED)
            {
                this->pid_.updateSettings(PID_Kp, PID_Ki, PID_Kd);
                PARAMS_UPDATED = false;
            }
            // Calculate linear and angular velocities using PID for speed control and Stanley control for steering.
            cmd_vel.linear.x = this->pid_.calculate(adjusted_speed_target, velocity);
            cmd_vel.angular.z = computeStanleyControl(heading_error, lat_error, velocity, adjusted_stanley_gain);
        }

        // Apply a simple filter to smooth the transition between control commands.
        static double prev_linear_x = 0.0;
        static double prev_angular_z = 0.0;
        const double filter_factor = 0.5;
        cmd_vel.linear.x = filter_factor * cmd_vel.linear.x + (1 - filter_factor) * prev_linear_x;
        cmd_vel.angular.z = filter_factor * cmd_vel.angular.z + (1 - filter_factor) * prev_angular_z;
        prev_linear_x = cmd_vel.linear.x;
        prev_angular_z = cmd_vel.angular.z;

        std::cout << "robot velocity is " << velocity << " throttle is " << cmd_vel.linear.x << std::endl;
        std::cout << "lateral error is " << lat_error << " heading_error is " << heading_error << " steering is " << cmd_vel.angular.z << std::endl;

        return cmd_vel;
    }

} // namespace me5413_world

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_tracker_node");
    me5413_world::PathTrackerNode path_tracker_node;
    ros::spin();  // spin the ros node.
    return 0;
}