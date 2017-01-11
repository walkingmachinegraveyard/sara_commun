#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>
#include <ros/time.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/function.hpp>
#include <math.h>

namespace wm_mecanum_controller_ns
{
  /**
   * This class makes some assumptions on the model of the robot:
   *  - the rotation axes of wheels are collinear
   *  - the wheels are identical in radius
   * Additional assumptions to not duplicate information readily available in the URDF:
   *  - the wheels have the same parent frame
   *  - a wheel collision geometry is a cylinder in the urdf
   *  - a wheel joint frame center's vertical projection on the floor must lie within the contact patch
   */
    class WMMecanumController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
        public:
          WMMecanumController();

          /**
           * \brief Initialize controller
           * \param hw            Velocity joint interface for the wheels
           * \param root_nh       Node handle at root namespace
           * \param controller_nh Node handle inside the controller namespace
           */
            bool init(hardware_interface::VelocityJointInterface* hw,
              ros::NodeHandle& root_nh,
              ros::NodeHandle &controller_nh);

            /**
             * \brief Updates controller, i.e. computes the odometry and sets the new velocity commands
             * \param time   Current time
             * \param period Time since the last called to update
             */
            void update(const ros::Time& time, const ros::Duration& period);

            /**
             * \brief Starts controller
             * \param time Current time
             */
            void starting(const ros::Time& time);

            /**
             * \brief Stops controller
             * \param time Current time
             */
            void stopping(const ros::Time& time);

        private:
            std::string name_;
            /// Odometry related:
            ros::Duration publish_period_;
            ros::Time last_state_publish_time_;

            // Velocity command related:
            struct Commands
            {
              urdf::Vector3 lin;
              urdf::Vector3 ang;
              ros::Time stamp;

              Commands() : lin(0.0,0.0,0.0), ang(0.0,0.0,0.0), stamp(0.0) {}
            };

            // Joint names
            std::string front_left_wheel_name_;
            std::string front_right_wheel_name_;
            std::string rear_left_wheel_name_;
            std::string rear_right_wheel_name_;

            // Hardware handles:
            hardware_interface::JointHandle front_right_wheel_joint_;
            hardware_interface::JointHandle front_left_wheel_joint_;
            hardware_interface::JointHandle rear_right_wheel_joint_;
            hardware_interface::JointHandle rear_left_wheel_joint_;

            // realtime stuff
            realtime_tools::RealtimeBuffer<Commands> command_;
            Commands command_struct_;

            // Subscriber
            ros::Subscriber sub_command_;
            ros::Subscriber sub_joint_state_;

            /// Odometry related:
            boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
            boost::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;

            /// Wheel separation
            double x_wheel_to_center_;
            double y_wheel_to_center_;
            double front_dist_to_center_;
            double rear_dist_to_center_;

            // Robot position
            double robot_pose_x_;
            double robot_pose_y_;
            double robot_pose_orientation_;

            /// Wheel radius:
            double wheel_radius_;

            double gb_ratio_;

            /// Timeout to consider cmd_vel commands old:
            double cmd_vel_timeout_;

            /// Frame to use for the robot base:
            std::string base_frame_id_;

            /// Whether to publish odometry to tf or not:
            bool enable_odom_tf_;

            /// Speed limiters:
            Commands last1_cmd_;
            Commands last0_cmd_;

            // Speeds
            double linear_speed_;
            double angular_speed_;
            double lin_max_vel_;
            double ang_max_vel_;

            // Flag to indicate if joint_state_ has been read
            bool read_state_;

            void InverseKinematics(struct Commands * cmd, double W[4]);

            void brake();

            void cmdVelCallback(const geometry_msgs::Twist& command);
    };
    PLUGINLIB_EXPORT_CLASS(wm_mecanum_controller_ns::WMMecanumController, controller_interface::ControllerBase);
}
