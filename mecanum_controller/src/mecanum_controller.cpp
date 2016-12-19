#include <cmath>

#include <tf/transform_datatypes.h>

#include <urdf_parser/urdf_parser.h>

#include <boost/assign.hpp>
#include <ros/console.h>

#include <_controller/aawd_controller.h>

#define PI 3.1415926535


namespace wm_controller
{
    MecanumController::MecanumController():
        command_struct_()
        ,linear_speed_(0.0)
        ,angular_speed_(0.0)
        ,robot_pose_x_(0.0)
        ,robot_pose_y_(0.0)
        ,robot_pose_orientation_(0.0)
        ,front_wheelbase_(0.0)
        ,rear_wheelbase_(0.0)
        ,front_dist_to_center_(0.0)
        ,rear_dist_to_center_(0.0)
        ,wheel_radius_(0.0)
        ,loop_count_(0)
        ,cmd_vel_timeout_(0.5)
        ,base_frame_id_("base_link")
        ,read_state_(false)
        ,i_front_left_vel_(0)
        ,i_front_right_vel_(1)
        ,i_rear_left_vel_(2)
        ,i_rear_right_vel_(3)
        ,enable_odom_tf_(false)
        {}

    bool MecanumController::init(hardware_interface::PositionJointInterface* hw,
                              ros::NodeHandle& root_nh,
                              ros::NodeHandle &controller_nh)
    {
        const std::string complete_ns = controller_nh.getNamespace();
        std::size_t id = complete_ns.find_last_of("/");
        name_ = complete_ns.substr(id + 1);

        // get parameters
        // x axis distance between wheel axis and the robot's centroid
        // python code // self.alpha = rospy.get_param('alpha', 0.31)    # in meter
        controller_nh.param<double>("alpha", alpha_, 0.31 );
    
        // y axis distance between wheel radial median and the robot'S centroid
        // python code // self.beta = rospy.get_param('beta', 0.30)    # in meter
        controller_nh.param<double>("beta", beta_, 0.30 );
    
        // Wheel radius
        // python code // self.radius = rospy.get_param('wheel_radius', 0.075)    # wheel radius, in meter
        controller_nh.param<double>("wheel_radius", wheel_radius_, 0.075 );
    
        // max linear velocity, in m/s
        // python code // self.maxLinearVelocity = float(rospy.get_param('max_linear_vel', 1))
    

        // max angular velocity, in rad/s
        divisor = rospy.get_param('angular_vel_div', 6)
        self.maxAngularVelocity = pi/divisor
    
        // gearbox ratio
        self.gb_ratio = rospy.get_param('gearbox_ratio', 15.0)


        // Get joint names from the parameter server
        controller_nh.param("front_left_wheel_joint", front_left_wheel_name_, front_left_wheel_name_);
        ROS_INFO_STREAM_NAMED(name_, "Front left wheel joint  is : " << front_left_wheel_name_);

        // Get joint names from the parameter server
        controller_nh.param("front_right_wheel_joint", front_right_wheel_name_, front_right_wheel_name_);
        ROS_INFO_STREAM_NAMED(name_, "Front right wheel joint  is : " << front_right_wheel_name_);

        // Get joint names from the parameter server
        controller_nh.param("rear_left_wheel_joint", rear_left_wheel_name_, rear_left_wheel_name_);
        ROS_INFO_STREAM_NAMED(name_, "rear left wheel joint  is : " << rear_left_wheel_name_);

        // Get joint names from the parameter server
        controller_nh.param("rear_right_wheel_joint", rear_right_wheel_name_, rear_right_wheel_name_);
        ROS_INFO_STREAM_NAMED(name_, "Front right wheel joint  is : " << rear_right_wheel_name_);

        // Odometry related:
        double publish_rate;
        controller_nh.param("publish_rate", publish_rate, 50.0);
        ROS_INFO_STREAM_NAMED(name_, "Controller state will be published at "
                              << publish_rate << "Hz.");
        publish_period_ = ros::Duration(1.0 / publish_rate);

        // Twist command related:
        controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, cmd_vel_timeout_);
        ROS_INFO_STREAM_NAMED(name_, "Velocity commands will be considered old if they are older than "
                              << cmd_vel_timeout_ << "s.");

        controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
        ROS_INFO_STREAM_NAMED(name_, "Base frame_id set to " << base_frame_id_);

        controller_nh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);
        ROS_INFO_STREAM_NAMED(name_, "Publishing to tf is " << (enable_odom_tf_?"enabled":"disabled"));

        // Velocity and acceleration limits:
        controller_nh.param("linear/has_velocity_limits"    , limiter_lin_.has_velocity_limits    , limiter_lin_.has_velocity_limits    );
        controller_nh.param("linear/has_acceleration_limits", limiter_lin_.has_acceleration_limits, limiter_lin_.has_acceleration_limits);
        controller_nh.param("linear/has_jerk_limits"        , limiter_lin_.has_jerk_limits        , limiter_lin_.has_jerk_limits        );
        controller_nh.param("linear/max_velocity"           , limiter_lin_.max_velocity           ,  limiter_lin_.max_velocity          );
        controller_nh.param("linear/min_velocity"           , limiter_lin_.min_velocity           , -limiter_lin_.max_velocity          );
        controller_nh.param("linear/max_acceleration"       , limiter_lin_.max_acceleration       ,  limiter_lin_.max_acceleration      );
        controller_nh.param("linear/min_acceleration"       , limiter_lin_.min_acceleration       , -limiter_lin_.max_acceleration      );
        controller_nh.param("linear/max_jerk"               , limiter_lin_.max_jerk               ,  limiter_lin_.max_jerk              );
        controller_nh.param("linear/min_jerk"               , limiter_lin_.min_jerk               , -limiter_lin_.max_jerk              );

        controller_nh.param("angular/has_velocity_limits"    , limiter_ang_.has_velocity_limits    , limiter_ang_.has_velocity_limits    );
        controller_nh.param("angular/has_acceleration_limits", limiter_ang_.has_acceleration_limits, limiter_ang_.has_acceleration_limits);
        controller_nh.param("angular/has_jerk_limits"        , limiter_ang_.has_jerk_limits        , limiter_ang_.has_jerk_limits        );
        controller_nh.param("angular/max_velocity"           , limiter_ang_.max_velocity           ,  limiter_ang_.max_velocity          );
        controller_nh.param("angular/min_velocity"           , limiter_ang_.min_velocity           , -limiter_ang_.max_velocity          );
        controller_nh.param("angular/max_acceleration"       , limiter_ang_.max_acceleration       ,  limiter_ang_.max_acceleration      );
        controller_nh.param("angular/min_acceleration"       , limiter_ang_.min_acceleration       , -limiter_ang_.max_acceleration      );
        controller_nh.param("angular/max_jerk"               , limiter_ang_.max_jerk               ,  limiter_ang_.max_jerk              );
        controller_nh.param("angular/min_jerk"               , limiter_ang_.min_jerk               , -limiter_ang_.max_jerk              );

        // Robot kinematic parameters
        controller_nh.param<double>("wheel_radius",        wheel_radius_, 0.5 );
        controller_nh.param<double>("front_wheelbase",     front_wheelbase_, 0.5 );
        controller_nh.param<double>("rear_wheelbase",      rear_wheelbase_, 0.5 );
        controller_nh.param<double>("front_dist_to_center", front_dist_to_center_, 0.5 );
        controller_nh.param<double>("rear_dist_to_center", rear_dist_to_center_, 0.5 );

        ROS_INFO_STREAM_NAMED(name_,
                              "Parameter loaded with wheel_radius: "   << wheel_radius_
                              << " and front_wheelbase: "              << front_wheelbase
                              << " and rear_wheelbase: "               << rear_wheelbase_
                              << " and front_dist_to_center: "         << front_dist_to_center_
                              << " and rear_dist_to_center: "          << rear_dist_to_center_ );

        // Get the joint object to use in the realtime loop

        ROS_INFO_STREAM_NAMED(name_,
                            "Adding  front left wheel with joint name: "   << front_left_wheel_name_
                            << " and front right wheel with joint name: " << front_right_wheel_name_
                            << " and rear left  wheel with joint name: "    << rear_left_wheel_name_
                            << " and rear right wheel with joint name: "   << rear_right_wheel_name_ );

        front_left_wheel_joint_    = hw->getHandle(front_left_wheel_name_);  // throws on failure
        front_right_wheel_joint_    = hw->getHandle(front_right_wheel_name_);  // throws on failure
        rear_left_wheel_joint_    = hw->getHandle(rear_left_wheel_name_);  // throws on failure
        rear_right_wheel_joint_    = hw->getHandle(rear_right_wheel_name_);  // throws on failure

        // Start realtime state publisher
        //controller_state_publisher_.reset(
        //new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(n, "state", 1));

        setOdomPubFields(root_nh, controller_nh);

        sub_command_ = controller_nh.subscribe("cmd_vel", 1, &AawdController::cmdVelCallback, this);
        sub_joint_state_ = controller_nh.subscribe<sensor_msgs::JointState>("/joint_states", 1, &AawdController::jointStateCallback, this);
    
        // Initialize joint indexes according to joint names
        if (read_state_)
        {
            std::vector<std::string> joint_names = joint_state_.name;
            i_front_left_vel_ = find(joint_names.begin(), joint_names.end(), std::string(front_left_wheel_name_)) - joint_names.begin();
            i_front_right_vel_= find(joint_names.begin(), joint_names.end(), std::string(front_right_wheel_name_)) - joint_names.begin();
            i_rear_left_vel_  = find(joint_names.begin(), joint_names.end(), std::string(rear_left_wheel_name_)) - joint_names.begin();
            i_rear_right_vel_ = find(joint_names.begin(), joint_names.end(),std:: string(rear_right_wheel_name_)) - joint_names.begin();
            return 0;
        }
        else
        {
    	    ROS_INFO("Joint State not received");
    	    return -1;
        }

        return true;
    } 
  

    void MecanumController::updateOdometry(const ros::Time& time)
    {
        // Compute Position

        // Linear speed of each wheel

        if(read_state_)
        {
            double v_front, v_rear;
    
            v_front = ((joint_state_.velocity[i_front_left_vel_] +joint_state_.velocity[i_front_right_vel_]) / 2.0) *  (wheel_radius_);
            v_rear =   ((joint_state_.velocity[i_rear_left_vel_]  + joint_state_.velocity[i_rear_right_vel_]) / 2.0) *  (wheel_radius_);
    
            //ROS_INFO("v_front = %.3lf, v_rear = %.3lf", v_front, v_rear);
            //ROS_INFO("v_front_left = %.3lf", joint_state_.velocity[i_front_left_vel_]);
            //ROS_INFO("v_front_right = %.3lf", joint_state_.velocity[i_front_right_vel_] );
    
            linear_speed_ = (v_front + v_rear) / 2.0;

            // Angle of steering
            double a_front, a_rear;
    
            // angle inverted because joint is inverted
            steering_angle_ = radnorm2( steering_joint_.getPosition()) * -1;
       
            // Filter noise
            if (fabs(linear_speed_) < 0.01)
            linear_speed_ = 0.0;
      
            // Compute Odometry
            double r = 10000000;
        
            if (fabs(steering_angle_) > 0.001)
            r = ( front_dist_to_center_ + ( rear_dist_to_center_ / cos( steering_angle_ ) ) ) / tan(steering_angle_);
       
            angular_speed_ = (linear_speed_ / r);
    
            double x_icc = robot_pose_x_ - r * sin(robot_pose_orientation_);
            double y_icc = robot_pose_y_ + r * cos(robot_pose_orientation_);
    
            double w_dt = angular_speed_ * publish_period_.toSec();
        
            robot_pose_orientation_ += w_dt;

            // normalize
            radnorm(&robot_pose_orientation_);
    

            // Positions
            robot_pose_x_ = cos(w_dt) * (robot_pose_x_ - x_icc) - sin(w_dt) * ( robot_pose_y_ - y_icc) + x_icc;
            robot_pose_y_ = sin(w_dt) * (robot_pose_x_ - x_icc) + cos(w_dt) * ( robot_pose_y_ - y_icc) + y_icc;
        }
    }
  
    // Publish robot odometry tf and topic depending
    void MecanumController::publishOdometry(const ros::Time& time)
    {
        // Populate odom message and publish
	if (odom_pub_->trylock())
	{
            // Compute and store orientation info
            const geometry_msgs::Quaternion orientation(
            tf::createQuaternionMsgFromYaw(robot_pose_orientation_));

            odom_pub_->msg_.header.stamp = time;
	    odom_pub_->msg_.pose.pose.position.x = robot_pose_x_;
	    odom_pub_->msg_.pose.pose.position.y =robot_pose_y_;
	    odom_pub_->msg_.pose.pose.orientation = orientation;
            odom_pub_->msg_.twist.twist.linear.x  = linear_speed_;
            odom_pub_->msg_.twist.twist.angular.z = angular_speed_;
            odom_pub_->unlockAndPublish();
        }

        // Publish tf /odom frame
        if (enable_odom_tf_ && tf_odom_pub_->trylock())
        {
            // Compute and store orientation info
	    const geometry_msgs::Quaternion orientation
            tf::createQuaternionMsgFromYaw(robot_pose_orientation_));

	    geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
	    odom_frame.header.stamp = time;
            odom_frame.transform.translation.x = robot_pose_x_;
            odom_frame.transform.translation.y = robot_pose_y_;
            odom_frame.transform.rotation = orientation;
            tf_odom_pub_->unlockAndPublish();
        }
    }

    void AawdController::update(const ros::Time& time, const ros::Duration& period)
    {
        // Update and Publish odometry message
        if (last_state_publish_time_ + publish_period_ < time)
        {
            last_state_publish_time_ += publish_period_;
      
            updateOdometry(time);
            publishOdometry(time);  
        }

        // MOVE ROBOT
        // Retreive current velocity command and time step:
        Commands curr_cmd = *(command_.readFromRT());
        const double dt = (time - curr_cmd.stamp).toSec();

        // Brake if cmd_vel has timeout:
        if (dt > cmd_vel_timeout_)
        {
            curr_cmd.lin = 0.0;
            curr_cmd.ang = 0.0;
        }

        // Limit velocities and accelerations:
        const double cmd_dt(period.toSec());

        limiter_lin_.limit(curr_cmd.lin, last0_cmd_.lin, last1_cmd_.lin, cmd_dt);
        limiter_ang_.limit(curr_cmd.ang, last0_cmd_.ang, last1_cmd_.ang, cmd_dt);

        last1_cmd_ = last0_cmd_;
        last0_cmd_ = curr_cmd;

        ROS_DEBUG_STREAM_NAMED(name_,
                               "Throttle position " << throttle_effort << ", "
                               << "Steering Position: "   << steering_position << "Max velocity "   << limiter_lin_.max_velocity );
        // TODO 
        compute_....; 
   
        front_left_wheel_joint_.setCommand(front_left_wheel_velocity);
        front_right_wheel_joint_.setCommand(front_right_wheel_velocity);
        front_left_wheel_joint_.setCommand(front_left_wheel_velocity);
        front_right_wheel_joint_.setCommand(front_right_wheel_velocity);
    }

    void AawdController::starting(const ros::Time& time)
    {
        brake();

        // Register starting time used to keep fixed rate
        last_state_publish_time_ = time;
    }

    void AawdController::stopping(const ros::Time& /*time*/)
    {
        brake();
    }

    void AawdController::brake()
    { 
        // TODO good joint
        throttle_joint_.setCommand(0.0);
        steering_joint_.setCommand(0.0);
    }
  
    void AawdController::cmdVelCallback(const geometry_msgs::Twist& command)
    {
        command_struct_.ang   = command.angular.z;
        command_struct_.lin   = command.linear.x;
        command_struct_.stamp = ros::Time::now();
        command_.writeFromNonRT (command_struct_);

        ROS_DEBUG_STREAM_NAMED(name_,
                               "Added values to command. "
                               << "Ang: "   << command_struct_.ang << ", "
                               << "Lin: "   << command_struct_.lin << ", "
                               << "Stamp: " << command_struct_.stamp);
    }

} // namespace aawd_controller
