/*
 * wmArm.cpp
 *
 *  Created on: Apr 10, 2016
 *      Author: xhache
 */

#include "wm_arm_driver/wm_arm.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <vector>

namespace manipulator
{

	inline double degToRad(float angle)
	{
		return (double)(M_PI * angle / 180.0);
	}

	wmArm::wmArm(ros::NodeHandle& nh, kinova::kinovaComm& kComm, const moveit::planning_interface::MoveGroup::Options& manipulatorOpt, const moveit::planning_interface::MoveGroup::Options& baseOpt)
				:nh_(nh), kComm_(kComm), manipulatorMoveGroup_(manipulatorOpt), baseMoveGroup_(baseOpt)
	{
		torqueFault_ = false;

		// Init structures
		torqueInfo_.InitStruct();
		angularInfo_.InitStruct();
//		velocityInfo_.InitStruct();	Not used for now

		// get robot information
		frameId_ = baseMoveGroup_.getPlanningFrame();
		ROS_INFO("Planning frame is %s", frameId_.c_str());

		baseGroupActiveJoints_ = baseMoveGroup_.getActiveJoints();
		baseGroupNbJoints_ = baseGroupActiveJoints_.size();

		manipulatorGroupActiveJoints_ = manipulatorMoveGroup_.getActiveJoints();
		manipulatorGroupNbJoints_ = manipulatorGroupActiveJoints_.size();

		baseEndEffectorName_ = baseMoveGroup_.getEndEffector();
		manipulatorEndEffectorName_ = manipulatorMoveGroup_.getEndEffector();

		baseMoveGroup_.allowReplanning(true);
		manipulatorMoveGroup_.allowReplanning(true);

		// get parameters from the parameter server
		double tmpTorque;
		nh_.param("/wm_arm_driver_node/max_torque_joint1", tmpTorque, 8.0);
		maxTorqJoint1_ = (float)tmpTorque;
		nh_.param("/wm_arm_driver_node/max_torque_joint2", tmpTorque, 8.0);
		maxTorqJoint2_ = (float)tmpTorque;
		nh_.param("/wm_arm_driver_node/max_torque_joint3", tmpTorque, 7.0);
		maxTorqJoint3_ = (float)tmpTorque;
		nh_.param("/wm_arm_driver_node/max_torque_joint4", tmpTorque, 6.0);
		maxTorqJoint4_ = (float)tmpTorque;
		nh_.param("/wm_arm_driver_node/max_torque_joint5", tmpTorque, 4.0);
		maxTorqJoint5_ = (float)tmpTorque;
		nh_.param("/wm_arm_driver_node/max_torque_joint1", tmpTorque, 4.0);
		maxTorqJoint6_ = (float)tmpTorque;

		jointsOffset_ = kComm_.getJointsOffset();

		// moveIt! related parameters
		double planningTime;
		nh_.param("/wm_arm_driver_node/planning_time", planningTime, 10.0);
		baseMoveGroup_.setPlanningTime(planningTime);
		manipulatorMoveGroup_.setPlanningTime(planningTime);

		int planningAttempts;
		nh_.param("/wm_arm_driver_node/planning_attempts", planningAttempts, 1);
		baseMoveGroup_.setNumPlanningAttempts(planningAttempts);
		manipulatorMoveGroup_.setNumPlanningAttempts(planningAttempts);

		double positionTol, orientationTol;
		nh_.param("/wm_arm_driver_node/goal_position_tolerance", positionTol, 0.01);
		baseMoveGroup_.setGoalPositionTolerance(positionTol);
		manipulatorMoveGroup_.setGoalPositionTolerance(positionTol);
		nh_.param("/wm_arm_driver_node/goal_orientation_tolerance", orientationTol, 0.1);
		baseMoveGroup_.setGoalOrientationTolerance(orientationTol);
		manipulatorMoveGroup_.setGoalOrientationTolerance(orientationTol);

		std::string basePlanner = baseMoveGroup_.getDefaultPlannerId(baseMoveGroup_.getName());
		nh_.param("/wm_arm_driver_node/base_planner", basePlanner, basePlanner);
		baseMoveGroup_.setPlannerId(basePlanner);
		std::string manipulatorPlanner = manipulatorMoveGroup_.getDefaultPlannerId(manipulatorMoveGroup_.getName());
		nh_.param("/wm_arm_driver_node/manipulator_planner", manipulatorPlanner, manipulatorPlanner);
		manipulatorMoveGroup_.setPlannerId(manipulatorPlanner);

		// set services
		stopControlSrv_ = nh_.advertiseService("stop_control", &wmArm::stopControlService, this);
		startControlSrv_ = nh_.advertiseService("start_control", &wmArm::startControlService, this);
		homeArmSrv_ = nh_.advertiseService("home", &wmArm::homeArmService, this);
		recoverTorqueFaultSrv_ = nh_.advertiseService("recover_torque_fault", &wmArm::recoverTorqueFaultService, this);
		statusSrv_ = nh_.advertiseService("status", &wmArm::armStatusService, this);
		computeBasePlanSrv_ = nh_.advertiseService("compute_base_plan", &wmArm::computeBasePlanService, this);
		computeManipulatorPlanSrv_ = nh_.advertiseService("compute_manipulator_plan", &wmArm::computeManipulatorPlanService, this);

		// set publishers
		jointStatePub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);

		double torqueRate;
		nh_.param("/wm_arm_driver_node/torque_callback_rate", torqueRate, 20.0);
		TorqueCbTimer_ = nh_.createTimer(ros::Duration(1.0/torqueRate), &wmArm::torqueCallback, this);

		double positionRate;
		nh_.param("/wm_arm_driver_node/position_callback_rate", positionRate, 20.0);
		PositionCbTimer_ = nh_.createTimer(ros::Duration(1.0/positionRate), &wmArm::positionCallback, this);
/*
		double velocityRate;
		nh_.param("/wm_arm_driver_node/velocity_callback_rate", velocityRate, 20.0);
		VelocityCbTimer_ = nh_.createTimer(ros::Duration(1.0/velocityRate), &wmArm::velocityCallback, this);
		*/
	}

	wmArm::~wmArm()
	{

	}

	bool wmArm::stopControlService(wm_arm_msgs::stopArmControl::Request& req, wm_arm_msgs::stopArmControl::Response& res)
	{
		int status = kComm_.myStopControlAPI();

		kComm_.myEraseAllTrajectories();

		if (status == NO_ERROR_KINOVA)
		{
			res.stopResult = res.STOP_SUCCESS;
		}
		else
		{
			res.stopResult = res.STOP_FAILURE;
		}
		return true;
	}

	bool wmArm::startControlService(wm_arm_msgs::startArmControl::Request& req, wm_arm_msgs::startArmControl::Response& res)
	{
		int status = kComm_.myStartControlAPI();

		if (status == NO_ERROR_KINOVA)
		{
			res.startResult = res.START_SUCCESS;
		}
		else
		{
			res.startResult = res.START_FAILURE;
		}
		return true;
	}

	bool wmArm::homeArmService(wm_arm_msgs::homeArm::Request& req, wm_arm_msgs::homeArm::Response& res)
	{
		int status = kComm_.myMoveHome();

		if (status == NO_ERROR_KINOVA)
		{
			res.homingResult = res.HOMING_SUCCESS;
		}
		else
		{
			res.homingResult = res.HOMING_FAILURE;
		}
		return true;
	}

	bool wmArm::recoverTorqueFaultService(wm_arm_msgs::recoverTorqFault::Request& req, wm_arm_msgs::recoverTorqFault::Response& res)
	{
		/*
		 * TODO
		 */
		res.recoverResult = res.RECOVER_FAILURE;
		return true;
	}

	bool wmArm::armStatusService(wm_arm_msgs::armStatus::Request& req, wm_arm_msgs::armStatus::Response& res)
	{
		/*
		 * TODO
		 */
		return true;
	}

	bool wmArm::computeBasePlanService(wm_arm_msgs::computePlan::Request& req, wm_arm_msgs::computePlan::Response& res)
	{
		if (req.planningSpace != req.CARTESIAN_SPACE)
		{
			res.planningResult = res.PLANNING_FAILURE;
			ROS_ERROR("Planning with base requires a cartesian pose.");
		}

		else
		{
			moveit::planning_interface::MoveGroup::Plan myPlan;

			bool success = computeBasePlan(req.targetPose, myPlan);

			if (success)
			{
				res.planningResult = res.PLANNING_SUCCESS;
				res.trajectory = myPlan.trajectory_;
			}
			else
			{
				res.planningResult = res.PLANNING_FAILURE;
			}
		}

		return true;
	}

	bool wmArm::computeManipulatorPlanService(wm_arm_msgs::computePlan::Request& req, wm_arm_msgs::computePlan::Response& res)
	{
		moveit::planning_interface::MoveGroup::Plan myPlan;
		bool success = false;

		if (req.planningSpace == req.CARTESIAN_SPACE)
		{
//			std::vector<moveit_msgs::CollisionObject> collisionObjects;
//			collisionObjects.push_back(req.collisionObject);
//			planningSceneInterface_.addCollisionObjects(collisionObjects);

//			ros::Duration(0.5).sleep();

//			manipulatorMoveGroup_.attachObject(req.collisionObject.id);

//			ros::Duration(0.5).sleep();

			success = computeManipulatorPlan(req.targetPose, myPlan);
		}
		else if (req.planningSpace == req.JOINT_SPACE)
		{
			if (req.jointPos.size() != manipulatorGroupNbJoints_)
			{
				ROS_ERROR("Planning: number of joints mismatch");
			}
			success = computeManipulatorPlan(req.jointPos, myPlan);
		}
		else
		{
			ROS_ERROR("Unknown planning space.");
		}

		if (success)
		{
			res.planningResult = res.PLANNING_SUCCESS;
			res.trajectory = myPlan.trajectory_;
		}
		else
		{
			res.planningResult = res.PLANNING_FAILURE;
		}

		return true;
	}

	void wmArm::torqueCallback(const ros::TimerEvent& e)
	{
		int status = kComm_.myGetAngularForce(torqueInfo_);

		if (status != NO_ERROR_KINOVA)
		{
			ROS_WARN("Could not get torque information.");
		}

		if (pow(torqueInfo_.Actuators.Actuator1, 2) > pow(maxTorqJoint1_, 2) ||
			pow(torqueInfo_.Actuators.Actuator2, 2) > pow(maxTorqJoint2_, 2) ||
			pow(torqueInfo_.Actuators.Actuator3, 2) > pow(maxTorqJoint3_, 2) ||
			pow(torqueInfo_.Actuators.Actuator4, 2) > pow(maxTorqJoint4_, 2) ||
			pow(torqueInfo_.Actuators.Actuator5, 2) > pow(maxTorqJoint5_, 2) ||
			pow(torqueInfo_.Actuators.Actuator6, 2) > pow(maxTorqJoint6_, 2))
		{
			kComm_.myStopControlAPI();
			if (!torqueFault_)
			{
				ROS_ERROR("Torque limit exceeded. Stopping the arm.");
			}
			torqueFault_ = true;
		}
	}

	void wmArm::positionCallback(const ros::TimerEvent& e)
	{
		int status = kComm_.myGetAngularPosition(angularInfo_);

		if (status == NO_ERROR_KINOVA)
		{
			sensor_msgs::JointState js;
			js.header.stamp = ros::Time::now();
			js.header.frame_id = frameId_;
			js.name = baseGroupActiveJoints_;

			std::vector<float> tmp;

			tmp.push_back(angularInfo_.Actuators.Actuator1 - jointsOffset_.at(0));
			tmp.push_back(angularInfo_.Actuators.Actuator2 - jointsOffset_.at(1));
			tmp.push_back(angularInfo_.Actuators.Actuator3 - jointsOffset_.at(2));
			tmp.push_back(angularInfo_.Actuators.Actuator4 - jointsOffset_.at(3));
			tmp.push_back(angularInfo_.Actuators.Actuator5 - jointsOffset_.at(4));
			tmp.push_back(angularInfo_.Actuators.Actuator6 - jointsOffset_.at(5));

			for (unsigned int i = 0; i < baseGroupNbJoints_ - manipulatorGroupNbJoints_; i++)
			{
				js.position.push_back(0.0);
			}

			for (unsigned int j = 0; j < manipulatorGroupNbJoints_; j++)
			{
				js.position.push_back(degToRad(tmp.at(j)));
			}

			jointStatePub_.publish(js);
		}
		else
		{
			ROS_WARN("Could not get joint position information.");
		}
		return;
	}
/*
	void wmArm::velocityCallback(const ros::TimerEvent& e)
	{
		// TODO
		return;
	}
*/
	int wmArm::recoverTorqueFault()
	{
		if (torqueFault_)
		{
			/*
			 * TODO: check if all joints' torque are now back below their respective limits
			 * 		if torque are below limits, it is likely the manipulator hit a moving object (i.e. a person)
			 * 		restart control by the API
			 * 		if torque are still above the limits, it is likely the manipulator has hit an immobile object (table, wall, itself...)
			 * 		try to move the manipulator by small increment to get torque below the limits
			 */
			return 0;

		}
		else
		{
			ROS_WARN("No torque fault detected. No action required.");
			return 0;
		}
	}

	bool wmArm::computeBasePlan(const geometry_msgs::PoseStamped& target, moveit::planning_interface::MoveGroup::Plan& myPlan)
	{
		if (!baseMoveGroup_.setPoseTarget(target))
		{
			return false;
		}

		baseMoveGroup_.setStartStateToCurrentState();
		bool planSuccess = baseMoveGroup_.plan(myPlan);
		baseMoveGroup_.clearPoseTargets();
		return planSuccess;
	}

	bool wmArm::computeManipulatorPlan(const geometry_msgs::PoseStamped& target, moveit::planning_interface::MoveGroup::Plan& myPlan)
	{
		if (!manipulatorMoveGroup_.setPoseTarget(target))
		{
			return false;
		}

		manipulatorMoveGroup_.setStartStateToCurrentState();
		bool planSuccess = manipulatorMoveGroup_.plan(myPlan);
		manipulatorMoveGroup_.clearPoseTargets();
		return planSuccess;
	}

	bool wmArm::computeManipulatorPlan(const std::vector<double>& target, moveit::planning_interface::MoveGroup::Plan& myPlan)
	{
		if (!manipulatorMoveGroup_.setJointValueTarget(target))
		{
			return false;
		}

		manipulatorMoveGroup_.setStartStateToCurrentState();
		bool planSuccess = manipulatorMoveGroup_.plan(myPlan);
		manipulatorMoveGroup_.clearPoseTargets();
		return planSuccess;
	}
} // namespace manipulator

