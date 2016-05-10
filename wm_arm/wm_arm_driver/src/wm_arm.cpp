/*
 * wmArm.cpp
 *
 *  Created on: Apr 10, 2016
 *      Author: xhache
 */

#include "wm_arm_driver/wm_arm.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>

namespace manipulator
{
	wmArm::wmArm(ros::NodeHandle& nh, kinova::kinovaComm& kComm, const moveit::planning_interface::MoveGroup::Options& opt)
				:nh_(nh), kComm_(kComm), moveGroup_(opt)
	{
		torqueFault_ = false;

		// Init structures
		torqueInfo_.InitStruct();
		angularInfo_.InitStruct();
		velocityInfo_.InitStruct();

		double tmpTorque;
		nh_.param("maxTorque_joint1", tmpTorque, 8.0);
		maxTorqJoint1_ = (float)tmpTorque;
		nh_.param("maxTorque_joint2", tmpTorque, 8.0);
		maxTorqJoint2_ = (float)tmpTorque;
		nh_.param("maxTorque_joint3", tmpTorque, 7.0);
		maxTorqJoint3_ = (float)tmpTorque;
		nh_.param("maxTorque_joint4", tmpTorque, 6.0);
		maxTorqJoint4_ = (float)tmpTorque;
		nh_.param("maxTorque_joint5", tmpTorque, 4.0);
		maxTorqJoint5_ = (float)tmpTorque;
		nh_.param("maxTorque_joint1", tmpTorque, 4.0);
		maxTorqJoint6_ = (float)tmpTorque;

		// get robot information
		nbJoints_ = moveGroup_.getVariableCount();
		ROS_INFO("Move group has %d joints", nbJoints_);

		frameId_ = moveGroup_.getPlanningFrame();
		ROS_INFO("Planning frame is %s", frameId_.c_str());

		activeJoints_ = moveGroup_.getActiveJoints();

		for (unsigned int j = 0; j < activeJoints_.size(); j++)
		{
			ROS_INFO("Joint %d name is %s", j, activeJoints_.at(j).c_str());
		}

		// set services
		stopControlSrv_ = nh_.advertiseService("stop_control", &wmArm::stopControlService, this);
		startControlSrv_ = nh_.advertiseService("start_control", &wmArm::startControlService, this);
		homeArmSrv_ = nh_.advertiseService("home", &wmArm::homeArmService, this);
		recoverTorqueFaultSrv_ = nh_.advertiseService("recover_torque_fault", &wmArm::recoverTorqueFaultService, this);
		statusSrv_ = nh_.advertiseService("status", &wmArm::armStatusService, this);
		computePlanSrv_ = nh_.advertiseService("compute_plan", &wmArm::computePlanService, this);

		// set publishers
		double torqueRate;
		nh_.param("checkTorque_rate", torqueRate, 20.0);
		TorqueCbTimer_ = nh_.createTimer(ros::Duration(1.0/torqueRate), &wmArm::torqueCallback, this);

		double positionRate;
		nh_.param("checkPosition_rate", positionRate, 20.0);
		PositionCbTimer_ = nh_.createTimer(ros::Duration(1.0/positionRate), &wmArm::positionCallback, this);

		double velocityRate;
		nh_.param("checkVelocity_rate", velocityRate, 20.0);
		VelocityCbTimer_ = nh_.createTimer(ros::Duration(1.0/velocityRate), &wmArm::velocityCallback, this);

		jointStatePub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
	}

	wmArm::~wmArm()
	{

	}

	bool wmArm::stopControlService(wm_arm_msgs::stopArmControl::Request& req, wm_arm_msgs::stopArmControl::Response& res)
	{
		int status = kComm_.myStopControlAPI();

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
		int status = kComm_.myMoveHome(homePosition_);

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

	bool wmArm::computePlanService(wm_arm_msgs::computePlan::Request& req, wm_arm_msgs::computePlan::Response& res)
	{
		moveit::planning_interface::MoveGroup::Plan myPlan;
		bool success = computePlan(req.targetPose, myPlan);

		if (success)
		{
			res.planningResult = res.PLANNING_SUCCESS;
			res.startState = myPlan.start_state_;
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
			ROS_ERROR("Torque limit exceeded. Stopping the arm.");
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
			js.name = activeJoints_;

			std::vector<double> tmp;

			tmp.push_back((double)angularInfo_.Actuators.Actuator1);
			tmp.push_back((double)angularInfo_.Actuators.Actuator2);
			tmp.push_back((double)angularInfo_.Actuators.Actuator3);
			tmp.push_back((double)angularInfo_.Actuators.Actuator4);
			tmp.push_back((double)angularInfo_.Actuators.Actuator5);
			tmp.push_back((double)angularInfo_.Actuators.Actuator6);

			for (unsigned int j = 0; j < nbJoints_; j++)
			{
				js.position.push_back(tmp[j]);
			}

			jointStatePub_.publish(js);
		}
		else
		{
			ROS_WARN("Could not get joint position information.");
		}
		return;
	}

	void wmArm::velocityCallback(const ros::TimerEvent& e)
	{
		// TODO
		return;
	}

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

	bool wmArm::computePlan(const geometry_msgs::Pose& target, moveit::planning_interface::MoveGroup::Plan& myPlan)
	{
		moveGroup_.setPoseTarget(target);
		bool planSuccess = moveGroup_.plan(myPlan);
		moveGroup_.clearPoseTargets();

		return planSuccess;
	}

} // namespace manipulator

