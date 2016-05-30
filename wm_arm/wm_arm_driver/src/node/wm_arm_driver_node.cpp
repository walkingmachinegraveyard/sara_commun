/*
 * wmArmDriver.cpp
 *
 *  Created on: Apr 10, 2016
 *      Author: xhache
 */

#include <boost/thread/recursive_mutex.hpp>

#include "ros/ros.h"
#include "wm_arm_driver/kinova_comm.h"
#include "wm_arm_driver/wm_arm.h"
#include "wm_arm_driver/execute_plan_action_server.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "wm_arm_driver_node");
	ros::NodeHandle nh("~");

	boost::recursive_mutex apiMutex;

	ros::AsyncSpinner spinner(0);
	spinner.start();

	std::string urdfParam = "sara";
	std::string baseMoveGroupName = "sara_base";
	std::string manipulatorMoveGroupName = "sara_manipulator";

	nh.param("/wm_arm_driver_node/urdf_param", urdfParam, urdfParam);
	nh.param("/wm_arm_driver_node/base_move_group_name", baseMoveGroupName, baseMoveGroupName);
	nh.param("/wm_arm_driver_node/manipulator_move_group_name", manipulatorMoveGroupName, manipulatorMoveGroupName);

	moveit::planning_interface::MoveGroup::Options baseOpt(baseMoveGroupName);
	moveit::planning_interface::MoveGroup::Options manipulatorOpt(manipulatorMoveGroupName);

	kinova::kinovaComm kComm(nh, apiMutex);

	if (!kComm.initSuccesfull())
	{
		ROS_FATAL("Failed to initialize communication with the controller.");
		return -1;
	}

	std::string actionName = "execute_plan";
	nh.param("/wm_arm_driver_node/action_server_name", actionName, actionName);

	manipulator::executePlanActionServer executePlanAS(nh, kComm, actionName);

	manipulator::wmArm wArm(nh, kComm, baseOpt, manipulatorOpt);

	while(ros::ok())
	{
//		ros::spin();
	}

	return 0;
}
