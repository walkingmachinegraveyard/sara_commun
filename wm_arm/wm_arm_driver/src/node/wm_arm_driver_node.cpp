/*
 * wmArmDriver.cpp
 *
 *  Created on: Apr 10, 2016
 *      Author: xhache
 */

#include <boost/thread/recursive_mutex.hpp>

#include "ros/ros.h"
#include "urdf/model.h"
#include "wm_arm_driver/kinova_comm.h"
#include "wm_arm_driver/manipulator.h"
#include "wm_arm_driver/ik_solver.h"
#include "wm_arm_driver/wm_arm.h"
#include "wm_arm_driver/execute_plan_action_server.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "wm_arm_driver_node");
	ros::NodeHandle nh("~");

	boost::recursive_mutex apiMutex;
/*
	ros::AsyncSpinner spinner(0);
	spinner.start();
*/
	std::string urdfParam = "";
	std::string moveGroupName = "";

	nh.getParam("urdfParam", urdfParam);
	nh.getParam("move_group_name", moveGroupName);

	moveit::planning_interface::MoveGroup::Options opt(moveGroupName, urdfParam);

/*
	std::string modelFile = "";
	nh.getParam("modelFile", modelFile);

	manipulator::Manipulator manip(modelFile);

	if (!manip.isValid())
	{
		ROS_FATAL("Failed to create model from XML file.");
		return -1;
	}

	// allowable position and orientation errors for CCD solver
	double posError, orError;

	nh.param("epsilonPosition", posError, 0.01);		// default value is 0.01m
	nh.param("epsilonOrientation", orError, 0.1);
	// maximum number of iterations ofr CCD solver
	int iterMax;
	nh.param("iterMax", iterMax, 50);

	manipulator::ikSolver ikSolver(posError, orError, iterMax);
*/
	kinova::kinovaComm kComm(nh, apiMutex);

	if (!kComm.initSuccesfull())
	{
		ROS_FATAL("Failed to initialize communication with the controller.");
		return -1;
	}

//	std::vector<double> homePos = manip.getJointsHome();

	std::string actionName = "execute_plan";
	nh.param("action_server_name", actionName, actionName);

	manipulator::executePlanActionServer executePlanAS(nh, kComm, actionName);

	manipulator::wmArm wArm(nh, kComm, opt);

	while(ros::ok())
	{
		ros::spin();
	}

	return 0;
}
