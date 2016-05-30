/*
 * test.cpp
 *
 *  Created on: May 1, 2016
 *      Author: xhache
 */

#include "ros/ros.h"
#include "wm_arm_msgs/computePlan.h"
#include "moveit_msgs/DisplayTrajectory.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "wm_arm_msgs/executePlanAction.h"

#include "iostream"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "plan_test");

	ros::AsyncSpinner spinner(0);
	spinner.start();

	ros::NodeHandle nh;
/*
	if (argc != 8)
	{
		ROS_ERROR("Invalid number of arguments");
		return -1;
	}
*/

	ros::ServiceClient client = nh.serviceClient<wm_arm_msgs::computePlan>("/wm_arm_driver_node/compute_manipulator_plan");
	wm_arm_msgs::computePlan srv;

	bool timeout = ros::service::exists("/wm_arm_driver_node/compute_manipulator_plan", false);
	if (!timeout)
	{
		ROS_FATAL("Service does not exist.");
		return -1;
	}
	else
	{
		ROS_INFO("Service is available.");

		srv.request.targetPose.pose.position.x = 0.5;
		srv.request.targetPose.pose.position.y = 0.0;
		srv.request.targetPose.pose.position.z = 0.0;
		srv.request.targetPose.pose.orientation.x = 0.0;
		srv.request.targetPose.pose.orientation.y = 0.0;
		srv.request.targetPose.pose.orientation.z = 0.0;
		srv.request.targetPose.pose.orientation.w = 1.0;

		srv.request.targetPose.header.frame_id = "odom";
		srv.request.targetPose.header.stamp = ros::Time::now();

		if (client.call(srv))
		{
			ROS_INFO("Request sent.");
			if (srv.response.planningResult == srv.response.PLANNING_SUCCESS)
			{
				ROS_INFO("Planning succeeded.");
				ROS_INFO("Trajectory contains %u points", srv.response.trajectory.joint_trajectory.points.size());

				ROS_INFO("*** Trajectory ***");
				for (unsigned int i=0; i < srv.response.trajectory.joint_trajectory.points.size(); i++)
				{
					std::cout << srv.response.trajectory.joint_trajectory.points.at(i) << std::endl;
				}
/*				ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
				moveit_msgs::DisplayTrajectory display_trajectory;
				display_trajectory.trajectory_start = srv.response.startState;
			    display_trajectory.trajectory.push_back(srv.response.trajectory);
			    display_publisher.publish(display_trajectory);
			    sleep(5.0);
			    return 0;*/
			}
			else
			{
				ROS_INFO("Planning failed.");
			}
		}
		else
		{
			ROS_ERROR("Service call failed.");
			return -1;
		}
	}
/*
	actionlib::SimpleActionClient<wm_arm_msgs::executePlanAction> ac("/wm_arm_driver_node/execute_plan", false);

	ac.waitForServer(ros::Duration(10.0));
	bool conn = ac.isServerConnected();

	if(conn)
	{
		ROS_INFO("Connected to the server!");
	}
	else
	{
		ROS_ERROR("Not connected to the server!");
		return -1;
	}

	ROS_INFO("Sending goal! SHITS ARE GOING TO MOVE!!");

	wm_arm_msgs::executePlanGoal goal;
	goal.trajectory = srv.response.trajectory;

	ac.sendGoal(goal);

	ac.waitForResult();

	ROS_INFO("Goal reached!");
*/
	return 0;
}

