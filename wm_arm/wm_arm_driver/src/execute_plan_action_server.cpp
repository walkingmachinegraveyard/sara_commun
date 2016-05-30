/*
 * execute_plan_action_server.cpp
 *
 *  Created on: May 2, 2016
 *      Author: xhache
 */

#include "wm_arm_driver/execute_plan_action_server.h"

namespace manipulator
{
	executePlanActionServer::executePlanActionServer(ros::NodeHandle& nh, kinova::kinovaComm& kComm, const std::string& actionName)
							:nh_(nh), kComm_(kComm), actionServer_(nh, actionName, false), actionName_(actionName)
	{
		nh_.param("wait_duration", waitDuration_, 0.5);
		nh_.param("maximum_skip", maxSkip_, 2);

		double feedbackRate;
		nh_.param("action_feedback_rate", feedbackRate, 5.0);
		feedbackTimer_ = nh_.createTimer(ros::Duration(1.0/feedbackRate), &executePlanActionServer::feedbackCallback, this);

		actionServer_.registerGoalCallback(boost::bind(&executePlanActionServer::goalCallback, this));
		actionServer_.registerPreemptCallback(boost::bind(&executePlanActionServer::preemptCallback, this));

		actionServer_.start();
	}

	executePlanActionServer::~executePlanActionServer()
	{

	}

	void executePlanActionServer::goalCallback()
	{
		moveit_msgs::RobotTrajectory goal = actionServer_.acceptNewGoal()->trajectory;

		ROS_INFO("Goal accepted");

		int res;
		int nbSkip = 0;
		trajectory_msgs::JointTrajectoryPoint jt;

		for (unsigned int i = 0; i < goal.joint_trajectory.points.size(); i++)
		{
			jt = goal.joint_trajectory.points.at(i);
			res = kComm_.mySendAngularPos(jt.positions, 0.0f);
			if (res != NO_ERROR_KINOVA)
			{
				ROS_WARN("Could not add trajectory point number %d to the controller FIFO.", i);
				ROS_WARN("Waiting %f seconds for some trajectory points to be processed...", waitDuration_);
				ros::Duration(waitDuration_).sleep();

				res = kComm_.mySendAngularPos(jt.positions, 0.0f);
				if (res != NO_ERROR_KINOVA)
				{
					ROS_WARN("Failed again to add trajectory point number %d to the controller FIFO. Skipping point...", i);
					nbSkip++;
					if (nbSkip > maxSkip_)
					{
						ROS_ERROR("Skipped more than %d trajectory points. Aborting current action goal.", maxSkip_);
						result_.fifoEmpty = false;
						actionServer_.setAborted(result_);
					}
				}
			}
		}
	}

	void executePlanActionServer::preemptCallback()
	{
		ROS_INFO("%s preempted.", actionName_.c_str());
		// stop the arm
		int res = kComm_.myStopControlAPI();
		res = kComm_.myEraseAllTrajectories();
		// reassert control over the arm
		res = kComm_.myStartControlAPI();
		actionServer_.setPreempted();
	}

	void executePlanActionServer::feedbackCallback(const ros::TimerEvent& e)
	{
		if (!actionServer_.isActive())
		{
			return;
		}

		int res = kComm_.myGetGlobalTrajectoryInfo();

		if (res != NO_ERROR_KINOVA)
		{
			ROS_WARN("Could not get FIFO count.");
		}

		else
		{
			if (kComm_.getFifoCount() == 0)
			{
				result_.fifoEmpty = true;
				actionServer_.setSucceeded(result_);
			}
			else
			{
				feedback_.fifoSize = kComm_.getFifoCount();
			}
		}
	}
} //namespace manipulator
