/*
 * wm_supervisor.cpp
 *
 *  Created on: May 11, 2016
 *      Author: xhache
 */

#include "wm_supervisor/wm_supervisor.h"

namespace wm
{
	wmSupervisor::wmSupervisor(const ros::NodeHandle& nh, const std::string& armActionName, const std::string& baseActionName)
								:nh_(nh), moveArmAC_(armActionName, false), moveBaseAC_(baseActionName, false)
	{
		status_ = wm::STATUS_OK;

		safeVelocityPub_ = nh_.advertise<geometry_msgs::Twist>("safe_cmd_vel", 1);
		FLWdrivePub_ = nh_.advertise<roboteq_msgs::Command>("/roboteq_driver_FLW/cmd", 1);
		FRWdrivePub_ = nh_.advertise<roboteq_msgs::Command>("/roboteq_driver_FRW/cmd", 1);
		RLWdrivePub_ = nh_.advertise<roboteq_msgs::Command>("/roboteq_driver_RLW/cmd", 1);
		RRWdrivePub_ = nh_.advertise<roboteq_msgs::Command>("/roboteq_driver_RRW/cmd", 1);

		audioStreamSub_ = nh_.subscribe("output", 10, &wmSupervisor::audioSubscriberCallback, this);
		safeVelocitySub_ = nh_.subscribe("cmd_vel", 10, &wmSupervisor::safeVelocityCallback, this);
		startSignalSub_ = nh_.subscribe("start_button_msg", 1, &wmSupervisor::startSignalCallback, this);

		robotStatusSrv_ = nh_.advertiseService("robot_status", &wmSupervisor::robotStatusService, this);
		stopSignalSrv_ = nh_.advertiseService("safety_stop_srv", &wmSupervisor::stopSignalService, this);
	}

	wmSupervisor::~wmSupervisor()
	{

	}

	bool wmSupervisor::robotStatusService(wm_supervisor::robotStatus::Request& req, wm_supervisor::robotStatus::Response& res)
	{
		boost::lock_guard<boost::mutex> guard(mtx_);

		if (status_ == wm::STATUS_OK)
		{
			res.status = res.STATUS_OK;
		}
		else if (status_ == wm::STOP_COMMANDED)
		{
			res.status = res.STOP_COMMANDED;
		}

		return true;
	}

	bool wmSupervisor::stopSignalService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
	{
		boost::lock_guard<boost::mutex> guard(mtx_);

		if (!req.data)
		{
			ROS_WARN("Received stop signal!!!");
			status_ = wm::STOP_COMMANDED;
			if (moveBaseAC_.isServerConnected())
			{
				moveBaseAC_.cancelAllGoals();
			}

			if (moveArmAC_.isServerConnected())
			{
				moveArmAC_.cancelAllGoals();
			}

			res.success = true;
		}
		else if (status_ == wm::STOP_COMMANDED)
		{
			ROS_INFO("Safety stop disengaged.");
			status_ = wm::STAND_BY;
			res.success = true;
		}

		return true;
	}

	void wmSupervisor::audioSubscriberCallback(const std_msgs::String& msg)
	{
		if(msg.data.find(STOP_STR) != std::string::npos)
		{
			ROS_WARN("Heard \"STOP\"!!! Preempting all goals!");

			if(moveArmAC_.isServerConnected())
			{
				moveArmAC_.cancelAllGoals();
			}

			if(moveBaseAC_.isServerConnected())
			{
				moveBaseAC_.cancelAllGoals();
			}

			boost::lock_guard<boost::mutex> guard(mtx_);

			status_ = wm::STOP_COMMANDED;
		}

		return;
	}

	void wmSupervisor::safeVelocityCallback(const geometry_msgs::Twist& msg)
	{
		boost::lock_guard<boost::mutex> guard(mtx_);

		if (status_ == wm::STATUS_OK)
		{
			safeVelocityPub_.publish(msg);
		}

		else
		{
			roboteq_msgs::Command cmd;
			cmd.mode = cmd.MODE_STOPPED;
			cmd.setpoint = 0.0;

			FLWdrivePub_.publish(cmd);
			FRWdrivePub_.publish(cmd);
			RLWdrivePub_.publish(cmd);
			RRWdrivePub_.publish(cmd);
		}
		return;
	}

	void wmSupervisor::startSignalCallback(const std_msgs::Bool& msg)
	{
		boost::lock_guard<boost::mutex> guard(mtx_);
		if (msg.data)
		{
			if (status_ == wm::STAND_BY)
			{
				ROS_INFO("Received start signal.");
				status_ = wm::STATUS_OK;
			}
		}

		return;
	}
} //namespace wm

int main(int argc, char **argv)
{
	ros::init(argc, argv, "wm_supervisor_node");
	ros::NodeHandle nh;

	std::string moveArmActionName = "/wm_arm_driver_node/execute_plan";
	nh.param("/wm_supervisor_node/move_arm_action_name", moveArmActionName, moveArmActionName);

	std::string moveBaseActionName = "/move_base";
	nh.param("/wm_supervisor_node/move_base_action_name", moveBaseActionName, moveBaseActionName);

	wm::wmSupervisor supervisor(nh, moveArmActionName, moveBaseActionName);
	
	while (ros::ok())
	{
		ros::spin();
	}

	return 0;
}
