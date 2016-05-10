/*
 * execute_plan_action_server.h
 *
 *  Created on: May 2, 2016
 *      Author: xhache
 */

#ifndef EXECUTE_PLAN_ACTION_SERVER_H_
#define EXECUTE_PLAN_ACTION_SERVER_H_

#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "wm_arm_msgs/executePlanAction.h"
#include "wm_arm_driver/kinova_comm.h"

namespace manipulator
{
	class executePlanActionServer
	{
		public:
			executePlanActionServer(ros::NodeHandle&, kinova::kinovaComm&, const std::string&);
			~executePlanActionServer();

		private:
			void goalCallback();
			void preemptCallback();
			void feedbackCallback(const ros::TimerEvent&);
			wm_arm_msgs::executePlanFeedback feedback_;
			wm_arm_msgs::executePlanResult result_;
			ros::NodeHandle nh_;
			kinova::kinovaComm kComm_;
			actionlib::SimpleActionServer<wm_arm_msgs::executePlanAction> actionServer_;
			ros::Timer feedbackTimer_;
			double waitDuration_;
			std::string actionName_;
			int maxSkip_;
	};
} // namespace manipulator
#endif /* EXECUTE_PLAN_ACTION_SERVER_H_ */
