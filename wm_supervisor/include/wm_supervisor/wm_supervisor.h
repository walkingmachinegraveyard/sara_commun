/*
 * wm_supervisor.h
 *
 *  Created on: May 11, 2016
 *      Author: xhache
 */

#ifndef WM_SUPERVISOR_H_
#define WM_SUPERVISOR_H_

#include <string>

#include "ros/ros.h"
#include "wm_supervisor/robotStatus.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "wm_arm_msgs/executePlanAction.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"

namespace wm{

	const std::string STOP_STR = "stop";
	const int STATUS_OK = 1;
	const int STOP_COMMANDED = -1;

	class wmSupervisor
	{
		public:
			wmSupervisor(const ros::NodeHandle&, const std::string&, const std::string&);
			~wmSupervisor();
			bool robotStatusService(wm_supervisor::robotStatus::Request&, wm_supervisor::robotStatus::Response&);
			bool recoverFromStopService(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

		private:
			void acConnectCallback(const ros::TimerEvent&);
			void audioSubscriberCallback(const std_msgs::String&);
			void estopSubscriberCallback(const std_msgs::String&); //TODO
			void watchdogCallback(const ros::TimerEvent&);
			void safeVelocityCallback(const geometry_msgs::Twist&);
			ros::NodeHandle nh_;
			ros::ServiceServer robotStatusSrv_;
			ros::ServiceServer recoverFromStopSrv_;
			ros::Subscriber audioStreamSub_, estopSignalSub_, safeVelocitySub_;
			ros::Publisher safeVelocityPub_;
			actionlib::SimpleActionClient<wm_arm_msgs::executePlanAction> moveArmAC_;
			actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBaseAC_;
			ros::Timer callbackTimer_;
			ros::Timer watchdogTimer_;
			double watchdogRate_;
			ros::Time lastCallback_;
			int status_;
	};
} //namespace wm
#endif /* WM_SUPERVISOR_H_ */
