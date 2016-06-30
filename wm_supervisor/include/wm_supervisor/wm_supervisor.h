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
#include "roboteq_msgs/Command.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"
#include "geometry_msgs/Twist.h"
#include <boost/thread/lock_guard.hpp>

namespace wm{

	const std::string STOP_STR = "stop";
	const int STATUS_OK = 1;
	const int STOP_COMMANDED = -1;
	const int STAND_BY = 2;

	class wmSupervisor
	{
		public:
			wmSupervisor(const ros::NodeHandle&, const std::string&, const std::string&);
			~wmSupervisor();
			bool robotStatusService(wm_supervisor::robotStatus::Request&, wm_supervisor::robotStatus::Response&);
			bool stopSignalService(std_srvs::SetBool::Request&, std_srvs::SetBool::Response&);

		private:
			void audioSubscriberCallback(const std_msgs::String&);
			void startSignalCallback(const std_msgs::Bool&);
			void safeVelocityCallback(const geometry_msgs::Twist&);
			ros::NodeHandle nh_;
			ros::ServiceServer robotStatusSrv_;
			ros::ServiceServer stopSignalSrv_;
			ros::Subscriber audioStreamSub_, startSignalSub_, safeVelocitySub_;
			ros::Publisher safeVelocityPub_, FLWdrivePub_, FRWdrivePub_, RLWdrivePub_, RRWdrivePub_;
			actionlib::SimpleActionClient<wm_arm_msgs::executePlanAction> moveArmAC_;
			actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBaseAC_;
			int status_;

			boost::mutex mtx_;
	};
} //namespace wm
#endif /* WM_SUPERVISOR_H_ */
