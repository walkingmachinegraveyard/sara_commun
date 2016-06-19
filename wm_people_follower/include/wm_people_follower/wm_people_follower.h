/*
 * wm_people_follower.h
 *
 *  Created on: Jun 5, 2016
 *      Author: xhache
 */

#ifndef WM_PEOPLE_FOLLOWER_H_
#define WM_PEOPLE_FOLLOWER_H_

#include <math.h>

#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "people_msgs/PositionMeasurementArray.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "wm_people_follower/peopleFollower.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <boost/thread/lock_guard.hpp>

namespace wm
{
	class peopleFollower
	{
		public:
			peopleFollower(const ros::NodeHandle&);
			~peopleFollower();
			bool peopleFollowerService(wm_people_follower::peopleFollower::Request&, wm_people_follower::peopleFollower::Response&);

		private:
			void legTrackerCallback(const people_msgs::PositionMeasurementArray&);
			ros::NodeHandle nh_;
			geometry_msgs::Point personNewPosition_, personPreviousPosition_;
			actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBaseAC_;
			ros::Subscriber legTrackerSub_;
			ros::ServiceServer followSrvServer_;
			ros::Publisher testPub_;
			boost::mutex mtx_;
			ros::Timer lockTargetTimer_;
			bool isFollowing_, lockTarget_, targetLocked_;
			double followDistance_, maxLockDistance_, searchDistance_, searchRadius_;
			double minMeasurementReliability_;
			double lastActionTime_, actionClientPeriod_;
			int lockingTimeOut_;
			tf2_ros::Buffer tfBuffer_;
			tf2_ros::TransformListener tfListener_;
			geometry_msgs::TransformStamped tfStamped_;
	};
} // namespace wm
#endif /* WM_PEOPLE_FOLLOWER_H_ */

