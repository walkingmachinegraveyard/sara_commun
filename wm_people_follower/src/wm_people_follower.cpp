/*
 * wm_people_follower.cpp
 *
 *  Created on: Jun 5, 2016
 *      Author: xhache
 */

#include "wm_people_follower/wm_people_follower.h"

namespace wm
{
	peopleFollower::peopleFollower(const ros::NodeHandle& nh)
								:nh_(nh), moveBaseAC_("move_base", false), tfListener_(tfBuffer_)
	{
		isFollowing_ = false;
		lockTarget_ = false;
		targetLocked_ = false;

		lastActionTime_ = ros::Time::now().toSec();

		nh_.param("wm_people_follower_node/action_client_period", actionClientPeriod_, 4.0);
		nh_.param("wm_people_follower_node/follow_distance", followDistance_, 1.0);
		nh_.param("wm_people_follower_node/maximum_locking_distance", maxLockDistance_, 1.0);
		nh_.param("wm_people_follower_node/locking__time_out", lockingTimeOut_, 10);
		nh_.param("wm_people_follower_node/search_distance", searchDistance_, 0.2);
		nh_.param("wm_people_follower_node/minimum_measurement_reliability", minMeasurementReliability_, 0.0);
		testPub_ = nh_.advertise<geometry_msgs::PoseStamped>("people_follower_test", 1);
		followSrvServer_ = nh_.advertiseService("wm_people_follow", &peopleFollower::peopleFollowerService, this);
		legTrackerSub_ = nh_.subscribe("/people_tracker_measurements", 10, &peopleFollower::legTrackerCallback, this);

		searchRadius_ = searchDistance_;
	}

	peopleFollower::~peopleFollower()
	{

	}

	bool peopleFollower::peopleFollowerService(wm_people_follower::peopleFollower::Request& req, wm_people_follower::peopleFollower::Response& res)
	{
		mtx_.lock();

		if (req.request == req.ACQUIRE_TARGET)
		{
			try
			{
				tfStamped_ = tfBuffer_.lookupTransform("odom", "base_link", ros::Time(0));
			}
			catch (tf2::TransformException &e)
			{
				res.response = res.FAILURE;
				return true;
			}
			
			lockTarget_ = true;
			ROS_INFO("Received request to acquire target to follow.");
			mtx_.unlock();

			ros::Duration(lockingTimeOut_).sleep();

			mtx_.lock();
			if (targetLocked_)
			{
				res.response = res.SUCCESS;
				ROS_INFO("Target acquisition succeeded.");
			}
			else
			{
				res.response = res.FAILURE;
				ROS_WARN("Target acquisition failed.");
			}
			mtx_.unlock();
		}
		else if (req.request == req.START_FOLLOWING)
		{
			if (targetLocked_ )//&& moveBaseAC_.isServerConnected()) TODO
			{
				lockTarget_ = false;
				isFollowing_ = true;
				res.response = res.SUCCESS;
				ROS_INFO("Ready to follow.");
			}
			else
			{
				ROS_WARN("Unable to follow: no target acquired or move_base client is not connected.");
				res.response = res.FAILURE;
			}
		}
		else if (req.request == req.STOP_FOLLOWING)
		{
			isFollowing_ = false;
			res.response = res.SUCCESS;
			ROS_INFO("Following stopped.");
		}
		else if (req.request == req.RELEASE_TARGET)
		{
			isFollowing_ = false;
			personNewPosition_ = geometry_msgs::Point();
			personPreviousPosition_ = geometry_msgs::Point();
			res.response = res.SUCCESS;
			ROS_INFO("Released target.");
		}

		mtx_.unlock();

		return true;
	}

	void peopleFollower::legTrackerCallback(const people_msgs::PositionMeasurementArray& inputMsg)
	{
		boost::lock_guard<boost::mutex> guard(mtx_);

		if (lockTarget_)
		{
			if (inputMsg.people.size() > 0)
			{
				// targetLocked_ becomes true after first successful pass in this loop
				if (targetLocked_)
				{
					personPreviousPosition_ = personNewPosition_;
				}
				// assume the closest person in front of the robot is the person to follow
				unsigned int closestPerson = 0;

				// the higher the score,the higher the probability that a person is closest and in front of the robot
				// score = 1.0 / (||(x, y)|| * abs(atan2(y, x)) +0.0001) where x and y represent a person's position
				// the way the score is computed, it favors locking on a person who is standing in front of robot over
				// a person standing nearer the robot but more on the side

				double bestScore = 0.0;
				double newScore = 0.0;

				double distance, angle;

				// get the closest person's position
				for (unsigned int i = 0; i < inputMsg.people.size(); i++)
				{
					distance = sqrt(pow(inputMsg.people.at(i).pos.x - tfStamped_.transform.translation.x, 2) +
									pow(inputMsg.people.at(i).pos.y - tfStamped_.transform.translation.y, 2));
					angle = abs(atan2(inputMsg.people.at(i).pos.y - tfStamped_.transform.translation.y,
										inputMsg.people.at(i).pos.x - tfStamped_.transform.translation.x));

					newScore = 1.0 / (distance * angle + 0.0001);

					if (newScore > bestScore && distance < maxLockDistance_)
					{
						closestPerson = i;
						bestScore = newScore;
					}
				}

				if (bestScore > 0.0)
				{
					personNewPosition_.x = inputMsg.people.at(closestPerson).pos.x;
					personNewPosition_.y = inputMsg.people.at(closestPerson).pos.y;
					personNewPosition_.z = 0.0;

					targetLocked_ = true;
					lockTarget_ = false;
				}
			}
		}

		else if (targetLocked_)
		{
			if (inputMsg.people.size() == 0)
			{
				// Do not update the position, the robot will stop after reaching the last known position
				ROS_WARN("Leg tracker did not find any person.");
			}
			else
			{
				double knownHeadingX = personNewPosition_.x - personPreviousPosition_.x;
				double knownHeadingY = personNewPosition_.y - personPreviousPosition_.y;

				geometry_msgs::Point projectedPosition;
				// estimate the tracked person's position given their last two positions
				projectedPosition.x = personNewPosition_.x + knownHeadingX;
				projectedPosition.y = personNewPosition_.y + knownHeadingY;

				// distance = ||(projected_x - evaluated_x), (projected_y -  evaluated_y)||
				// heading = (evaluated_* - last_known_*, evaluated_* - last_known_*)
				double distance, distanceX, distanceY, cosTheta, headingX, headingY;

				// score = (1.0/distance) * (cos(theta) + 1) where theta is the angle between knownHeading and newHeading
				// the higher the score, the more probable the currently evaluated measurement is the new position
				// of the person the robot is supposed to follow
				double bestScore = 0.0;
				double newScore = 0.0;

				unsigned int followedPerson;
				bool emptyMsg = true;

				for (unsigned int i=0; i < inputMsg.people.size(); i++)
				{
					distanceX = pow(inputMsg.people.at(i).pos.x - projectedPosition.x, 2);
					distanceY = pow(inputMsg.people.at(i).pos.y - projectedPosition.y, 2);
					distance = sqrt(distanceX + distanceY);

					headingX = inputMsg.people.at(i).pos.x - personNewPosition_.x;
					headingY = inputMsg.people.at(i).pos.y - personNewPosition_.y;

					if (distance < searchRadius_ && inputMsg.people.at(i).reliability > minMeasurementReliability_)
					{
						if (distanceX != 0.0 && distanceY != 0.0)
						{
							cosTheta = (((inputMsg.people.at(i).pos.x) * personNewPosition_.x +
										inputMsg.people.at(i).pos.y * personNewPosition_.y) /
										(sqrt(distanceX) * sqrt(distanceY)));
						}
						else if (distanceX == 0.0)
						{
							cosTheta = ((inputMsg.people.at(i).pos.y * personNewPosition_.y) / sqrt(distanceY));
						}
						else if (distanceY == 0.0)
						{
							cosTheta = ((inputMsg.people.at(i).pos.x * personNewPosition_.x) / sqrt(distanceX));
						}
						else
						{
							cosTheta = 1;
						}

						newScore = (1.0 / distance) * (cosTheta + 1.0);

						if (newScore > bestScore)
						{
							emptyMsg = false;
							followedPerson = i;
							bestScore = newScore;
						}
					}
				}

				personPreviousPosition_ = personNewPosition_;
				if (!emptyMsg)
				{
					personNewPosition_.x = inputMsg.people.at(followedPerson).pos.x;
					personNewPosition_.y = inputMsg.people.at(followedPerson).pos.y;
					searchRadius_ = searchDistance_;
				}
				else
				{
					searchRadius_ += 0.0;
				}
			}

			if (isFollowing_ && ((ros::Time::now().toSec() - lastActionTime_) > actionClientPeriod_))
			{
				lastActionTime_ = ros::Time::now().toSec();

				try
				{
					tfStamped_ = tfBuffer_.lookupTransform("odom", "base_link", ros::Time(0));
					geometry_msgs::PoseStamped outputMsg;

					outputMsg.header.frame_id = "odom";
					outputMsg.header.stamp = ros::Time::now();

					double distance = sqrt(pow(personNewPosition_.x - tfStamped_.transform.translation.x, 2)
										+ pow(personNewPosition_.y - tfStamped_.transform.translation.y, 2));

					tf2::Quaternion q;
					double yaw = atan2(personNewPosition_.y - tfStamped_.transform.translation.y,
										personNewPosition_.x - tfStamped_.transform.translation.x);
					q.setEuler(0.0, 0.0, yaw);

					outputMsg.pose.orientation.x = q[0];
					outputMsg.pose.orientation.y = q[1];
					outputMsg.pose.orientation.z = q[2];
					outputMsg.pose.orientation.w = q[3];

					// if the followed person is closer than followDistance_, the robot can only rotate
					if (distance < followDistance_)
					{
						outputMsg.pose.position.x = tfStamped_.transform.translation.x;
						outputMsg.pose.position.y = tfStamped_.transform.translation.y;
					}
					else
					{
						outputMsg.pose.position.x = (distance - followDistance_) * cos(yaw);
						outputMsg.pose.position.y = (distance - followDistance_) * sin(yaw);
					}

					outputMsg.pose.orientation.x = q[0];
					outputMsg.pose.orientation.y = q[1];
					outputMsg.pose.orientation.z = q[2];
					outputMsg.pose.orientation.w = q[3];

					testPub_.publish(outputMsg);

					if (moveBaseAC_.isServerConnected())
					{
						move_base_msgs::MoveBaseGoal goal;

						goal.target_pose = outputMsg;
						moveBaseAC_.sendGoal(goal);
					}
					else
					{
						ROS_ERROR("Action client is not connected to action server.");
					}
				}
				catch (tf2::TransformException &e)
				{
					ROS_WARN("Lookup transform exception: %s", e.what());
				}
			}
		}

		return;
	}
} // namespace wm

int main(int argc, char **argv)
{
	ros::init(argc, argv, "wm_people_follower_node");
	ros::NodeHandle nh;

	wm::peopleFollower peopleFollower(nh);

	while (ros::ok())
	{
		ros::spin();
	}

	return 0;
}

