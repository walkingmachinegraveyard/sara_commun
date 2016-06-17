/*
 * wmArm.h
 *
 *  Created on: Apr 10, 2016
 *      Author: xhache
 */

#ifndef WMARM_H_
#define WMARM_H_

#include "wm_arm_driver/kinova_comm.h"
#include "Kinova/KinovaTypes.h"

#include "wm_arm_msgs/homeArm.h"
#include "wm_arm_msgs/recoverTorqFault.h"
#include "wm_arm_msgs/startArmControl.h"
#include "wm_arm_msgs/stopArmControl.h"
#include "wm_arm_msgs/armStatus.h"
#include "wm_arm_msgs/computePlan.h"

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "moveit_msgs/CollisionObject.h"

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

namespace manipulator
{
	class wmArm
	{
		public:
			wmArm(ros::NodeHandle&, kinova::kinovaComm&, const moveit::planning_interface::MoveGroup::Options&, const moveit::planning_interface::MoveGroup::Options&);
			~wmArm();
			bool stopControlService(wm_arm_msgs::stopArmControl::Request& req, wm_arm_msgs::stopArmControl::Response& res);
			bool startControlService(wm_arm_msgs::startArmControl::Request& req, wm_arm_msgs::startArmControl::Response& res);
			bool homeArmService(wm_arm_msgs::homeArm::Request& req, wm_arm_msgs::homeArm::Response& res);
			bool recoverTorqueFaultService(wm_arm_msgs::recoverTorqFault::Request& req, wm_arm_msgs::recoverTorqFault::Response& res);
			bool armStatusService(wm_arm_msgs::armStatus::Request& req, wm_arm_msgs::armStatus::Response& res);
			bool computeBasePlanService(wm_arm_msgs::computePlan::Request& req, wm_arm_msgs::computePlan::Response& res);
			bool computeManipulatorPlanService(wm_arm_msgs::computePlan::Request& req, wm_arm_msgs::computePlan::Response& res);

		private:
			void torqueCallback(const ros::TimerEvent&);
			void positionCallback(const ros::TimerEvent&);
//			void velocityCallback(const ros::TimerEvent&);
			int recoverTorqueFault();
			bool computeBasePlan(const geometry_msgs::PoseStamped&, moveit::planning_interface::MoveGroup::Plan&);
			bool computeManipulatorPlan(const geometry_msgs::PoseStamped&, moveit::planning_interface::MoveGroup::Plan&);
			bool computeManipulatorPlan(const std::vector<double>&, moveit::planning_interface::MoveGroup::Plan&);

			ros::NodeHandle nh_;
			kinova::kinovaComm kComm_;
			ros::Timer TorqueCbTimer_;
			ros::Timer PositionCbTimer_;
			ros::Timer VelocityCbTimer_;
			ros::ServiceServer stopControlSrv_;
			ros::ServiceServer startControlSrv_;
			ros::ServiceServer homeArmSrv_;
			ros::ServiceServer recoverTorqueFaultSrv_;
			ros::ServiceServer statusSrv_;
			ros::ServiceServer computeBasePlanSrv_, computeManipulatorPlanSrv_;
			ros::Publisher jointStatePub_;

			AngularPosition torqueInfo_;
			AngularPosition angularInfo_;
//			AngularPosition velocityInfo_;

			unsigned int baseGroupNbJoints_, manipulatorGroupNbJoints_;
			std::string frameId_;
			std::vector<std::string> baseGroupActiveJoints_, manipulatorGroupActiveJoints_;
			std::string baseEndEffectorName_, manipulatorEndEffectorName_;

			bool torqueFault_;
			float maxTorqJoint1_, maxTorqJoint2_, maxTorqJoint3_, maxTorqJoint4_, maxTorqJoint5_, maxTorqJoint6_;
			std::vector<float> jointsOffset_;

			moveit::planning_interface::MoveGroup baseMoveGroup_;
			moveit::planning_interface::MoveGroup manipulatorMoveGroup_;
		//	moveit::planning_interface::PlanningSceneInterface planningSceneInterface_;
	};
} //namespace manipulator

#endif /* WMARM_H_ */
