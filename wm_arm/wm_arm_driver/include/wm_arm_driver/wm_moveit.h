/*
 * wm_moveit.h
 *
 *  Created on: Apr 18, 2016
 *      Author: xhache
 */

#ifndef WM_MOVEIT_H_
#define WM_MOVEIT_H_

#include "ros/ros.h"
#include "moveit/move_group_interface/move_group.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"


namespace manipulator
{
	class WM_MoveIt
	{
		public:
			WM_MoveIt(const moveit::planning_interface::MoveGroup::Options&);
			~WM_MoveIt();
			 void getTorqueLimits(std::vector<float>&);

		private:
			moveit::planning_interface::MoveGroup moveGroup_;
			moveit::planning_interface::PlanningSceneInterface planScene_;
			moveit::planning_interface::MoveGroup::Plan myPlan_;
	};
} // namespace manipulator
#endif /* WM_MOVEIT_H_ */
