/*
 * wm_moveit.cpp
 *
 *  Created on: Apr 18, 2016
 *      Author: xhache
 */

#include "wm_arm_driver/wm_moveit.h"

namespace manipulator
{
	WM_MoveIt::WM_MoveIt(const moveit::planning_interface::MoveGroup::Options& opt)
				:moveGroup_(opt)
	{

	}

	WM_MoveIt::~WM_MoveIt()
	{

	}

	void WM_MoveIt::getTorqueLimits(std::vector<float>& torqueList)
	{


	}

} //namespace manipulator


