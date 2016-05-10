/*
 * main.cpp
 *
 *  Created on: Apr 1, 2016
 *      Author: xhache
 */

#include "include/ikcpp/manipulator.h"
#include "include/ikcpp/ik_solver.h"

#include <iostream>

int main()
{

	manipulator::Manipulator manip("/home/xhache/Eclipse_ws/ikcpp/src/sara5.xml");

	if(!manip.isValid())
	{
		return -1;
	}

	manipulator::ikSolver solver(0.0001, 0.004, 50);

	Eigen::Matrix4d target;

	manipulator::goPose("front", target);

	std::vector<double> q = solver.ccd(manip, target);

	return 0;
}


