/*
 * ik_solver.h
 *
 *  Created on: Mar 22, 2016
 *      Author: xhache
 */

#ifndef IK_SOLVER_H_
#define IK_SOLVER_H_

#include <math.h>
#include <vector>

#include "Eigen/Dense"
#include "manipulator.h"

namespace manipulator
{
	class ikSolver
	{
		public:
			ikSolver(double epsilonPosition, double epsilonOrientation, unsigned int iterMax);
			~ikSolver();
			std::vector<double> ccd(const Manipulator&, const Eigen::Matrix4d&);

		private:
			std::vector<Eigen::Matrix4d> getFrames(const Manipulator&, const std::vector<double>&);
			Eigen::Matrix4d fwdKine(const std::vector<Eigen::Matrix4d>&);
			std::vector<double> computeError(const Eigen::Matrix4d&, const Eigen::Matrix4d&);
			double checkJointLimits(const Joint& joint, const double& delta, const double& jointPos);

			double epsilonPosition;
			double epsilonOrientation;
			unsigned int iterMax;
	};

} // namespace manipulator
#endif /* IK_SOLVER_H_ */
