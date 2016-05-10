/*
 * ik_solver.cpp
 *
 *  Created on: Mar 22, 2016
 *      Author: xhache
 */

#include <iostream>
#include "include/ikcpp/ik_solver.h"

namespace manipulator
{
	ikSolver::ikSolver(double epsilonPosition, double epsilonOrientation, unsigned int iterMax)
	:epsilonPosition(epsilonPosition), epsilonOrientation(epsilonOrientation), iterMax(iterMax)
	{

	}

	ikSolver::~ikSolver()
	{

	}

	std::vector<double> ikSolver::ccd(const Manipulator& manip, const Eigen::Matrix4d& targetPose)
	{
		// vector containing joints' current position
		std::vector<double> q;

		for (unsigned int i = 0; i < manip.nbJoints; i++)
		{
			q.push_back(manip.joints[i].currentPosition);
		}

		// rotation matrix of targetPose
		Eigen::Matrix3d targetRotationMatrix = targetPose.topLeftCorner<3,3>();

		// get all homogeneous transform matrices
		std::vector<Eigen::Matrix4d> frames = getFrames(manip, q);

		// compute forward kinematics, i.e. get en effector pose
		Eigen::Matrix4d effectorPose = fwdKine(frames);

		// compute position and orientation error
		std::vector<double> error = computeError(targetPose, effectorPose);

		// 3 element column vector representing joint axis (always z axis) in local frame
		Eigen::Vector3d zAxis(0.0, 0.0, 1.0);

		unsigned int nbIter = 1;

		bool oneMoreLoop = true;

		// currently evaluated joint pose
		Eigen::Matrix4d evalPose;

		// evalPose rotation matrix
		Eigen::Matrix3d evalRotationMatrix;

		// row vector, current joint position to end effector position
		Eigen::RowVector3d jointToEffector;

		// row vector, current joint position to target position
		Eigen::RowVector3d jointToTarget;

		Eigen::Vector3d localAxis;

		// position and orientation weighing factors
		// used when joint is of type "rotation"
		double wP = 1.0;
		double wO = 0.0;

		// coefficients used to compute joint position adjustment
		// used when joint is of type "rotation"
		double k1, k2, k3;
		Eigen::Vector3d tmpColK3;

		std::vector<double> firstDerivTest;
		double optTest1, optTest2, optRotation;
		optTest1 = optTest2 = optRotation = 0.0;

		do
		{
			// start "for" loop at the manipulator's wrist frame
			if (frames.back().determinant() == 0)
			{
				std::cout << "Determinant is 0." << std::endl;
			}
				evalPose = effectorPose * frames.back().inverse();

			for (unsigned int i = manip.nbJoints; i > 0; i--)
			{
				if (frames.at(i).determinant() == 0)
				{
					std::cout << "Determinant is 0." << std::endl;
				}
				evalPose = evalPose * frames.at(i).inverse();

				jointToEffector << effectorPose(0,3) - evalPose(0,3),
									effectorPose(1,3) - evalPose(1,3),
									effectorPose(2,3) - evalPose(2,3);

				jointToTarget << targetPose(0,3) - evalPose(0,3),
									targetPose(1,3) - evalPose(1,3),
									targetPose(2,3) - evalPose(2,3);

				evalRotationMatrix = evalPose.topLeftCorner<3,3>();

				localAxis = evalRotationMatrix * zAxis;

				// if joint is prismatic, i.e. movement is translational
				if (manip.joints.at(i-1).type.compare("translation") == 0)
				{
					double jTrans = (jointToTarget - jointToEffector) * localAxis;
					q.at(i-1) = checkJointLimits(manip.joints.at(i-1), jTrans, q.at(i-1));
				}

				else	// joint is rotational
				{
					// make jointTo... unit vectors
					jointToEffector.normalize();
					jointToTarget.normalize();

					wO = error.back() / (12.0 * 5.0);
					wP = 1.0 / (1.0 + wO);
					wO /= wO + 1.0;

					k1 = wP * (jointToTarget * localAxis) * (jointToEffector * localAxis);
					k2 = wP * jointToTarget.dot(jointToEffector) + 0.000001;		// add a small number to prevent division by zero
					tmpColK3 = wP * jointToEffector.cross(jointToTarget);

					for (unsigned int j = 0; j < 3; j++)
					{
						k1 += wO * targetRotationMatrix.col(j).dot(localAxis) * evalRotationMatrix.col(j).dot(localAxis);
						k2 += wO * targetRotationMatrix.col(j).dot(evalRotationMatrix.col(j));

						tmpColK3 += wO * evalRotationMatrix.col(j).cross(targetRotationMatrix.col(j));
					}
					k3 = localAxis.dot(tmpColK3);

					// clear vectors
					firstDerivTest.clear();

					// objective function is: MAX( k1*(1- cos(phi)) + k2*cos(phi) + k3*sin(phi) )
					// first derivative test to find optimal phi
					firstDerivTest.push_back(atan(k3 / (k2 - k1)));

					// since tan is periodic, check for other candidates at +-PI

					// if firstDerivTest.front() is a noisy 0.0
					// truncate to prevent selecting a false potential candidate at +-PI
					if (pow(firstDerivTest.front(), 2) < 0.000001)
					{
						firstDerivTest.front() = 0.0;
					}

					if (firstDerivTest.front() + M_PI < M_PI)
					{
						firstDerivTest.push_back(firstDerivTest.front() + M_PI);
					}

					if (firstDerivTest.front() - M_PI > -1.0 * M_PI)
					{
						firstDerivTest.push_back(firstDerivTest.front() - M_PI);
					}

					// check which value of the first derivative test maximizes objective function
					if (firstDerivTest.size() == 1)
					{
						optRotation = firstDerivTest.front();
					}
					else
					{
						optTest1 = k1*(1.0 - cos(firstDerivTest.front())) + k2*cos(firstDerivTest.front()) +
									k3*sin(firstDerivTest.front());

						optTest2 = k1*(1.0 - cos(firstDerivTest.at(1))) + k2*cos(firstDerivTest.at(1)) +
									k3*sin(firstDerivTest.at(1));

						if (optTest1 > optTest2)
						{
							optRotation = firstDerivTest.front();
						}
						else
						{
							optRotation = firstDerivTest.at(1);
						}
					}

					q.at(i-1) = checkJointLimits(manip.joints.at(i-1), optRotation, q.at(i-1));
				}

				// update frames, effector pose and error
				frames.clear();
				frames = getFrames(manip, q);
				effectorPose = fwdKine(frames);
				error.clear();
				error = computeError(targetPose, effectorPose);
				std::cout << "Position error: " << error.front() << std::endl;
				std::cout << "Orientation error: " << error.at(1) << std::endl;
			}

			if (error.front() < epsilonPosition && error.back() < epsilonOrientation)
			{
				oneMoreLoop = false;
				std::cout << "Target reached in " << nbIter << " iteration(s)." << std::endl;
				std::cout << "Position error at last iteration was: " << error.front() << std::endl;
				std::cout << "Orientation error at last iteration was: " << error.back() << std::endl;
				std::cout << "Joints' position will be updated." << std::endl;
			}

			if (nbIter > iterMax)
			{
				oneMoreLoop = false;
				std::cout << "Could not reach target pose in " << iterMax << " iterations." << std::endl;
				std::cout << "Position error at last iteration was: " << error.front() << std::endl;
				std::cout << "Orientation error at last iteration was: " << error.back() << std::endl;
				std::cout << "No change has been made to the joints' position." << std::endl;

				q.clear();
			}

			nbIter++;

		} while (oneMoreLoop);

		return q;
	}

	std::vector<Eigen::Matrix4d> ikSolver::getFrames(const Manipulator& manip, const std::vector<double>& q)
	{
		std::vector<Eigen::Matrix4d> frames;

		frames.push_back(manip.worldFrame);

		Eigen::Matrix4d dhFrame;

		for (unsigned int i = 0; i < manip.nbJoints; i++)
		{

			double theta = manip.joints.at(i).dhParam.theta;
			if (manip.joints.at(i).type.compare("rotation") == 0)
			{
				theta += q.at(i) - manip.joints.at(i).homePosition;
			}

			double d = manip.joints.at(i).dhParam.d;

			if (manip.joints.at(i).type.compare("translation") == 0)
			{
				d += q.at(i) - manip.joints.at(i).homePosition;
			}

			double alpha = manip.joints.at(i).dhParam.alpha;
			double t = manip.joints.at(i).dhParam.t;

			dhFrame << cos(theta), -1.0*sin(theta)*cos(t), sin(theta)*sin(t), alpha*cos(theta),
						sin(theta), cos(theta)*cos(t), -1.0*cos(theta)*sin(t), alpha*sin(theta),
						0.0, sin(t), cos(t), d,
						0.0, 0.0, 0.0, 1.0;

			frames.push_back(dhFrame);
		}

		frames.push_back(manip.effectorFrame);

		return frames;
	}

	Eigen::Matrix4d ikSolver::fwdKine(const std::vector<Eigen::Matrix4d>& frames)
	{
			Eigen::Matrix4d effectorPose = frames[0];

			for (unsigned int i = 1; i < frames.size(); i++)
			{
				effectorPose = effectorPose * frames.at(i);
			}

			return effectorPose;
	}

	std::vector<double> ikSolver::computeError(const Eigen::Matrix4d& targetPose, const Eigen::Matrix4d& effectorPose)
	{
		double positionError = (targetPose(0,3) - effectorPose(0,3)) * (targetPose(0,3) - effectorPose(0,3)) +
								(targetPose(1,3) - effectorPose(1,3)) * (targetPose(1,3) - effectorPose(1,3)) +
								(targetPose(2,3) - effectorPose(2,3)) * (targetPose(2,3) - effectorPose(2,3));

		Eigen::Matrix3d targetRotationMatrix = targetPose.topLeftCorner<3,3>();
		Eigen::Matrix3d effectorRotationMatrix = effectorPose.topLeftCorner<3,3>();

		double orientationError = 0.0;
		double tmp;

		for (int i = 0; i < 3; i++)
		{
			tmp = targetRotationMatrix.col(i).dot(effectorRotationMatrix.col(i)) - 1.0;
			tmp = pow(tmp, 2);
			orientationError += tmp;
		}

		double tmpList[] = {positionError, orientationError};
		std::vector<double> error(tmpList, tmpList+sizeof(tmpList)/sizeof(double));

		return error;
	}

	double ikSolver::checkJointLimits(const Joint& joint, const double& delta, const double& jointPos)
	{
		double newPos = jointPos + delta;

		// if newPos is outside the limits, check for an alternate position at +-2PI
		if (newPos < joint.lowerLimit)
		{
			if ((newPos + 2*M_PI) < joint.upperLimit)
			{
				return newPos + 2*M_PI;
			}
			else
			{
				return joint.lowerLimit;
			}
		}
		else if (newPos > joint.upperLimit)
		{
			if ((newPos - 2*M_PI) > joint.lowerLimit)
			{
				return newPos - 2*M_PI;
			}
			else
			{
				return joint.upperLimit;
			}
		}
		else
		{
			return newPos;
		}
	}
} // namespace manipulator




