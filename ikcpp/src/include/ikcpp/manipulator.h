/*
 * ccd.h
 *
 *  Created on: Mar 19, 2016
 *      Author: xhache
 */

#ifndef MANIPULATOR_H_
#define MANIPULATOR_H_

#include <iostream>
#include <vector>
#include <math.h>

#include "Eigen/Dense"
#include "Pugi/pugixml.hpp"


#define NO_ERROR		1
#define PARSE_ERROR		10

// Macros for angle unit conversion
#define ToRadians(Degrees) (Degrees * M_PI / 180.0)
#define ToDegrees(Radians) (Radians * 180.0 / M_PI)

namespace manipulator
{
	struct DH_Param{
		double theta;
		double d;
		double alpha;
		double t;
	};

	class Joint
	{
		friend class Manipulator;
		friend class ikSolver;
		public:
			Joint(std::string jointType, double theta, double d, double alpha, double t,
					double lowerLimit, double upperLimit, double homePosition);
			~Joint();

		private:
			void setHomePosition(double pos);
			void setCurrentPosition(double pos);

			// Joint type: "rotation" or "translation"
			std::string type;

			// Denavit-Hartenberg parameters
			DH_Param dhParam;

			// Joint limits
			double lowerLimit, upperLimit;

			// Useful
			double homePosition;
			double previousPosition;
			double currentPosition;
	};

	class Manipulator
	{
		friend class ikSolver;
		public:
			Manipulator(char* fileName);
			~Manipulator();
			bool isValid(void);

		private:
			int parseXML(const char* fileName);
			int parseTransform(pugi::xml_node& node, Eigen::Matrix4d& matrix);

			std::string angleUnit;			// "radian" or "degree"
			unsigned int nbJoints;			// Number of joints
			std::vector<Joint> joints;		// Container for Joint class instances
			bool validModel;

			Eigen::Matrix4d worldFrame; 	// Joint 1 in world frame
			Eigen::Matrix4d effectorFrame; 	// Effector in wrist frame
	};

	void goPose(const std::string&, Eigen::Matrix4d&);

} //namespace manipulator
#endif /* MANIPULATOR_H_ */
