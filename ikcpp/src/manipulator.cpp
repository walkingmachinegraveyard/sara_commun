/*
 * ccd.cpp
 *
 *  Created on: Mar 19, 2016
 *      Author: xhache
 */

#include <iostream>
#include "include/ikcpp/manipulator.h"
#include "include/ikcpp/ik_solver.h"

namespace manipulator
{
	Joint::Joint(std::string jointType, double theta, double d, double alpha, double t, double lowerLimit,
			double upperLimit, double homePosition)
				:type(jointType), lowerLimit(lowerLimit), upperLimit(upperLimit), homePosition(homePosition),
				 previousPosition(homePosition), currentPosition(homePosition)
	{
		dhParam.theta = theta;
		dhParam.d = d;
		dhParam.alpha = alpha;
		dhParam.t = t;
	}

	Joint::~Joint()
	{

	}

	void Joint::setHomePosition(double pos)
	{
		homePosition = pos;
		return;
	}

	void Joint::setCurrentPosition(double pos)
	{
		currentPosition = pos;
		return;
	}

	Manipulator::Manipulator(char* fileName)
	:validModel(false)
	{
		int result = parseXML(fileName);

		if(result == NO_ERROR)
		{
			validModel = true;
		}
	}

	Manipulator::~Manipulator()
	{

	}

	bool Manipulator::isValid()
	{
		return validModel;
	}

	int Manipulator::parseXML(const char* file)
	{
		pugi::xml_document doc;
		pugi::xml_parse_result result = doc.load_file(file);

		if (result.status)
		{
			std::cout << result.description() << std::endl;
			return PARSE_ERROR;
		}

		pugi::xml_node root = doc.child("manipulator");
		std::string nodeName = root.name();

		if (nodeName.compare("manipulator"))
		{
			std::cout << "Invalid XML file: Expected root to be \"manipulator\" but found \""
					<< root.name() << "\"." << std::endl;
			return PARSE_ERROR;
		}
		nbJoints = root.attribute("numberOfJoints").as_int();

		angleUnit = root.child("angle").attribute("unit").as_string();

		if (angleUnit.compare("degree") && angleUnit.compare("radian"))
		{
			std::cout << "Invalid XML file: Angle unit must be \"degree\" or \"radian\"" << std::endl;
			return PARSE_ERROR;
		}

		pugi::xml_node staticFrame = root.child("world_transform");
		nodeName = staticFrame.name();
		if (nodeName.compare("world_transform"))
		{
			std::cout << "Invalid XML file: Could not find \"world_transform\" node." << std::endl;
			return PARSE_ERROR;
		}
		Manipulator::parseTransform(staticFrame, worldFrame);

		for (pugi::xml_node joint = root.child("joint"); joint; joint = joint.next_sibling("joint"))
		{
			std::string type = joint.attribute("type").as_string();

			if (type.compare("rotation") && type.compare("translation"))
			{
				std::cout << "Invalid XML file: joint type must be \"rotation\" or \"translation\"" << std::endl;
				return PARSE_ERROR;
			}

			double theta = joint.child("dh_param").attribute("rotational_angle").as_double();
			double d = joint.child("dh_param").attribute("link_length").as_double();
			double alpha = joint.child("dh_param").attribute("offset_length").as_double();
			double t = joint.child("dh_param").attribute("twist_angle").as_double();

			double lowerLimit = joint.child("limits").attribute("lower").as_double();
			double upperLimit = joint.child("limits").attribute("upper").as_double();

			double homePosition = joint.child("home").attribute("position").as_double();

			if (angleUnit.compare("degree") == 0)
			{
				theta = ToRadians(theta);
				t = ToRadians(t);
				if (type.compare("rotation") == 0)
				{
					lowerLimit = ToRadians(lowerLimit);
					upperLimit = ToRadians(upperLimit);
//					homePosition = ToRadians(upperLimit);
				}
			}

			Joint j(type, theta, d, alpha, t, lowerLimit, upperLimit, homePosition);

			joints.push_back(j);
		}

		if (nbJoints != joints.size())
		{
			std::cout << "Invalid XML file: Number of joints mismatch. " << nbJoints << " joints were declared but " <<
						joints.size() << " joints were defined." << std::endl;
			return PARSE_ERROR;
		}

		staticFrame = root.child("effector_transform");
		nodeName = staticFrame.name();
		if (nodeName.compare("effector_transform"))
		{
			std::cout << "Invalid XML file: Could not find \"effector_transform\" node." << std::endl;
			return PARSE_ERROR;
		}
		parseTransform(staticFrame, effectorFrame);

		return NO_ERROR;
	}

	int Manipulator::parseTransform(pugi::xml_node& node, Eigen::Matrix4d& mat)
	{
		double roll = node.child("roll").attribute("value").as_double();
		double pitch = node.child("pitch").attribute("value").as_double();
		double yaw = node.child("yaw").attribute("value").as_double();

		if (angleUnit.compare("degree") == 0)
		{
			roll = ToRadians(roll);
			pitch = ToRadians(pitch);
			yaw = ToRadians(yaw);
		}

		double tx = node.child("tx").attribute("value").as_double();
		double ty = node.child("ty").attribute("value").as_double();
		double tz = node.child("tz").attribute("value").as_double();

		mat << 	cos(pitch)*cos(roll), sin(pitch)*cos(roll)*sin(yaw)-sin(roll)*cos(yaw), 	// frame(0,0), frame(0,1)
				sin(pitch)*cos(roll)*cos(yaw)+sin(roll)*sin(yaw), tx, 						// frame(0,2), frame(0,3)
				cos(pitch)*sin(roll), cos(roll)*cos(yaw)+sin(pitch)*sin(roll)*sin(yaw),		// frame(1,0), frame(1,1)
				sin(pitch)*sin(roll)*cos(yaw)-cos(roll)*sin(yaw), ty,						// frame(1,2), frame(1,3)
				-1.0*sin(pitch), cos(pitch)*sin(yaw), cos(pitch)*cos(yaw), tz,				// frame(2,0), frame(2,1), frame(2,2), frame(2,3)
				0.0, 0.0, 0.0, 1.0;															// frame(3,0), frame(3,1), frame(3,2), frame(3,3)

		return NO_ERROR;
	}

	void goPose(const std::string& pose, Eigen::Matrix4d& target)
	{
		if (pose.compare("front") == 0)
		{
			target << 0.0, 0.0, 1.0, 0.28575,
					0.0, -1.0, 0.0, -0.2159,
					1.0, 0.0, 0.0, 0.7366,
					0.0, 0.0, 0.0, 1.0;
			return;
		}
		else if (pose.compare("up") == 0)
		{
			target << -1.0, 0.0, 0.0, 0.0,
						0.0, -1.0, 0.0, -0.2159,
						0.0, 0.0, 1.0, 1.7018,
						0.0, 0.0, 0.0, 1.0;
			return;
		}
		else if (pose.compare("overreach") == 0)
		{
			target << -1.0, 0.0, 0.0, 0.0,
						0.0, -1.0, 0.0, -0.2159,
						0.0, 0.0, 1.0, 3.0,
						0.0, 0.0, 0.0, 1.0;
			return;
		}
		else if (pose.compare("side") == 0)
		{
			target << 1.0, 0.0, 0.0, 0.0,
						0.0, 0.0, -1.0, -0.7239,
						0.0, 1.0, 0.0, 1.1176,
						0.0, 0.0, 0.0, 1.0;
			return;
		}
		else if (pose.compare("right") == 0)
		{
			target << 0.0, -1.0, 0.0, 0.0,
						0.0, 0.0, -1.0, -0.50165,
						1.0, 0.0, 0.0, 0.7366,
						0.0, 0.0, 0.0, 1.0;
			return;
		}
		else
		{
			return;
		}
	}

} // namespace manipulator
