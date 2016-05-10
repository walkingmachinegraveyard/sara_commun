/*
 * kinovaApi.h
 *
 *  Created on: Mar 19, 2016
 *      Author: xhache
 */

/* This work is greatly inspired by Clearpath Robotics' jaco-driver ROS package */

#ifndef KINOVAAPI_H_
#define KINOVAAPI_H_

#include <dlfcn.h>

#include "Kinova/Kinova.API.CommLayerUbuntu.h"
#include "Kinova/KinovaTypes.h"

#include "ros/ros.h"

#define KINOVA_USB_LIBRARY "Kinova.API.USBCommandLayerUbuntu.so"

namespace kinova
{
	class kinovaAPI
	{
	 public:
		kinovaAPI();
		~kinovaAPI();

		int (*initAPI)(void);
		int (*closeAPI)(void);
		int (*getAPIVersion)(std::vector<int> &);
		int (*getDevices)(KinovaDevice[MAX_KINOVA_DEVICE], int &);
		int (*setActiveDevice)(KinovaDevice);

		int (*getGeneralInformations)(GeneralInformations &);
		int (*getQuickStatus)(QuickStatus &);
		int (*GetForcesInfo)(ForcesInfo &);

		int (*getCodeVersion)(std::vector<int> &);
		int (*startControlAPI)();
		int (*stopControlAPI)();
		int (*initFingers)();

		int (*moveHome)();

		int (*getCartesianPosition)(CartesianPosition &);
		int (*getAngularPosition)(AngularPosition &);
		int (*getAngularVelocity)(AngularPosition &);
		int (*getCartesianForce)(CartesianPosition &);
		int (*setCartesianForceMinMax)(CartesianInfo, CartesianInfo);
		int (*setCartesianInertiaDamping)(CartesianInfo, CartesianInfo);
		int (*startForceControl)();
		int (*stopForceControl)();
		int (*getAngularForce)(AngularPosition &);
		int (*getAngularCurrent)(AngularPosition &);
		int (*getControlType)(int &);
		int (*getActualTrajectoryInfo)(TrajectoryPoint &);
		int (*getGlobalTrajectoryInfo)(TrajectoryFIFO &);
		int (*getSensorsInfo)(SensorsInfo &);
		int (*setAngularControl)();
		int (*setCartesianControl)();
		int (*restoreFactoryDefault)();
		int (*sendJoystickCommand)(JoystickCommand);
		int (*sendAdvanceTrajectory)(TrajectoryPoint);
		int (*sendBasicTrajectory)(TrajectoryPoint);
		int (*getClientConfigurations)(ClientConfigurations &);
		int (*setClientConfigurations)(ClientConfigurations);
		int (*eraseAllTrajectories)();
		int (*getPositionCurrentActuators)(std::vector<float> &);
		int (*setActuatorPID)(unsigned int, float, float, float);
	};
} // namespace kinova
#endif /* KINOVAAPI_H_ */
