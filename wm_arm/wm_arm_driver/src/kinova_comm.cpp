/*
 * kinovaComm.cpp
 *
 *  Created on: Mar 19, 2016
 *      Author: xhache
 */

#include <iostream>
#include <vector>

#include "wm_arm_driver/kinova_comm.h"

namespace kinova
{
	kinovaComm::kinovaComm(const ros::NodeHandle& nh, boost::recursive_mutex &apiMutex)
		:nh_(nh), apiMutex_(apiMutex)
	{
		boost::recursive_mutex::scoped_lock lock(apiMutex_);

		isControlApiStarted_ = false;
		initSuccess_ = false;

		// get parameters
		double tmpParam;
		nh_.param("maxVelocity_joints123", tmpParam, 30.0);
		if (tmpParam < 0)
		{
			tmpParam *= -1.0;
			ROS_WARN("Maximum velocity cannot be negative. Setting maximum velocity for joints 1, 2 and 3 at %f degrees/second.", tmpParam);
		}
		else
		{
			ROS_INFO("Setting maximum velocity for joints 1, 2 and 3 at %f degrees/second.", tmpParam);
		}
		maxVel123_ = (float)tmpParam;
		nh_.param("maxVelocity_joints456", tmpParam, 30.0);
		if (tmpParam < 0)
		{
			tmpParam *= -1.0;
			ROS_WARN("Maximum velocity cannot be negative. Setting maximum velocity for joints 4, 5 and 6 at %f degrees/second.", tmpParam);
		}
		else
		{
			ROS_INFO("Setting maximum velocity for joints 4, 5 and 6 at %f degrees/second", tmpParam);
		}
		maxVel456_ = (float)tmpParam;

		nh_.param("jointMovement_Synchronization", enableJointsSync_, 1);
		if (enableJointsSync_ != 1 || enableJointsSync_ != 0)
		{
			ROS_WARN("Invalid input for joints synchronization. Setting default value");
			enableJointsSync_ = 1;
		}
		if (enableJointsSync_ == 1)
		{
			ROS_INFO("Joints' movement will be synchronized.");
		}
		else if (enableJointsSync_ == 0)
		{
			ROS_INFO("Joints' movement will not be synchronized.");
		}

		// initialize API
		int res = kAPI_.initAPI();

		if (res != NO_ERROR_KINOVA)
		{
			std::cout << "API initialization failed. Error code: " << res << std::endl;
			return;
		}

		KinovaDevice deviceList[MAX_KINOVA_DEVICE];

		// get all devices connected to the client's USB bus
		unsigned int nbDevices = kAPI_.getDevices(deviceList, res);

		if (res != NO_ERROR_KINOVA)
		{
			std::cout << "No device found on the USB bus. Error code: " << res << std::endl;
			return;
		}
		else
		{
			std::cout << "Found " << nbDevices << " device(s)." << std::endl;
		}

		// set active device
		if (nbDevices == 1)
		{
			std::cout << "Setting device as active device." << std::endl;
			res = kAPI_.setActiveDevice(deviceList[0]);
			if (res != NO_ERROR_KINOVA)
			{
				std::cout << "Could not set first device found as active device. Error code: " << res << std::endl;
				return;
			}
			std::cout << "**** Active device information ****" << std::endl;
			std::cout << "Serial number: " << deviceList[0].SerialNumber << std::endl;
			std::cout << "Model: " << deviceList[0].Model << std::endl;
			std::cout << "Code version: " << deviceList[0].VersionMajor << "." <<
					deviceList[0].VersionMinor << "." << deviceList[0].VersionRelease << std::endl;
		}
		else if (nbDevices > 1)
		{
			std::cout << "Multiple devices. Specify which device you want active." << std::endl;
			/* TO DO:
			 * Loop through all devices, print devices' informations and ask user to select a device
			 */
		}

		// tell the controller that from now on, commands will be issued by the API
		res = kAPI_.startControlAPI();

		if (res != NO_ERROR_KINOVA)
		{
			std::cout << "Could not start control over the controller. Error code: " << res << std::endl;
			return;
		}

		// tell the controller that the actuators will be commanded joint by joint
		res = kAPI_.setAngularControl();

		if (res != NO_ERROR_KINOVA)
		{
			std::cout << "Could not set angular control. Error code: " << res << std::endl;
			return;
		}

		// get trajectory FIFO information
		TrajectoryFIFO trajInfo;

		res = kAPI_.getGlobalTrajectoryInfo(trajInfo);
		if (res != NO_ERROR_KINOVA)
		{
			std::cout << "Could not get trajectory FIFO information. Error code: " << res << std::endl;
			return;
		}

		trajectoryCount_ = trajInfo.TrajectoryCount;
		trajectoryCapacity_ = trajInfo.MaxSize;

		ROS_INFO("Initialization completed succesfully.");
		initSuccess_ = true;
	}

	kinovaComm::~kinovaComm()
	{
		boost::recursive_mutex::scoped_lock lock(apiMutex_);
		kAPI_.stopControlAPI();
		kAPI_.closeAPI();
	}

	int kinovaComm::myGetAngularPosition(AngularPosition& posInfo)
	{
		boost::recursive_mutex::scoped_lock lock(apiMutex_);

		int res = kAPI_.getAngularPosition(posInfo);

		return res;
	}

	int kinovaComm::mySendAngularPos(const std::vector<double>& angularCmd, float delay)
	{
		boost::recursive_mutex::scoped_lock lock(apiMutex_);

		if (trajectoryCount_ < trajectoryCapacity_)
		{
			TrajectoryPoint PosCmd;
			PosCmd.InitStruct();

			// enable limitations
			PosCmd.LimitationsActive = 1;

			// set velocity limitations
			// speedParameter1 controls velocity limits for joints 1, 2 and 3 (ONLY if angular control is enabled)
			// speedParameter2 controls velocity limits for joints 4, 5 and 6 (ONLY if angular control is enabled)
			PosCmd.Limitations.speedParameter1 = maxVel123_;
			PosCmd.Limitations.speedParameter2 = maxVel456_;

			// tell the controller the commands represent angular positions
			PosCmd.Position.Type = ANGULAR_POSITION;
			// no fingers, no movement
			PosCmd.Position.HandMode = HAND_NOMOVEMENT;
			// delay, in seconds
			PosCmd.Position.Delay = delay;
			// enable/disable joints synchronization
			PosCmd.SynchroType = enableJointsSync_;

			PosCmd.Position.Actuators.Actuator1 = (float)angularCmd.at(0);
			PosCmd.Position.Actuators.Actuator2 = (float)angularCmd.at(1);
			PosCmd.Position.Actuators.Actuator3 = (float)angularCmd.at(2);
			PosCmd.Position.Actuators.Actuator4 = (float)angularCmd.at(3);
			PosCmd.Position.Actuators.Actuator5 = (float)angularCmd.at(4);

			// send cmd
			int res = kAPI_.sendAdvanceTrajectory(PosCmd);

			return res;
		}
		else
		{
			return ERROR_OPERATION_INCOMPLETED;
		}
	}

	int kinovaComm::mySendAngularVel(const std::vector<double>& angularCmd)
	{
		boost::recursive_mutex::scoped_lock lock(apiMutex_);

		if (trajectoryCount_ < trajectoryCapacity_)
		{
			TrajectoryPoint VelCmd;
			VelCmd.InitStruct();

			// enable limitations
			VelCmd.LimitationsActive = 1;

			// set velocity limitations
			// speedParameter1 controls velocity limits for joints 1, 2 and 3 (ONLY if angular control is enabled)
			// speedParameter2 controls velocity limits for joints 4, 5 and 6 (ONLY if angular control is enabled)
			VelCmd.Limitations.speedParameter1 = 30.0f;
			VelCmd.Limitations.speedParameter2 = 30.0f;

			// tell the controller the commands represent angular velocities
			VelCmd.Position.Type = ANGULAR_VELOCITY;
			// no fingers, no movement
			VelCmd.Position.HandMode = HAND_NOMOVEMENT;
			// no delay
			VelCmd.Position.Delay = 0.0f;

			VelCmd.Position.Actuators.Actuator1 = (float)angularCmd.at(0);
			VelCmd.Position.Actuators.Actuator2 = (float)angularCmd.at(1);
			VelCmd.Position.Actuators.Actuator3 = (float)angularCmd.at(2);
			VelCmd.Position.Actuators.Actuator4 = (float)angularCmd.at(3);
			VelCmd.Position.Actuators.Actuator5 = (float)angularCmd.at(4);

			// send cmd
			int res = kAPI_.sendAdvanceTrajectory(VelCmd);

			return res;
		}
		else
		{
			return ERROR_OPERATION_INCOMPLETED;
		}
	}

	int kinovaComm::myGetAngularForce(AngularPosition& torqueInfo)
	{
		boost::recursive_mutex::scoped_lock lock(apiMutex_);

		int res = kAPI_.getAngularForce(torqueInfo);

		return res;
	}

	int kinovaComm::myGetGlobalTrajectoryInfo()
	{
		boost::recursive_mutex::scoped_lock lock(apiMutex_);

		TrajectoryFIFO trajInfo;

		int res = kAPI_.getGlobalTrajectoryInfo(trajInfo);

		if (res != NO_ERROR_KINOVA)
		{
			kAPI_.stopControlAPI();
		}
		else
		{
			trajectoryCount_ = trajInfo.TrajectoryCount;
			trajectoryCapacity_ = trajInfo.MaxSize;
		}

		return res;
	}

	int kinovaComm::myMoveHome(const std::vector<double>& homePos)
	{
		boost::recursive_mutex::scoped_lock lock(apiMutex_);

		kAPI_.eraseAllTrajectories();

		int res = mySendAngularPos(homePos, 1.0f);

		return res;
	}

	int kinovaComm::myStartControlAPI()
	{
		boost::recursive_mutex::scoped_lock lock(apiMutex_);

		int res = kAPI_.startControlAPI();

		return res;
	}

	int kinovaComm::myStopControlAPI()
	{
		boost::recursive_mutex::scoped_lock lock(apiMutex_);

		int res = kAPI_.stopControlAPI();

		return res;
	}

	int kinovaComm::myEraseAllTrajectories()
	{
		boost::recursive_mutex::scoped_lock lock(apiMutex_);

		int res = kAPI_.eraseAllTrajectories();

		return res;
	}

} // namespace kinova


