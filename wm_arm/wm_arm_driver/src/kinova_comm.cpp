/*
 * kinovaComm.cpp
 *
 *  Created on: Mar 19, 2016
 *      Author: xhache
 */

#include <vector>

#include "wm_arm_driver/kinova_comm.h"

namespace kinova
{

	inline float radToDeg(double angle)
	{
		return (float)(angle * 180.0 / M_PI);
	}

	kinovaComm::kinovaComm(const ros::NodeHandle& nh, boost::recursive_mutex &apiMutex)
		:nh_(nh), apiMutex_(apiMutex)
	{
		boost::recursive_mutex::scoped_lock lock(apiMutex_);

		isControlApiStarted_ = false;
		initSuccess_ = false;

		// get parameters from the parameter server
		double tmpParam;
		nh_.param("/wm_arm_driver_node/max_velocity_joints123", tmpParam, 30.0);
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
		nh_.param("/wm_arm_driver_node/max_velocity_joints456", tmpParam, 30.0);
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

		bool jointSync;
		nh_.param("/wm_arm_driver_node/joint_movement_synchronization", jointSync, true);
		if (jointSync == 1)
		{
			ROS_INFO("Joints' movement will be synchronized.");
			enableJointsSync_ = 1;
		}
		else
		{
			ROS_INFO("Joints' movement will not be synchronized.");
			enableJointsSync_ = 0;
		}

		if (!nh_.getParam("/wm_arm_driver_node/home_position", homePosition_))
		{
			ROS_WARN("Joint space home position not specified. Assuming every joint's home position is 0.0.");
			homePosition_.assign(6, 0.0);
		}

		if(!nh_.getParam("/wm_arm_driver_node/joints_offset", jointsOffset_));
		{
			ROS_WARN("Joints' offset not specified. Assuming no offset.");
			/*
			 * TODO: make getParam() work and remove hard coded values
			 */
			jointsOffset_.push_back(166.60);
			jointsOffset_.push_back(89.25);
			jointsOffset_.push_back(24.26);
			jointsOffset_.push_back(350.85);
			jointsOffset_.push_back(331.23);
			jointsOffset_.push_back(0.0);
		}

		// initialize API
		int res = kAPI_.initAPI();

		if (res != NO_ERROR_KINOVA)
		{
			ROS_FATAL("API initialization failed. Error code: %d", res);
			return;
		}

		KinovaDevice deviceList[MAX_KINOVA_DEVICE];

		// get all devices connected to the client's USB bus
		unsigned int nbDevices = kAPI_.getDevices(deviceList, res);

		if (res != NO_ERROR_KINOVA)
		{
			ROS_FATAL("No device found on the USB bus. Error code: %d", res);
			return;
		}
		else
		{
			ROS_INFO("Found %d device(s).", nbDevices);
		}

		// set active device
		if (nbDevices == 1)
		{
			res = kAPI_.setActiveDevice(deviceList[0]);
			if (res != NO_ERROR_KINOVA)
			{
				ROS_FATAL("Could not set first device found as active device. Error code: %d", res);
				return;
			}
			ROS_INFO("**** Active device information ****");
			ROS_INFO("Serial number: %s", deviceList[0].SerialNumber);
			ROS_INFO("Model: %s", deviceList[0].Model);
			ROS_INFO("Code version: %d.%d.%d", deviceList[0].VersionMajor, deviceList[0].VersionMinor, deviceList[0].VersionRelease);
		}
		else if (nbDevices > 1)
		{
			ROS_INFO("Multiple devices. Specify which device you want active.");
			/* TODO
			 * Loop through all devices, print devices' informations and ask user to select a device
			 */
		}

		// tell the controller that from now on, commands will be issued by the API
		res = kAPI_.startControlAPI();

		if (res != NO_ERROR_KINOVA)
		{
			ROS_FATAL("Could not start control over the controller. Error code: %d", res);
			return;
		}

		// tell the controller that the actuators will be commanded joint by joint
		res = kAPI_.setAngularControl();

		if (res != NO_ERROR_KINOVA)
		{
			ROS_FATAL("Could not set angular control. Error code: %d", res);
			return;
		}

		// get trajectory FIFO information
		TrajectoryFIFO trajInfo;

		res = kAPI_.getGlobalTrajectoryInfo(trajInfo);
		if (res != NO_ERROR_KINOVA)
		{
			ROS_FATAL("Could not get trajectory FIFO information. Error code: %d", res);
			return;
		}

		trajectoryCount_ = trajInfo.TrajectoryCount;
		trajectoryCapacity_ = trajInfo.MaxSize;

		ROS_INFO("Communication initialization completed succesfully.");
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

			PosCmd.Position.Actuators.Actuator1 = radToDeg(angularCmd.at(0)) + jointsOffset_.at(0);
			PosCmd.Position.Actuators.Actuator2 = radToDeg(angularCmd.at(1)) + jointsOffset_.at(1);
			PosCmd.Position.Actuators.Actuator3 = radToDeg(angularCmd.at(2)) + jointsOffset_.at(2);
			PosCmd.Position.Actuators.Actuator4 = radToDeg(angularCmd.at(3)) + jointsOffset_.at(3);
			PosCmd.Position.Actuators.Actuator5 = radToDeg(angularCmd.at(4)) + jointsOffset_.at(4);

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
			VelCmd.Limitations.speedParameter1 = maxVel123_;
			VelCmd.Limitations.speedParameter2 = maxVel456_;

			// tell the controller the commands represent angular velocities
			VelCmd.Position.Type = ANGULAR_VELOCITY;
			// no fingers, no movement
			VelCmd.Position.HandMode = HAND_NOMOVEMENT;
			// no delay
			VelCmd.Position.Delay = 0.0f;

			VelCmd.Position.Actuators.Actuator1 = (float)(M_PI * angularCmd.at(0) / 180.0);
			VelCmd.Position.Actuators.Actuator2 = (float)(M_PI * angularCmd.at(1) / 180.0);
			VelCmd.Position.Actuators.Actuator3 = (float)(M_PI * angularCmd.at(2) / 180.0);
			VelCmd.Position.Actuators.Actuator4 = (float)(M_PI * angularCmd.at(3) / 180.0);
			VelCmd.Position.Actuators.Actuator5 = (float)(M_PI * angularCmd.at(4) / 180.0);

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

	int kinovaComm::myMoveHome()
	{
		boost::recursive_mutex::scoped_lock lock(apiMutex_);

		kAPI_.eraseAllTrajectories();

		int res = mySendAngularPos(homePosition_, 1.0f);

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


