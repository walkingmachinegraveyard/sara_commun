/*
 * kinovaComm.h
 *
 *  Created on: Mar 19, 2016
 *      Author: xhache
 */

#ifndef KINOVACOMM_H_
#define KINOVACOMM_H_

#include <vector>
#include <boost/thread/recursive_mutex.hpp>
#include "Kinova/Kinova.API.CommLayerUbuntu.h"
#include "Kinova/KinovaTypes.h"
#include "wm_arm_driver/kinova_api.h"

#include "ros/ros.h"


namespace kinova
{
	class kinovaComm
	{
	public:
		kinovaComm(const ros::NodeHandle&, boost::recursive_mutex&);
		~kinovaComm();
		bool initSuccesfull() {return initSuccess_;}

		int myStartControlAPI();
		int myStopControlAPI();

		int mySendAngularPos(const std::vector<double>&, float);
		int mySendAngularVel(const std::vector<double>&);

		int myMoveHome(const std::vector<double>&);

		int myGetAngularPosition(AngularPosition&);

		int myGetAngularForce(AngularPosition&);
		int myGetGlobalTrajectoryInfo();

		int myGetSensorsInfo(SensorsInfo&);

		int myEraseAllTrajectories();

		unsigned int getFifoCount() {return trajectoryCount_;}

	private:
		ros::NodeHandle nh_;
		boost::recursive_mutex& apiMutex_;
		kinova::kinovaAPI kAPI_;
		bool isControlApiStarted_;
		bool initSuccess_;
		unsigned int trajectoryCount_, trajectoryCapacity_;
		float maxVel123_, maxVel456_;
		int enableJointsSync_;
		int status_;
	};
} // namespace kinova
#endif /* KINOVACOMM_H_ */
