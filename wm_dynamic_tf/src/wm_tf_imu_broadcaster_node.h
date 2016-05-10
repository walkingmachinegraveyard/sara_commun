#include "ros/ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "sensor_msgs/Imu.h"

class WM_TF_BC 
{
	public:
		WM_TF_BC();

	private:
		void callback(const sensor_msgs::Imu::ConstPtr& odom);

		ros::NodeHandle nh;
		
		tf2_ros::TransformBroadcaster imu_broadcaster;
		ros::Subscriber sub;
		
		double x;
		double y;
		double yaw;
};
