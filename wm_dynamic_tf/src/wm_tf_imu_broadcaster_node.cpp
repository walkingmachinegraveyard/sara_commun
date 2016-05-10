#include "wm_tf_imu_broadcaster_node.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

WM_TF_BC::WM_TF_BC(): x(0), y(0), yaw(0)
{
	sub = nh.subscribe<sensor_msgs::Imu>("imu/data", 1, &WM_TF_BC::callback, this);
}

void WM_TF_BC::callback(const sensor_msgs::Imu::ConstPtr& imu) 
{
	geometry_msgs::TransformStamped imu_tf;

	ros::Time current_time = ros::Time::now();
	
	double dt = (current_time - imu->header.stamp).toSec();
	
	double delta_yaw = imu->angular_velocity.z * dt;
	
	double acc_x = -1 * imu->linear_acceleration.x;
	double acc_y = -1 * imu->linear_acceleration.y;
			
	double trans_x = (acc_x*cos(delta_yaw) - acc_y*sin(delta_yaw)) * dt * dt;
	double trans_y = (acc_x*sin(delta_yaw) - acc_y*cos(delta_yaw)) * dt * dt;
	
	this->x += trans_x;
	this->y += trans_y;
	this->yaw += delta_yaw;
	
	imu_tf.header.stamp = current_time;
	imu_tf.header.frame_id = "imu_base";
	imu_tf.child_frame_id = "sara_base";
	
	imu_tf.transform.translation.x = this->x;
	imu_tf.transform.translation.y = this->y;
	imu_tf.transform.translation.z = 0.0;
	
	tf2::Quaternion q;
	q.setRPY(0.0, 0.0, this->yaw);
	imu_tf.transform.rotation.x = q.x();
	imu_tf.transform.rotation.y = q.y();
	imu_tf.transform.rotation.z = q.z();
	imu_tf.transform.rotation.w = q.w();
	
	imu_broadcaster.sendTransform(imu_tf);
}


int main(int argc, char** argv) 
{
	ros::init(argc, argv, "wm_tf_imu_broadcaster_node");

	WM_TF_BC wm_tf_bc;
	
	ros::spin();

	return 0;
}
