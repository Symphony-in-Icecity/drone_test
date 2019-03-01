/**
 * @file position_ctrl_with_rc.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */
 
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/tf.h>
// #include <mavros_msgs/Thrust.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/RCIn.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandLong.h>
#include "opencvtest/contours.h"
#include "tf/transform_datatypes.h"

// define 4 modes
#define init_mode 0
#define hover_mode 1
#define mission_mode 2
#define rc_landing_mode 3

// RC compensation input 
float rc_d_roll = 0;
float rc_d_pitch = 0;
float rc_d_yaw = 0;
float rc_d_thrust = 0;
tfScalar yaw,pitch,roll; //RC oreintation
tf::Quaternion q;

// picture size is 480 * 640
#define picture_centerX 320;
#define picture_centerY 240;

// init status as init_mdoe
int status = init_mode;

int offb_flag = 0; // if it's offboard mode now, offb_flag = 1
int armed_flag = 0;// if the vehicle is armed, armed_flag = 1
int delta_pixels[2]; // delta pixels in X and Z direction
float kp = 0.01; // the proportional between "delta_pixels" and "delta loccal position(meters)"


mavros_msgs::State current_state; // show pixkawk status
mavros_msgs::RCIn current_RC_in; // get the radio input
geometry_msgs::PoseStamped current_pose; // pose of the vehicle
sensor_msgs::NavSatFix gps_raw_fix; // data of gps
opencvtest::img_pro_info camera_data; // data of box center and some other flags
geometry_msgs::Quaternion px4_xyzw;

void state_cb(const mavros_msgs::State::ConstPtr& msg){ // state callback
    current_state = *msg;
}

void RC_subCallback(const mavros_msgs::RCIn::ConstPtr& RCmsg) // radio data callback
{
	current_RC_in = *RCmsg;
}

void local_pos_subCallback(const geometry_msgs::PoseStamped::ConstPtr& curr_p) // local position callback
{
	current_pose = *curr_p;
}

void gps_raw_subCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_data){ // GPS data callback
	gps_raw_fix = *gps_data;
}

void cam_subCallback(const opencvtest::img_pro_info::ConstPtr& cam_data){ // camera data callback
	camera_data = *cam_data;
}

// a transformation from Quaternion to Eluer angle
geometry_msgs::Vector3 Quaternion2Euler(const geometry_msgs::Quaternion msg)
{
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg, quat);
 
    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
 
    // the found angles are written in a geometry_msgs::Vector3
    geometry_msgs::Vector3 rpy;
    rpy.x = roll ;
    rpy.y = pitch ;
    rpy.z = yaw ;

 
    //ROS_INFO("angles: roll=%f pitch=%f yaw=%f", rpy.x, rpy.y, rpy.z);

	return rpy;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
 
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 1, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 1);
    ros::Publisher attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude",10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
	ros::ServiceClient take_off_client = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/takeoff");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
	ros::Subscriber RC_sub = nh.subscribe("mavros/rc/in", 1, RC_subCallback);
	ros::Subscriber cam_sub = nh.subscribe("/contours_topic", 1, cam_subCallback);
	

	ros::Subscriber local_position_sub = nh.subscribe("mavros/local_position/pose", 1, local_pos_subCallback);

	ros::Subscriber gps_raw_sub = nh.subscribe("/mavros/global_position/raw/fix", 1, gps_raw_subCallback);
	
	
    //the setpoint publishing rate MUST be faster than 2Hz,or pxiahwk will be switched from offboard mode to RTL mode
    ros::Rate rate(20.0);

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD"; // set mdoe
 
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true; // set arm positive

	geometry_msgs::PoseStamped pose; // vehicle pose variables: position and oreintation
    pose.pose.position.x = 0 ;
    pose.pose.position.y = 0 ;
    pose.pose.position.z = 2 ;


	while(ros::ok()){
		//****tatus mechine*******************
		if(status == init_mode){
			//1.waiting connecting to FCU via USB;
			while(!current_state.connected){
					ros::spinOnce();
					rate.sleep();
			}
			ROS_INFO("pixhawk has been connected via USB!\n");
			// waiting for GPS fixed
			while(ros::ok() && gps_raw_fix.status.status){
					ros::spinOnce();
					rate.sleep();
			}
			//3.switching to offboard mode and try to arm;
			ros::Time last_request = ros::Time::now();
			while(!(offb_flag * armed_flag) && ros::ok()){
				if( current_state.mode != "OFFBOARD" &&
					(ros::Time::now() - last_request > ros::Duration(5.0))){
					if( set_mode_client.call(offb_set_mode) &&
						offb_set_mode.response.mode_sent){
						ROS_INFO("Offboard enabled");
						offb_flag = 1;
					}
					last_request = ros::Time::now();
				} 
				else {
					if( !current_state.armed &&
						(ros::Time::now() - last_request > ros::Duration(5.0))){
						if( arming_client.call(arm_cmd) &&
							arm_cmd.response.success){
							ROS_INFO("Vehicle armed");
							armed_flag = 1;
						}
						last_request = ros::Time::now();
					}
				}
				local_pos_pub.publish(pose);//the setpoint publishing rate MUST be faster than 2Hz,or pxiahwk will be switched from offboard mode to RTL mode
				ros::spinOnce();
				rate.sleep();
			}
			//3.transfer to hover_mode
			status = hover_mode;
			ROS_INFO("switching to hover mode...");
		}
		else if(status == hover_mode){
			if(2 - current_pose.pose.position.z > 0.2){ // if the vehicle altitude is higher than 1.8m, then we think it's time to switching to hover mode...
				printf("error: %f \n",2 - current_pose.pose.position.z);
				printf("pose: %f \n",pose.pose.position.z);
			}
			else{
				status = mission_mode;
				ROS_INFO("switching to mission mode...");
			}
			local_pos_pub.publish(pose);
		}
		else if(status == mission_mode){
			if(!camera_data.out_flag){ // if the box is in camera sight
				// RC compensation 
				rc_d_roll = float(current_RC_in.channels.at(0)-1513) / (-840) * 5;  //invert with -840
				rc_d_pitch =  float(current_RC_in.channels.at(1)-1513) / (-840) * 5;
				rc_d_yaw =  float(current_RC_in.channels.at(3)-1514) / (-840) * 5;
				rc_d_thrust = float(current_RC_in.channels.at(2)-1513) / 840 * 5;
				printf("rc_d_yaw : %f \n",rc_d_yaw * 180 / 3.14);

				roll = 0;
				pitch = 0;
				px4_xyzw.x = current_pose.pose.orientation.x;
				px4_xyzw.y = current_pose.pose.orientation.y;
				px4_xyzw.z = current_pose.pose.orientation.z;
				px4_xyzw.w = current_pose.pose.orientation.w;
				// printf("x_cur: %f \n",px4_xyzw.x);
				// printf("y_cur: %f \n",px4_xyzw.y);
				// printf("z_cur: %f \n",px4_xyzw.z);
				// printf("w_cur: %f \n",px4_xyzw.w);
				yaw = Quaternion2Euler(px4_xyzw).z + rc_d_yaw;
				// yaw = 30 * 3.14 / 180 + rc_d_yaw;
				printf("tg_yaw : %f \n",yaw * 180 / 3.14159265358979);	
				q.setRPY(roll, pitch, yaw);

				pose.pose.position.x = current_pose.pose.position.x + rc_d_pitch;
				// pose.pose.position.y = pose.pose.position.y + rc_d_roll ;
				// pose.pose.position.z = pose.pose.position.z + rc_d_thrust ;
				pose.pose.orientation.x = q[0];
				pose.pose.orientation.y = q[1];
				pose.pose.orientation.z = q[2];
				pose.pose.orientation.w = q[3];
				delta_pixels[0] = camera_data.x_pos - picture_centerX;
				delta_pixels[1] = camera_data.y_pos - picture_centerY;
				pose.pose.position.y = current_pose.pose.position.y +  (-kp * delta_pixels[0]) + rc_d_roll;
				pose.pose.position.z = current_pose.pose.position.z +  (-kp * delta_pixels[1])+ rc_d_thrust;
				printf("out_flag:  %d \n",camera_data.out_flag);
			}
			else{
				ROS_INFO("box is out of sight!");  // if the box is in camera sight
			}
			printf("tg_yaw_prepub : %f \n",yaw * 180 / 3.14);
			local_pos_pub.publish(pose); // publish target position
		}
		else{
			;
		}
		


		ros::spinOnce();
		rate.sleep();
	}
 
    return 0;
}

