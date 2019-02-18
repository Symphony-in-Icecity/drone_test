/**
 * @file offb_node.cpp
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

int offb_flag = 0;
int armed_flag = 0;
 
mavros_msgs::State current_state;
mavros_msgs::RCIn current_RC_in;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void RC_subCallback(const mavros_msgs::RCIn::ConstPtr& RCmsg)
{
	current_RC_in = *RCmsg;
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
 
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude",10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
	ros::Subscriber RC_sub = nh.subscribe("mavros/rc/in", 10, RC_subCallback);
	
 
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
 
    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

	float k_roll = float(current_RC_in.channels.at(1)-1500) / 1;
	float k_pitch =  float(current_RC_in.channels.at(0)-1500) / 1;
	float k_yaw =  float(current_RC_in.channels.at(3)-1500) / 1;
	float k_thrust = float(current_RC_in.channels.at(2)-1500) / 1;
	
	// tfScalar yaw,pitch,roll;
	// yaw = 0;
	// pitch = 20 * 3.14 / 180;
	// roll = 0;
	// tf::Quaternion q;
	// q.setRPY(yaw,pitch,roll);

    // geometry_msgs::PoseStamped pose;
    // pose.pose.position.x = 0;
    // pose.pose.position.y = 0;
    // pose.pose.position.z = 1;
	// pose.pose.orientation.x = q[3];
	// pose.pose.orientation.y = q[0];
	// pose.pose.orientation.z = q[1];
	// pose.pose.orientation.w = q[2];

	mavros_msgs::AttitudeTarget attitude;
	// attitude.IGNORE_YAW_RATE = 4;
	// attitude.IGNORE_ROLL_RATE = 1;
	// attitude.IGNORE_PITCH_RATE = 2;
	attitude.type_mask = attitude.IGNORE_PITCH_RATE + attitude.IGNORE_YAW_RATE + attitude.IGNORE_ROLL_RATE;
	attitude.thrust = 0.5 + k_thrust;
	// attitude.orientation.

 
    //send a few setpoints before starting

    // for(int i = 100; ros::ok() && i > 0; --i){
    //     local_pos_pub.publish(pose);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

	// for(int i = 100;ros::ok() && i > 0; --i)
	// {
	// 	attitude_pub.publish(attitude);
	// 	ros::spinOnce();
	// 	rate.sleep();
	// }
 
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
 
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
 
    ros::Time last_request = ros::Time::now();
 
    while(ros::ok()){
		if(!(offb_flag * armed_flag)){
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
		}
        
	
        // local_pos_pub.publish(pose);
		attitude_pub.publish(attitude);
 
        ros::spinOnce();
        rate.sleep();
    }
 
    return 0;
}
