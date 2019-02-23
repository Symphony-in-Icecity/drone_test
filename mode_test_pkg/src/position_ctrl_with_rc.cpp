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
geometry_msgs::PoseStamped current_pose;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void RC_subCallback(const mavros_msgs::RCIn::ConstPtr& RCmsg)
{
	current_RC_in = *RCmsg;
}

void local_pos_subCallback(const geometry_msgs::PoseStamped::ConstPtr& curr_p)
{
	current_pose = *curr_p;
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
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
	ros::Subscriber RC_sub = nh.subscribe("mavros/rc/in", 1, RC_subCallback);

	ros::Subscriber local_position_sub = nh.subscribe("mavros/local_position/pose", 1, local_pos_subCallback);
	
	
	
 
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
 
    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
	
	// tfScalar yaw,pitch,roll;
	// yaw = 0;
	// pitch = 20 * 3.14 / 180;
	// roll = 0;
	// tf::Quaternion q;
	// q.setRPY(yaw,pitch,roll);

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 1 ;
    pose.pose.position.y = 1 ;
    pose.pose.position.z = 0 ;
	// pose.pose.orientation.x = q[3];
	// pose.pose.orientation.y = q[0];
	// pose.pose.orientation.z = q[1];
	// pose.pose.orientation.w = q[2];

	// mavros_msgs::AttitudeTarget attitude;
	// attitude.type_mask = attitude.IGNORE_PITCH_RATE + attitude.IGNORE_YAW_RATE + attitude.IGNORE_ROLL_RATE;
	// attitude.thrust = 0.5 + k_thrust;

 
    //send a few setpoints before starting

    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

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
        
		float k_roll = float(current_RC_in.channels.at(0)-1513) / 840;
		float k_pitch =  float(current_RC_in.channels.at(1)-1513) / (-840);
		float k_yaw =  float(current_RC_in.channels.at(3)-1514) / 840;
		float k_thrust = float(current_RC_in.channels.at(2)-1513) / 840;

		tfScalar yaw,pitch,roll;
		yaw = 0 + k_yaw;
		pitch = 0;
		roll = 0;
		tf::Quaternion q;
		q.setRPY(yaw,pitch,roll);

		// geometry_msgs::PoseStamped pose;
		pose.pose.position.x = current_pose.pose.position.x + k_pitch ;
		pose.pose.position.y = current_pose.pose.position.y + k_roll ;
		pose.pose.position.z = current_pose.pose.position.z + k_thrust ;
		pose.pose.orientation.x = q[3];
		pose.pose.orientation.y = q[0];
		pose.pose.orientation.z = q[1];
		pose.pose.orientation.w = q[2];
        local_pos_pub.publish(pose);
		// ROS_INFO("x,y,z");
		// printf("%f,%f,%f \n",pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);
		// ROS_INFO("pitch,yaw,roll");
		// printf("%f,%f,%f \n",pitch,yaw,roll);
		// ROS_INFO("current: x,y,z");
		// printf("%f,%f,%f \n",current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z);
		// ROS_INFO("pos error: x,y,z");
		// printf("%f,%f,%f \n",(pose.pose.position.x-current_pose.pose.position.x),(pose.pose.position.y-current_pose.pose.position.y),(pose.pose.position.z-current_pose.pose.position.z));
		// // attitude_pub.publish(attitude);
		// ROS_INFO("bias: pitch, roll, thrust");
		// printf("%f,%f,%f \n",k_pitch,k_roll,k_thrust);
 
        ros::spinOnce();
        rate.sleep();
    }
 
    return 0;
}
