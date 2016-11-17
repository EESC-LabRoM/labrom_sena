/*************************************************************************
*   Send steering commands to channel drive topic
*   
*   This file is part of labrom_sena_steering
*
*   labrom_sena_steering is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   labrom_sena_steering is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with labrom_sena_steering.  If not, see <http://www.gnu.org/licenses/>.
***************************************************************************/

#include "labrom_sena_steering/steering.h"

namespace labrom_sena_steering{

/**
 * Empty constructor.
 */
SteerNode::SteerNode(void): pnh_("~"){
    // Params
    pnh_.param("motor_drive_ratio", _motor_drive_ratio, 1.364);
    pnh_.param("node_loop_rate", _loop_rate, 20);
    // Start subscribers
    sub_steering_ = nh_.subscribe("steering_cmd", 1, &SteerNode::SteeringCallback, this);

    // Start publishers
    pub_steering_channel_ = nh_.advertise<roboteq_msgs::Command>("roboteq_driver/channel_1/cmd", 1);
};

/**
 * Empty destructor.
 */
SteerNode::~SteerNode(void){};

/**
 * Steering callback. Save last steering command
 */
void SteerNode::SteeringCallback(const labrom_sena_msgs::SteeringCommand::ConstPtr &msg){
    command_ = *msg;    
}

/**
 * Publishing steering channel drive command message
 */
void SteerNode::PublishSteeringChannel(void){

    // Mount the steering channel drive command message
    steering_.setpoint = (command_.angle * M_PI) * _motor_drive_ratio / 180;
    steering_.mode = roboteq_msgs::Command::MODE_POSITION_ANGULAR;
    
    pub_steering_channel_.publish(steering_);
}

/**
 * ROS spin loop
 */
void SteerNode::Spin(void){
	ros::Rate loop_rate(_loop_rate);

	while(ros::ok()){
		// Publish steering channel drive message
		PublishSteeringChannel();

		// Spin and Sleep
		ros::spinOnce();
		loop_rate.sleep();

	}
}

} // labrom_sena_steering namespace

int main(int argc, char ** argv){

    //Initialize ROS within this node
    ros::init(argc, argv, "steering_node");

    // Steering node
    labrom_sena_steering::SteerNode node;
    node.Spin();
}