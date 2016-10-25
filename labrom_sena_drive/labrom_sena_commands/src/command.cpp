/*************************************************************************
*   Define commands, steering and speed, to be send to the vehicle drivers
*   
*   This file is part of labrom_sena_commands
*
*   labrom_sena_commands is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   labrom_sena_commands is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with labrom_sena_commands.  If not, see <http://www.gnu.org/licenses/>.
***************************************************************************/

#include "labrom_sena_commands/command.h"


namespace labrom_sena_commands{

/**
 * Empty constructor.
 */
CmdNode::CmdNode(void): pnh_("~"){
	// Paramas  
	pnh_.param("ackermann_steering_ratio", _ackermann_steering_ratio, 1038);
	pnh_.param("node_loop_rate", _loop_rate, 20);
	// Start subscribers
	sub_ackermann_ = nh_.subscribe("ackermann_cmd", 1, &CmdNode::AckermannCallback, this);

	// Start publishers
	pub_steering_cmd_ = nh_.advertise<labrom_sena_msgs::SteeringCommand>("steering_cmd", 1);
};

/**
 * Empty destructor.
 */
 CmdNode::~CmdNode(void){};

/**
 * Ackermann callback. Save last drive command received
 */
void CmdNode::AckermannCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &msg){
	ackermann_ = *msg; 
}

/**
 * Publish steering command message (steering wheel reference)
 */
void CmdNode::PublishSteeringCmd(void){

	// Transform steering angle (radians) to steering wheel angle (degrees)
	steering_.angle = ackermann_.drive.steering_angle * _ackermann_steering_ratio;

	pub_steering_cmd_.publish(steering_);	
}

/**
 * ROS spin loop
 */
void CmdNode::Spin(void){
	ros::Rate loop_rate(_loop_rate);

	while(ros::ok()){
		// Publish steering message
		PublishSteeringCmd();

		// Spin and Sleep
		ros::spinOnce();
		loop_rate.sleep();

	}
}

}  // labrom_sena_commands namespace

int main(int argc, char **argv){

	// Initialize ROS within this node
	ros::init(argc, argv, "command_node");
	
	// Commands node
	labrom_sena_commands::CmdNode node;
	node.Spin();
}
