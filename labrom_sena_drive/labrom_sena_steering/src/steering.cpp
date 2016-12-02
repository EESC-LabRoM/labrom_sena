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
	pnh_.param("steering_simulation", _simulate, false);

	static int qDepth = 1;
	ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);

	// Start subscribers
	sub_steering_ = nh_.subscribe("steering_cmd", qDepth, &SteerNode::SteeringCallback, this, noDelay);
	sub_driver_state_ = nh_.subscribe("roboteq_driver/status", qDepth, &SteerNode::DriverStatusCallback, this, noDelay);
	sub_steering_feedback_ = nh_.subscribe("roboteq_driver/steering/feedback", qDepth, &SteerNode::SteeringFeedbackCallback, this, noDelay);

	// Start publishers
	pub_steering_channel_ = nh_.advertise<roboteq_msgs::Command>("roboteq_driver/steering/cmd", qDepth);
	pub_steering_state_ = nh_.advertise<labrom_sena_msgs::SteeringState>("steering_state", qDepth);

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
 * Driver State callback. Save last driver Status and Fault condiction
 */
void SteerNode::DriverStatusCallback(const roboteq_msgs::Status::ConstPtr &msg){
	status_ = *msg;
}

/**
 * Steering channel feedback callback. Save the last steering feedback
 */
void SteerNode::SteeringFeedbackCallback(const roboteq_msgs::Feedback::ConstPtr &msg){
	feedback_ = *msg;
} 

/**
 * Publishing steering channel drive command message
 */
void SteerNode::PublishSteeringChannel(void){

	// Mount the steering channel drive command message
	steering_.setpoint = setpoint_;
	steering_.mode = mode_;

	pub_steering_channel_.publish(steering_);

}

/**
 * Get Driver State information
 */
int8_t SteerNode::GetDriverState(roboteq_msgs::Status _status){

  int status_timeout = 160; //set the timoeout status in msec. (10Hz topic)

  ros::Time time_now = ros::Time::now();

  int64_t time_now_ms;
  int64_t time_message_ms;

  time_now_ms = (int64_t) time_now.sec * 1000 + (int64_t) time_now.nsec / 1000000;
  time_message_ms = (int64_t) _status.header.stamp.sec * 1000 + (int64_t) _status.header.stamp.nsec / 1000000;

	// Evaluate if there are published messages on Roboteq Driver Status topic
  if(time_now_ms > time_message_ms + status_timeout)
  {
		ROS_DEBUG("Message Timeou");
		ROS_DEBUG_STREAM("Time_elapsed_ms " << time_now_ms-time_message_ms);
    return DriverState::CLOSED;
  }
	// Evaluate status flag
  else if(_status.status & roboteq_msgs::Status::STATUS_MICROBASIC_SCRIPT_RUNNING) 
  {
    if(_status.status & roboteq_msgs::Status::STATUS_SERIAL_MODE)
      return DriverState::RUNNING;
    else
      return DriverState::OPENED;
  }
  else
	{
		ROS_DEBUG("Not identified status");
		return DriverState::CLOSED;
	}
      
}

/**
 *	Read the count ticks of sttering motor encoder and convert to steering wheel angle.
 *  
 *  Must run only after setting up the steering wheel encoder tick count based on CAN
 *  steering wheel information.
 *
 */
void SteerNode::GetWheelAngle(void){

	if(_simulate)
		return;
	
	steering_encoder_ = feedback_.measured_position;
	steering_angle_ = from_motor_rads(from_encoder_ticks(steering_encoder_), _motor_drive_ratio);

}


/**
 * Publish Steering Device State
 */
void SteerNode::PublishSteeringState(void){
	
	// Mount the steering device state message
	steering_state_.driver.state = driver_state_;
	steering_state_.angle = steering_angle_; 
	steering_state_.sensor = 0; 
	steering_state_.header.stamp = ros::Time::now();

	pub_steering_state_.publish(steering_state_);

}

/**
 * Check Driver Fault Information
 */
 bool SteerNode::CheckDriverFault(void){

	 if( status_.fault || (status_.status & BAD_DRIVER_STATUS))
		 return true;
	 else
	 	return false;
 }

/**
 * ROS spin loop
 */
void SteerNode::Spin(void){
	ros::Rate loop_rate(_loop_rate);
  
	// Set initial condiction to driver state
  driver_state_ = DriverState::CLOSED;
	last_driver_state_ = 255;							

	while(ros::ok())
  {
		
  	// Define state machine to monitor de driver state
  	driver_state_ = GetDriverState(status_);

		switch(driver_state_){

			case DriverState::CLOSED:
				if(last_driver_state_ != DriverState::CLOSED)
				{
					last_driver_state_ = DriverState::CLOSED;
					ROS_WARN("Roboteq Driver not connected!");
				}
	  		
        break;

			case DriverState::OPENED:
				if(last_driver_state_ != DriverState::OPENED)
				{
					last_driver_state_ = DriverState::OPENED;
					ROS_WARN("Roboteq Driver Running Script. Trying to set Serial Mode... ");
				}

				// Publish steering channel drive message to set Serial Mode
				PublishSteeringChannel();

        break;
        			
			case DriverState::RUNNING:
				if(last_driver_state_ != DriverState::RUNNING)
				{
					last_driver_state_ = DriverState::RUNNING;
					ROS_WARN("Roboteq Driver in Serial Mode and Runnnig Script");	
				}

				// Check for fault condiction on driver
				if(CheckDriverFault())
				{
					// ROS Erros message
					ROS_ERROR("Roboteq Driver Fault. Unable to control steering wheel");
					// Set setpoint and mode to stopped mode
					setpoint_ = 0;
					mode_ = roboteq_msgs::Command::MODE_STOPPED;

				}
				else
				{
					// Set the desired setpoint for steering motor angular position
					setpoint_ = (steer_limit_travel(command_.angle) * M_PI) * _motor_drive_ratio / 180;
					mode_ = roboteq_msgs::Command::MODE_POSITION_ANGULAR;
				}

				// Publish steering channel drive message
				PublishSteeringChannel();
        
				break;

      default:
        break;
    }  

		// Publish current steering device state
		PublishSteeringState();
			
		// Spin and Sleep
		ros::spinOnce();
		loop_rate.sleep();

	}
}

} // labrom_sena_steering namespace

int main(int argc, char ** argv){

	// Initialize ROS within this node
	ros::init(argc, argv, "steering_node");

	// Steering node
	labrom_sena_steering::SteerNode node;
	node.Spin();
}