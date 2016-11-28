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

#ifndef STEERING_H_
#define STEERING_H_

// ROS libraries
#include <ros/ros.h>

//Labrom_sena libraries
#include <labrom_sena_commands/senasetup.h>

// Labrom_sena messages libraries
#include <labrom_sena_msgs/SteeringCommand.h>
#include <labrom_sena_msgs/DriverState.h>

// Roboteq message libraries
#include <roboteq_msgs/Command.h>
#include <roboteq_msgs/Feedback.h>
#include <roboteq_msgs/Status.h>

// Top level namespace
namespace labrom_sena_steering{

// Interface to Steering Channel
class SteerNode{
    public:
        //! Constructor
        SteerNode(void);
        //! Destructor
        ~SteerNode(void);
        //! Steering callback
        void SteeringCallback(const labrom_sena_msgs::SteeringCommand::ConstPtr &msg);
        //! Driver Status callback
        void DriverStateCallback(const roboteq_msgs::Status::ConstPtr &msg);
        //! Publish Steering Channel Command message
        void PublishSteeringChannel(void);
        //! Get Driver State
        int8_t GetDriverState(roboteq_msgs::Status _state);
        //! Sping
        void Spin(void);
    
    private:
        ros::NodeHandle nh_;                      //!< ROS node handle
        ros::NodeHandle pnh_;                     //!< ROS node handle   
        ros::Subscriber sub_steering_;            //!< Steering subscriber
        ros::Subscriber sub_driver_state_;        //!< Driver Status subscriber
        ros::Publisher pub_steering_channel_;     //!< Steering Channel publisher

        labrom_sena_msgs::SteeringCommand command_;  //!< receiver steering message
        roboteq_msgs::Command steering_;             //!< steering channel message to be sent
        roboteq_msgs::Status state_;                 //!< receiver roboteq driver status message

        typedef labrom_sena_msgs::DriverState DriverState;
        DriverState::_state_type driver_state_;
        DriverState::_state_type last_driver_state_;
        
        //params  
       
        double _motor_drive_ratio;        //!< Transmission ratio from steering motor drive to steering wheel
        int _loop_rate;                   //!< ROS loop rate





};
} // labrom_sena_steering namespace
#endif