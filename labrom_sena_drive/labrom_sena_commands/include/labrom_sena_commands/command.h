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

#ifndef COMMAND_H_
#define COMMAND_H_

// ROS libraries
#include <ros/ros.h>

// Ackermann message libraries
#include <ackermann_msgs/AckermannDriveStamped.h>

// Labrom_sena messages libraries
#include <labrom_sena_msgs/SteeringCommand.h>


// Top level namespace
namespace labrom_sena_commands{

// Interface to steering control
class CmdNode{
    public:
        //! Constructor
        CmdNode(void);
        //! Destrutor
        ~CmdNode(void);
        //! Ackermann callback
        void AckermannCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &msg);
        //! Publish SteeringCommand message
        void PublishSteeringCmd(void);
        //! Spin Loop
        void Spin(void);

    private:
        ros::NodeHandle nh_;                      //!< ROS node handle
        ros::NodeHandle pnh_;                     //!< ROS node handle   
        ros::Subscriber sub_ackermann_;           //!< Ackermann subscriber
        ros::Publisher pub_steering_cmd_;         //!< Steering cmd publisher

        ackermann_msgs::AckermannDriveStamped ackermann_;   //!< received ackermann message
        labrom_sena_msgs::SteeringCommand steering_;        //!< steering message to be sent

        //params  
        int _ackermann_steering_ratio;               //!< Ackermann angle to steering wheel ratio
        int _loop_rate;                              //!< ROS loop rate

};
} // labrom_sena_commands namespace
#endif