/**
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ros/ros.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <fcntl.h>

#include <boost/thread.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include "socketcan/RosCanFrame.h"


class CanManager
{
public:
    ros::NodeHandle n;
    ros::Publisher can_pub;
    ros::Subscriber can_sub;

    CanManager()
    {
        n_p_ = ros::NodeHandle("~");

        n_p_.param<std::string>("interface_name",  interface_name_,  "can0");


        can_pub = n.advertise<socketcan::RosCanFrame>("can_to_ros", 100);
        can_sub = n.subscribe("ros_to_can", 100, &CanManager::sendCanFrameToSocketCAN, this);

        /* Create the CAN socket */
        can_socket_ = socket( PF_CAN, SOCK_RAW, CAN_RAW );
        if((can_socket_ < 0))
        {
            ROS_ERROR_STREAM("Socket could not be created. Shutting down.");
            ros::shutdown();
        }

        /* Locate the interface */
        struct ifreq ifr;

        strcpy(ifr.ifr_name, interface_name_.c_str());

        ioctl(can_socket_, SIOCGIFFLAGS, &ifr);


        bool is_up = (ifr.ifr_flags & IFF_UP);
        if(ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
        {
            ROS_ERROR_STREAM("SocketCAN-Device \"" << interface_name_ << "\" not found. Shutting down.");
            ros::shutdown();
        }
        else
        {
            ROS_INFO_STREAM("SocketCAN-Device \"" << interface_name_ << "\" found.");

            if (!(is_up))
            {
                ROS_ERROR_STREAM("Interface \"" << interface_name_ << "\" is not up. Make sure to set the corresponding baudrate (for e.g. 250k:  sudo ip link set " << interface_name_ << " up type can bitrate 250000 ).  Shutting down.");
                ros::shutdown();
            }
            else
            {
                ROS_INFO_STREAM("Interface \"" << interface_name_ << "\" is up.");

                /* Select that CAN interface, and bind the socket to it. */
                struct sockaddr_can addr;
                addr.can_family = AF_CAN;
                addr.can_ifindex = ifr.ifr_ifindex;
                if(bind( can_socket_, (struct sockaddr*)&addr, sizeof(addr) ) < 0)
                {
                    ROS_ERROR_STREAM("Error while binding socket to the interface. Shutting down.");
                    ros::shutdown();
                }
                else
                {
                    ROS_INFO_STREAM("Bound socket to the interface.");
                    recv_thread = new boost::thread(&CanManager::receiveCanFrame, this);
                    ROS_INFO_STREAM("SocketCAN Node is running.");
                }
            }
        }
    }

    ~CanManager()
    {
        close(can_socket_); // close socket
    }

    void sendCanFrameToSocketCAN(const socketcan::RosCanFrame& sub_frame)
    {
        // Create frame for sending
        out_frame_.can_id = sub_frame.can_id;
        /*out_frame_.data[0] = sub_frame.data[0];
        out_frame_.data[1] = sub_frame.data[1];
        out_frame_.data[2] = sub_frame.data[2];
        out_frame_.data[3] = sub_frame.data[3];
        out_frame_.data[4] = sub_frame.data[4];
        out_frame_.data[5] = sub_frame.data[5];
        out_frame_.data[6] = sub_frame.data[6];
        out_frame_.data[7] = sub_frame.data[7];
        out_frame_.can_dlc = sizeof( out_frame_.data);*/

        out_frame_.can_dlc = sub_frame.can_dlc;
        for(int i = 0; i < sub_frame.can_dlc; i++) {
          out_frame_.data[i] = sub_frame.data[i];
        }

        // Send CAN-Frame to socket
        if(int nwrite = write( can_socket_, &out_frame_, sizeof(out_frame_)) < 0)
        {
                ROS_ERROR_STREAM("Sending failed. Please restart the node." << nwrite);

        }
        else
        {
            //ROS_INFO_STREAM("Frame sent to ROS: | ID: " << out_frame_.can_id << " | Length: " << (int)out_frame_.can_dlc << " | Data: " << std::uppercase << std::setfill('0') << std::setw(2) << std::hex << (int)out_frame_.data[0] << " " << std::setfill('0') << std::setw(2) << (int)out_frame_.data[1] << " " << std::setfill('0') << std::setw(2) << (int)out_frame_.data[2] << " " << std::setfill('0') << std::setw(2) << (int)out_frame_.data[3] << " " << std::setfill('0') << std::setw(2) << (int)out_frame_.data[4] << " " << std::setfill('0') << std::setw(2) << (int)out_frame_.data[5] << " " << std::setfill('0') << std::setw(2) << (int)out_frame_.data[6] << " " << std::setfill('0') << std::setw(2) << (int)out_frame_.data[7]);
        }
    }

    void receiveCanFrame()
    {
        /* Continuously read new CAN messages */
        while(ros::ok())
        {
           socketcan::RosCanFrame pub_frame;

           int nread = read( can_socket_, &in_frame_, sizeof(in_frame_));
           if( nread != -1)
           {
               pub_frame.header.frame_id = interface_name_;
               pub_frame.header.stamp = ros::Time::now();
               pub_frame.can_id = in_frame_.can_id;
               pub_frame.can_dlc = in_frame_.can_dlc;
               for(int i=0; i<in_frame_.can_dlc; i++)
               {
                   pub_frame.data[i] = in_frame_.data[i];
               }
               can_pub.publish(pub_frame);

               //ROS_INFO_STREAM("Frame sent to ROS: | ID: " << in_frame_.can_id << " | Length: " << (int)in_frame_.can_dlc << " | Data: " << std::uppercase << std::setfill('0') << std::setw(2) << std::hex << (int)in_frame_.data[0] << " " << std::setfill('0') << std::setw(2) << (int)in_frame_.data[1] << " " << std::setfill('0') << std::setw(2) << (int)in_frame_.data[2] << " " << std::setfill('0') << std::setw(2) << (int)in_frame_.data[3] << " " << std::setfill('0') << std::setw(2) << (int)in_frame_.data[4] << " " << std::setfill('0') << std::setw(2) << (int)in_frame_.data[5] << " " << std::setfill('0') << std::setw(2) << (int)in_frame_.data[6] << " " << std::setfill('0') << std::setw(2) << (int)in_frame_.data[7]);
           }
        }
    }

private:
    ros::NodeHandle n_p_;
    boost::thread* recv_thread;
    std::string interface_name_;
    int can_socket_;
    struct can_frame in_frame_;
    struct can_frame out_frame_;
};


// Main
int main(int argc, char **argv)
{
    ros::init(argc, argv, "socketcan_node");

    CanManager can_manager;

    ros::spin();

    return 0;
}
