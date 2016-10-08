/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Eric Perko, Chad Rockey
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Case Western Reserve University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *0
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <math.h>

#include "robot_driver/robotPOS.h"

float x = 0;


robotPOS::robotPOS(const std::string &port, uint32_t baud_rate, boost::asio::io_service &io)
    : port_(port)
    , baud_rate_(baud_rate)
    , shutting_down_(false)
    , serial_(io, port_)
{
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
}

void robotPOS::poll(nav_msgs::Odometry *odom, sensor_msgs::Imu *imu)
{
    uint8_t start_count = 0;
    bool got_pos = false;
    boost::array<uint8_t, 13> raw_bytes;
    boost::array<uint8_t, 2> out_data;

    int index;
    while(!shutting_down_ && !got_pos) 
    {
        // Wait until first data sync of frame: 0xFA, 0xA0
        boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[start_count], 1));
        if(start_count == 0) 
        {
            if(raw_bytes[start_count] == 0xFA) start_count = 1;
        }
        else if(start_count == 1) 
        {
            // Now that entire start sequence has been found, read in the rest of the message
            got_pos = true;

            // Read in rest of msg
            boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[1], 4));

            uint8_t msgCount = raw_bytes[0];
            uint8_t intakePot = raw_bytes[1];
            uint8_t leftQuad = raw_bytes[2];
            uint8_t rightQuad = raw_bytes[3];

            float theta = 0; // read in
            float radians = theta * (M_PI / 180);

            // for(int i = 0; i < 13; i++)
            // {
            // 	ROS_INFO("%X",raw_bytes[i]);
            // }

            // Pose
            odom->pose.pose.position.x = x;
            odom->pose.pose.position.y = 0;
            odom->pose.pose.position.z = 0;

            odom->pose.pose.orientation.x = cos(radians / 2);
            odom->pose.pose.orientation.y = 0;
            odom->pose.pose.orientation.z = 0;
            odom->pose.pose.orientation.w = sin(radians / 2);

            odom->pose.covariance = ODOM_POSE_COV_MAT;

            // Twist
            odom->twist.twist.linear.x = 0.000125 * (ros::Time::now() - prevTime).toSec() / 1000;
            odom->twist.twist.linear.y = 0;
            odom->twist.twist.linear.z = 0;

            odom->twist.twist.angular.x = 0;
            odom->twist.twist.angular.y = 0;
            odom->twist.twist.angular.z = 0;

            odom->twist.covariance = ODOM_TWIST_COV_MAT;
            
            out_data[0] = 42;
            out_data[1] = 24;
            
            boost::asio::write(serial_,  boost::asio::buffer(&out_data[0], 2));

            // TODO: Fill imu message

            // 	pos->pose.position.x = x;
            // 	pos->pose.position.y = 0;
            // 	pos->pose.position.z = 0;

            // 	pos->pose.orientation.x =  cos(radians/2);
            // 	pos->pose.orientation.y =  0;
            // 	pos->pose.orientation.z =  0;
            // 	pos->pose.orientation.w =  sin(radians/2);

            x += 0.000125;
        }
    }
}
