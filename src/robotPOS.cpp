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
#include <geometry_msgs/Quaternion.h>

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

/**
 * Polls UART and sets its inputs to the latest data
 * @param odom Odometry data
 * @param imu  IMU data
 */
void robotPOS::poll(nav_msgs::Odometry *odom, sensor_msgs::Imu *imu)
{
/*  boost::array<uint8_t, 3> flagHolders; //0 = msg, 1 = type, 2 = count

  // Load start byte
  do
  {
    boost::asio::read(serial_, boost::asio::buffer(&flagHolders[0], 1));
  } while (flagHolders[0] != 0xFA);

  // Load rest of header
  boost::asio::read(serial_, boost::asio::buffer(&flagHolders[1], 2));

  // Load msg
  boost::array<uint8_t, flagHolders[2]> msgData;
  boost::asio::read(serial_, boost::asio::buffer(&msgData[0], flagHolders[2]));

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
  odom->twist.twist.linear.x = 0.000125 * (ros::Time::now() - prevTime).toSec() / 1000; //B.S. data
  odom->twist.twist.linear.y = 0;
  odom->twist.twist.linear.z = 0;

  odom->twist.twist.angular.x = 0;
  odom->twist.twist.angular.y = 0;
  odom->twist.twist.angular.z = 0;

  odom->twist.covariance = ODOM_TWIST_COV_MAT;

  // TODO: Fill imu message

  // 	pos->pose.position.x = x;
  // 	pos->pose.position.y = 0;
  // 	pos->pose.position.z = 0;

  // 	pos->pose.orientation.x =  cos(radians/2);
  // 	pos->pose.orientation.y =  0;
  // 	pos->pose.orientation.z =  0;
  // 	pos->pose.orientation.w =  sin(radians/2);
*/}

/**
 * Callback function for sending message to cortex
 * @param in Odometry message to send
 */
void robotPOS::publish_callback(const nav_msgs::Odometry::ConstPtr& in)
{
  boost::array<uint8_t, 7> out;

  out[0] = outMsgCount++;
  out[1] = in->pose.pose.position.x;
  out[2] = in->pose.pose.position.y;
  const geometry_msgs::Quaternion quat = in->pose.pose.orientation;
  out[3] = atan2((2 * ((quat.x * quat.w) + (quat.y * quat.z))),
                ((quat.x * quat.x) + (quat.y * quat.y) - (quat.z * quat.z) - (quat.w * quat.w)));
  out[4] = 0;
  out[5] = 0;
  out[6] = 0;

  //Send start byte
  boost::asio::write(serial_, boost::asio::buffer(&startFlag[0], 1));

  //Send data
  boost::asio::write(serial_,  boost::asio::buffer(&out[0], 7));
}
