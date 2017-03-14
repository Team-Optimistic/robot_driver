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
*
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

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <boost/asio.hpp>
#include <std_msgs/UInt16.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>

#include "robot_driver/robotPOS.h"
#include "robot_driver/MPU6000.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_publisher");
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");
  tf::TransformBroadcaster br;
  std::string port;
  int baud_rate;
  std::string frame_id;

  n.getParam("port", port);
  n.getParam("baud_rate", baud_rate);
  n.getParam("frame_id", frame_id);
  ROS_INFO("Running with port: %s and baud rate: %d", port.c_str(), baud_rate);

  boost::asio::io_service io;
  tf::Transform transform;

  try
  {
    robotPOS robot(port, baud_rate, io, 0, 500000);

    ros::Publisher odomPub = n.advertise<nav_msgs::Odometry>("robot_publisher/odom0", 1000),
                   imuPub = n.advertise<sensor_msgs::Imu>("robot_publisher/imu0", 1000);

    nav_msgs::Odometry odomOut;
    sensor_msgs::Imu imuOut;

    odomOut.header.frame_id = "odom";
    odomOut.child_frame_id = "base_link";
    odomOut.twist.twist.linear.x = 0;
    odomOut.twist.twist.linear.y = 0;
    odomOut.twist.twist.linear.z = 0;
    odomOut.twist.twist.angular.x = 0;
    odomOut.twist.twist.angular.y = 0;
    odomOut.twist.twist.angular.z = 0;
    odomOut.pose.pose.position.x = 0;
    odomOut.pose.pose.position.y = 0;
    odomOut.pose.pose.position.z = 0;
    odomOut.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

    imuOut.header.frame_id = "base_link";
    imuOut.angular_velocity.x = 0;
    imuOut.angular_velocity.y = 0;
    imuOut.angular_velocity.z = 0;
    imuOut.linear_acceleration.y = 0;
    imuOut.linear_acceleration.x = 0;
    imuOut.linear_acceleration.z = 0;

    odomPub.publish(odomOut);
    imuPub.publish(imuOut);

    while (ros::ok())
    {
      odomOut.header.stamp = ros::Time::now();
      imuOut.header.stamp = ros::Time::now();

      robot.poll(&odomOut, &imuOut);

      odomPub.publish(odomOut);
      imuPub.publish(imuOut);

      ros::spinOnce();
    }

    return 0;
  }
  catch (boost::system::system_error ex)
  {
    ROS_ERROR("robot_driver: Error instantiating robot object. Are you sure you have the correct port and baud rate? Error was: %s", ex.what());
    return -1;
  }
}
