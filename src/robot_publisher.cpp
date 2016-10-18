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
  std::string world_frame("map"), base_link_frame("base_link");

  // priv_nh.param("port", port, std::string("/dev/ttyS0"));
  n.getParam("port", port);
  priv_nh.param("baud_rate", baud_rate, 9600);
  priv_nh.param("frame_id", frame_id, std::string("neato_laser"));

  boost::asio::io_service io;
  tf::Transform transform;

  try
  {
    mpu6000 imu(0,500000);
    std::cout <<"supposedly inited" <<std::endl;
    std::cout <<imu.init(1,BITS_DLPF_CFG_5HZ) <<std::endl;
    usleep(100000);
	std::cout << "who am i = " <<imu.whoami()<<std::endl;
	usleep(100000);
	imu.wakeup();
	std::cout << "gyro scale = " <<std::dec<<imu.set_gyro_scale(BITS_FS_2000DPS) << std::endl;
	usleep(500000);//half second wait. Function breaks with 1 million
	usleep(500000);    
	std::cout << "accel scale = " <<std::dec<<imu.set_acc_scale(BITS_FS_16G) << std::endl;
    usleep(100000);
	usleep(500000);
	usleep(500000);
	usleep(500000);
	usleep(500000);
	usleep(500000);
    /*robotPOS robot(port, baud_rate, io);
    ros::Publisher odomPub = n.advertise<nav_msgs::Odometry>("robot_publisher/odom0", 1000),
                   imuPub = n.advertise<sensor_msgs::Imu>("robot_publisher/imu0", 1000);
    ros::Subscriber ekfSub = n.subscribe<nav_msgs::Odometry>("odometry/filtered", 1000, &robotPOS::publish_callback, &robot);
	*/
	//imu.wakeup();
    while (ros::ok())
    {
      std::cout << "whoami: " << imu.whoami() << std::endl;
	//imu.wakeup();
      std::cout << "Temp: " << imu.read_temp() << ", R0: " << imu.read_rot(0) << ", R1: " << imu.read_rot(1) << ", R2: " << imu.read_rot(2)
		<< ", A0: " << imu.read_acc(0) << ", A1: "  << imu.read_acc(1) << ", A2: " << imu.read_acc(2) << std::endl;
      usleep(500000);
	usleep(500000);
      /*nav_msgs::Odometry odomOut;
      sensor_msgs::Imu imuOut;

      odomOut.header.frame_id = world_frame;
      odomOut.header.stamp = ros::Time::now();
      odomOut.child_frame_id = base_link_frame;

      imuOut.header.frame_id = world_frame;
      imuOut.header.stamp = ros::Time::now();

      robot.poll(&odomOut, &imuOut);
x
      geometry_msgs::Point xyz = odomOut.pose.pose.position;
      geometry_msgs::Quaternion direction = odomOut.pose.pose.orientation;
      transform.setRotation( tf::Quaternion(direction.x, direction.y, direction.z, direction.w) );
      transform.setOrigin( tf::Vector3(xyz.x, xyz.y, xyz.z) );
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/neato_laser"));

      odomPub.publish(odomOut);*/


    }

    //robot.close();
    return 0;
  }
  catch (boost::system::system_error ex)
  {
    ROS_ERROR("Error instantiating robot object. Are you sure you have the correct port and baud rate? Error was %s", ex.what());
    return -1;
  }
}
