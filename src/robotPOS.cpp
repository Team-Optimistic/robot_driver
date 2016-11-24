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

#include <cmath>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <iostream>
#include <tf/transform_broadcaster.h>

#include "robot_driver/robotPOS.h"

robotPOS::robotPOS(const std::string &port, uint32_t baud_rate, boost::asio::io_service &io, int csChannel, long speed):
port_(port),
baud_rate_(baud_rate),
serial_(io, port_),
imu_(csChannel, speed)
{
  serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
  spcPub = n.advertise<std_msgs::Empty>("spcRequest", 1000);
  mpcPub = n.advertise<sensor_msgs::PointCloud2>("pickedUpObjects", 1000);

  // Init imu
  std::cout << "IMU INIT" << std::endl;
  std::cout << imu_.init(1, BITS_DLPF_CFG_20HZ) << std::endl;

  usleep(100000);
  //usleep(100000);

  std::cout << "gyro scale = " << std::dec<< imu_.set_gyro_scale(BITS_FS_500DPS) << std::endl;

  //half second wait. Function breaks with 1 million
  usleep(500000);
  //usleep(500000);

  std::cout << "accel scale = " << std::dec<< imu_.set_acc_scale(BITS_FS_2G) << std::endl;

  usleep(100000);
  usleep(500000);
  //usleep(500000);
  //usleep(500000);
  //usleep(500000);
  //usleep(500000);

  //Sample imu to get bias
  std::cout << "IMU CALIBRATING" << std::endl;

  const int imuSampleCount = 1000;
  for (int i = 0; i < imuSampleCount; i++)
  {
    channel0Bias += imu_.read_acc(0);
    channel1Bias += imu_.read_acc(1);
    channel2RotBias += imu_.read_rot(2);
  }

  channel0Bias /= imuSampleCount;
  channel1Bias /= imuSampleCount;
  channel2RotBias /= imuSampleCount;

  std::cout << "IMU CALIBRATION DONE" << std::endl;

  std::cout << "IMU INIT DONE" << std::endl;
}

/**
* Polls UART and sets its inputs to the latest data
* @param odom Odometry data
* @param imu  IMU data
*/
void robotPOS::poll(nav_msgs::Odometry *odom, sensor_msgs::Imu *imu)
{
  boost::array<uint8_t, 3> flagHolders; //0 = start byte, 1 = msg type, 2 = msg count

  const int start_index = 0, msg_type_index = 1, msg_count_index = 2;

  // Load start byte
  int tempCounter = 10; //Only do 10 reads before exiting as to not block
  do
  {
    boost::asio::read(serial_, boost::asio::buffer(&flagHolders[0], 1));
    tempCounter--;
  } while (flagHolders[start_index] != 0xFA && tempCounter > 0);

  //If we exited because we tried too many times
  if (tempCounter <= 0)
  {
    //Don't set the messages
    ROS_INFO("gave up reading");
    return;
  }

  // Load rest of header
  boost::asio::read(serial_, boost::asio::buffer(&flagHolders[msg_type_index], 2));

  // Verify msg count
  if (!verifyMsgHeader(flagHolders[1], flagHolders[2]))
  {
    std::cout << "Message count invalid (" << unsigned(flagHolders[msg_count_index])
              << ") for type " << unsigned(flagHolders[msg_type_index]) << "."
              << std::endl;
  }

  // Load msg
  //Union for converting 4 bytes of a long from RobotC into a int32_t
  union long2Bytes { int32_t l; int8_t b[4]; };

  //Init data vector with size of message
  std::vector<int8_t> msgData(getMsgLengthForType(flagHolders[1]));
  boost::asio::read(serial_, boost::asio::buffer(msgData));

  static int32_t lastRightQuad = 0, lastLeftQuad = 0;

  static float xPosGlobal = 0, yPosGlobal = 0, thetaGlobal = ROBOT_STARTING_THETA;

  // Parse msg
  switch (flagHolders[1])
  {
    //STD msg means the robot is telling us its current sensor values
    case std_msg_type:
    {
      long2Bytes quads;

      //Read in left quads from 4 byte union
      for (int i = 0; i < 4; i++)
      {
        quads.b[i] = msgData[i + 1];
      }
      const int32_t leftQuad = quads.l;

      //Read in right quads from 4 byte union
      for (int i = 0; i < 4; i++)
      {
        quads.b[i] = msgData[i + 5];
      }
      const int32_t rightQuad = quads.l;

      //Read in dt
      const int8_t dt = msgData[9];

      // Twist
      const int32_t rightDelta = (rightQuad - lastRightQuad),
                    leftDelta = (leftQuad - lastLeftQuad);

      lastRightQuad = rightQuad;
      lastLeftQuad = leftQuad;

      const float avg = (rightDelta + leftDelta) / 2.0,
                  dif = (rightDelta - leftDelta) / 2.0;

      const float dist = (avg * straightConversion) / 1000.0, //robots coordinate frame
                  dtheta = dif * thetaConversion;
                  ROS_INFO("dif: %1.2f", dif);
                  ROS_INFO("dtheta: %1.2f", dtheta);

      const float theta = thetaGlobal + dtheta;

      const float dx = cos(theta) * dist, //world coordinate frame
                  dy = sin(theta) * dist;

      const float vx = dx / dt,
                  vy = dy / dt,
                  vtheta = dtheta / dt;
                  ROS_INFO("vtheta: %1.2f", vtheta);
                  ROS_INFO("dt: %d", dt);

      odom->twist.twist.linear.x = vx;
      odom->twist.twist.linear.y = vy;
      odom->twist.twist.linear.z = 0;

      odom->twist.twist.angular.x = 0;
      odom->twist.twist.angular.y = 0;
      thetaGlobal += vtheta;
      odom->twist.twist.angular.z = vtheta;

      // odom->twist.covariance = ODOM_TWIST_COV_MAT;

      // Pose
      xPosGlobal += dx;
      yPosGlobal += dy;
      odom->pose.pose.position.x = xPosGlobal;
      odom->pose.pose.position.y = yPosGlobal;
      odom->pose.pose.position.z = 0;

      odom->pose.pose.orientation = tf::createQuaternionMsgFromYaw(thetaGlobal);

      // odom->pose.covariance = ODOM_POSE_COV_MAT;

      //ROS_INFO("quat: z: %1.2f, w: %1.2f", odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
      //ROS_INFO("pos x: %1.2f, pos y: %1.2f, theta: %1.2f", odom->pose.pose.position.x, odom->pose.pose.position.y, quatToEuler(odom->pose.pose.orientation));

      break;
    }

    //SPC msg means the robot wants to know whats behind it
    //What's behind gets published from motion_path_creator as a regular MPC msg
    case spc_msg_type:
    {
      //Tell motion_path_creator to tell the cortex which object to get
      spcPub.publish(std_msgs::Empty());
      break;
    }

    //MPC msg means the robot is telling us it has scored its last objects
    case mpc_msg_type:
    {
      //Publish the objects that got picked up
      sensor_msgs::PointCloud2 out;
      sensor_msgs::convertPointCloudToPointCloud2(cloud, out);
      mpcPub.publish(out);

      //Set flag
      didPickUpObjects = true;
      break;
    }

    default:
    {
      break;
    }
  }

  // Fill imu message
  const float dpsToRps = 0.01745;
  imu->angular_velocity.x = 0; //imu_.read_rot(0) * dpsToRps;
  imu->angular_velocity.y = 0; //imu_.read_rot(1) * dpsToRps;
  imu->angular_velocity.z = (imu_.read_rot(2) - channel2RotBias) * dpsToRps;
  //imu->angular_velocity_covariance = emptyIMUCov;

  const float gravity = 9.80665;
  imu->linear_acceleration.y = (imu_.read_acc(0) - channel0Bias) * gravity;
  imu->linear_acceleration.x = (imu_.read_acc(1) - channel1Bias) * gravity;
  imu->linear_acceleration.z =  gravity; //imu_.read_acc(2) * gravity;
  //imu->linear_acceleration_covariance = emptyIMUCov;
}

/**
* Converts a quaternion to an euler angle (yaw only)
* @param  quat Quaternion
* @return  n   Euler angle (yaw)
*/
inline const float robotPOS::quatToEuler(const geometry_msgs::Quaternion& quat) const
{
  return std::atan2((2 * ((quat.x * quat.w) + (quat.y * quat.z))),
  ((quat.x * quat.x) + (quat.y * quat.y) - (quat.z * quat.z) - (quat.w * quat.w)));
}

/**
* Callback function for sending ekf position estimate to cortex
* STD Msg
*/
void robotPOS::ekf_callback(const nav_msgs::Odometry::ConstPtr& in)
{
  const int msgLength = 3;
  boost::array<uint8_t, msgLength> out;

  out[0] = in->pose.pose.position.x;
  out[1] = in->pose.pose.position.y;
  const geometry_msgs::Quaternion quat = in->pose.pose.orientation;
  out[2] = quatToEuler(quat);

  //Send header
  sendMsgHeader(std_msg_type);

  //Send data
  boost::asio::write(serial_,  boost::asio::buffer(&out[0], msgLength));
}

/**
* Callback function for sending new object positions to cortex
* MPC Msg
*/
void robotPOS::mpc_callback(const sensor_msgs::PointCloud2::ConstPtr& in)
{
  //Only tell the robot to get more objects if it isn't busy
  if (didPickUpObjects)
  {
    sensor_msgs::convertPointCloud2ToPointCloud(*in, cloud);

    const int msgLength = 12;

    std::vector<uint8_t> out(4);
    for (int i = 0; i < out.size(); i++)
    {
      out.push_back(cloud.points[i].x);
      out.push_back(cloud.points[i].y);
      out.push_back(cloud.points[i].z);
    }

    //Send header
    sendMsgHeader(mpc_msg_type);

    //Send data
    boost::asio::write(serial_, boost::asio::buffer(&out[0], msgLength));

    //Set flag
    didPickUpObjects = false;
  }
}

/**
* Returns the length of a given type of message
* @param  type Type of message
* @return      Length of message
*/
inline const uint8_t robotPOS::getMsgLengthForType(const uint8_t type) const
{
  switch (type)
  {
    case std_msg_type:
    return std_msg_length;

    case spc_msg_type:
    return spc_msg_length;

    case mpc_msg_type:
    return mpc_msg_length;

    default:
    return 0;
  }
}

/**
* Sends message header over UART
* @param type Type of message
*/
void robotPOS::sendMsgHeader(const uint8_t type)
{
  //Send start byte
  boost::asio::write(serial_, boost::asio::buffer(&startFlag[0], 1));

  //Send type byte
  boost::asio::write(serial_, boost::asio::buffer(&msgTypes[type - 1], 1));

  //Send count
  msgCounts[type - 1] = msgCounts[type - 1] + 1 >= 255 ? 0 : msgCounts[type - 1] + 1;
  boost::asio::write(serial_, boost::asio::buffer(&msgCounts[type - 1], 1));
}

/**
* Verifies a message header
* @param  type  Message type
* @param  count Message count
* @return       If header is valid
*/
inline const bool robotPOS::verifyMsgHeader(const uint8_t type, const uint8_t count)
{
  if (type > 0 && type < 3)
  {
    if (isFirstMsg)
    {
      msgCounts[type] = count;
      isFirstMsg = false;
      return true;
    }
    else if (count == (msgCounts[type] + 1 >= 0xFF ? 0 : msgCounts[type] + 1))
    {
      msgCounts[type] = count;
      return true;
    }
  }

  return false;
}
