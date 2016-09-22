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
#include <pos_driver/robotPOS.h>
#include <std_msgs/UInt16.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pos_publisher");
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");
  tf::TransformBroadcaster br;
  std::string port;
  int baud_rate;
  std::string frame_id; 
  std::string world_frame("world"); 


  priv_nh.param("port", port, std::string("/dev/ttyUSB0"));
  priv_nh.param("baud_rate", baud_rate, 9600);
  priv_nh.param("frame_id", frame_id, std::string("neato_laser"));

  boost::asio::io_service io;
  tf::Transform transform;

  try {
    pos_driver::robotPOS robot(port, baud_rate, io);
    ros::Publisher pos_pub = n.advertise<geometry_msgs::PoseStamped>("pos", 1000);

    while (ros::ok()) {
      geometry_msgs::PoseStamped::Ptr pos(new geometry_msgs::PoseStamped);

      pos->header.frame_id = world_frame;
      pos->header.stamp = ros::Time::now();      

      robot.poll(pos);

      geometry_msgs::Point xyz = pos->pose.position;
      geometry_msgs::Quaternion direction = pos->pose.orientation;
      transform.setRotation( tf::Quaternion(direction.x, direction.y, direction.z, direction.w) );
      transform.setOrigin( tf::Vector3(xyz.x, xyz.y, xyz.z) );
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/neato_laser"));



      pos_pub.publish(pos);

    }
    robot.close();
    return 0;
  } catch (boost::system::system_error ex) {
    ROS_ERROR("Error instantiating robot object. Are you sure you have the correct port and baud rate? Error was %s", ex.what());
    return -1;
  }
}