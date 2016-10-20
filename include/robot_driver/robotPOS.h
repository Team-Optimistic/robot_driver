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

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point32.h>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <string>
#include <ros/ros.h>

#include "robot_driver/MPU6000.h"

class robotPOS
{
  public:
    robotPOS(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io, int csChannel, long speed);

    ~robotPOS() {};

    /**
      * Poll the laser to get a new scan. Blocks until a complete new scan is received or close is called.
      * @param scan LaserScan message pointer to fill in with the scan. The caller is responsible for filling in the ROS timestamp and frame_id
      */
    void poll(nav_msgs::Odometry *odom, sensor_msgs::Imu *imu);

    /**
     * Callback function for sending ekf position estimate to cortex
     */
    void ekf_callback(const nav_msgs::Odometry::ConstPtr& in);

    /**
     * Callback function for sending new object position to cortex
     */
    void mpc_callback(const geometry_msgs::Point32::ConstPtr& in);
  private:
    std::string port_; ///< @brief The serial port the driver is attached to
    uint32_t baud_rate_; ///< @brief The baud rate for the serial connection

    mpu6000 imu_;

    static const uint8_t std_msg_type = 1, spc_msg_type = 2, mpc_msg_type = 3;
    static const uint8_t std_msg_length = 3, spc_msg_length = 3, mpc_msg_length = 3;

    static const int msgType_Count = 3;
    const boost::array<uint8_t, msgType_Count> msgTypes = {{std_msg_type, spc_msg_type, mpc_msg_type}};
    boost::array<uint8_t, msgType_Count> msgCounts = {{0, 0, 0}};

    boost::asio::serial_port serial_; // UART port for the Cortex

    ros::Time prevTime; //previous time of last poll

    //Matrix format is x,y,z,rotx,roty,rotz
    const boost::array<float, 36> ODOM_POSE_COV_MAT =
    {{
      0.01, 0,    0,    0,    0,    0,
      0,    0.01, 0,    0,    0,    0,
      0,    0,    0.01, 0,    0,    0,
      0,    0,    0,    0.01, 0,    0,
      0,    0,    0,    0,    0.01, 0,
      0,    0,    0,    0,    0,    0.01
    }}; //Odometry pose covariance matrix

    //Matrix format is x,y,z,rotx,roty,rotz
    const boost::array<float, 36> ODOM_TWIST_COV_MAT =
    {{
      0.01, 0,    0,    0,    0,    0,
      0,    0.01, 0,    0,    0,    0,
      0,    0,    0.01, 0,    0,    0,
      0,    0,    0,    0.01, 0,    0,
      0,    0,    0,    0,    0.01, 0,
      0,    0,    0,    0,    0,    0.01
    }}; //Odometry twist covariance matrix

    ros::NodeHandle n;
    ros::Publisher spcPub, mpcPub;

    //Counter for messages sent to cortex
    uint8_t outMsgCount = 0;

    //Starting flag for sending a message to the cortex
    const boost::array<uint8_t, 1> startFlag  = {{0xFA}};

    /**
     * Returns the length of a given type of message
     * @param  type Type of message
     * @return      Length of message
     */
    inline const uint8_t getMsgLengthForType(const uint8_t type) const;

    /**
     * Sends message header over UART
     * @param type Type of message
     */
    void sendMsgHeader(const uint8_t type);

    /**
     * Verifies a message header
     * @param  count Message count
     * @param  type  Message type
     * @return       If header is valid
     */
    inline const bool verifyMsgHeader(const uint8_t type, const uint8_t count);

    /**
     * Converts a quaternion to an euler angle (yaw only)
     * @param  quat Quaternion
     * @return      Euler angle (yaw)
     */
    inline const float quatToEuler(const geometry_msgs::Quaternion& quat) const;
};
