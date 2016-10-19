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

class robotPOS
{
    public:
        robotPOS(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io);

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

        void mpc_callback(const geometry_msgs::Point32::ConstPtr& in);
    private:
        std::string port_; ///< @brief The serial port the driver is attached to
        uint32_t baud_rate_; ///< @brief The baud rate for the serial connection

        const uint8_t std_msg_type = 1, spc_msg_type = 2, mpc_msg_type = 3;
        const uint8_t std_msg_length = 3, spc_msg_length = 3, mpc_msg_length = 3;
        const boost::array<uint8_t, 2> msgTypes = {{std_msg_type, spc_msg_type, mpc_msg_type}};
        boost::array<uint8_t, 3> msgCounts = {{0, 0, 0}};

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

        //Counter for messages sent to cortex
        uint8_t outMsgCount = 0;

        //Starting flag for sending a message to the cortex
        const boost::array<uint8_t, 1> startFlag  = {{0xFA}};

        inline const uint8_t getMsgLengthForType(const uint8_t type) const
        {
          switch (type)
          {
            case std_msg_type:
              return std_msg_length;

            case spc_msg_type:
              return spc_msg_length;

            default:
              return 0;
          }
        }
};
