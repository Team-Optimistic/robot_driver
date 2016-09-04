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

#include <pos_driver/robotPOS.h>

 namespace pos_driver {
 	robotPOS::robotPOS(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io): port_(port),
 	baud_rate_(baud_rate), shutting_down_(false), serial_(io, port_) {
 		serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
 	}

 	void robotPOS::poll(geometry_msgs::PoseStamped::Ptr pos, geometry_msgs::TransformStamped::Ptr transform){

 		uint8_t start_count = 0;
 		bool got_pos = false;
 		boost::array<uint8_t, 13> raw_bytes;
 		
 		int index;
 		while (!shutting_down_ && !got_pos) {
	// Wait until first data sync of frame: 0xFA, 0xA0
 			boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[start_count],1));
 			if(start_count == 0) {
 				if(raw_bytes[start_count] == 0xFA) {
 					start_count = 1;
 				}
 			} else if(start_count == 1) {				

	    			// Now that entire start sequence has been found, read in the rest of the message
 				got_pos = true;

 				boost::asio::read(serial_,boost::asio::buffer(&raw_bytes[1], 12));

 				//do math

 				transform->transform.translation.x = pos->pose.position.x = 0;
 				transform->transform.translation.y = pos->pose.position.y = 0;
 				transform->transform.translation.z = pos->pose.position.z = 0;

 			}
 			
 		}
 	}

 };
