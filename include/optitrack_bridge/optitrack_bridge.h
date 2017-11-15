// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <stdint.h>
#include <string>
#include <thread>
#include <mutex>

#include <sys/socket.h>

namespace optitrack_bridge
{

class OptitrackBridge
{
public:
	/* create optitrack bridge with multicast address and port */
	OptitrackBridge();
	~OptitrackBridge();

	enum
	{
		MAX_PACKET_SIZE = 1000000,
		DIAG_QUEUE_SIZE = 64,
		INIT_QUEUE_SIZE = 128
	};

private:
	// compact structure to store pose
	struct PoseF
	{
		float x, y, z;
		float qx, qy, qz, qw;
	} __attribute__((packed));


	void Run();
	void HandlePacket(const void* buffer, int len);
	void PublishPose(const PoseF& pose, int frame_id, double ts);
	void AssignLatency();
	void DiagnosticCallback(const ros::TimerEvent&);

private:
	int sock_;
	volatile int running_;
	std::thread listening_thread_;

	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;
	std::string body_name_;
	std::string body_frame_id_;
	std::string map_frame_id_;
	int fps_;

	ros::Publisher tf_publisher_;
	ros::Publisher pose_publisher_;
	tf2_msgs::TFMessage tf_msg_;
	geometry_msgs::PoseStamped pose_stamped_;

	// diagnostics
	ros::Time time_last_msg_pub_;
	ros::Time max_time_gap_;
	int window_size_, start_pos_;
	ros::Time timestamp_queue_[DIAG_QUEUE_SIZE];

	// init
	std::mutex queue_mutex_;
	int warm_up_size_;
	double time_latencies_[INIT_QUEUE_SIZE];
	double latency_;

	ros::Publisher diagnostics_pub_;
	ros::Timer diagnostics_timer;
	diagnostic_msgs::DiagnosticArray diag_array_msg_;
};

}
