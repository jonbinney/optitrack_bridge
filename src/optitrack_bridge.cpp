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
#include <ros/ros.h>
#include <ros/console.h>

#include <cstdio>
#include <cstdlib>
#include <thread>
#include <cmath>
#include <mutex>

#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/select.h>

#include <optitrack_bridge/optitrack_bridge.h>

#include <Eigen/Dense>

namespace optitrack_bridge
{

namespace
{
template<typename T>
inline T Read(const char* &ptr)
{
	const T* tptr = reinterpret_cast<const T*>(ptr);
	ptr += sizeof(T);
	return *tptr;
}

// NOTE: treat float and double specially b/c some arm processor doesn't 
// support unaligned memory access for float/double/int64

template<>
inline float Read(const char* &ptr)
{
	const uint32_t* tptr = reinterpret_cast<const uint32_t*>(ptr);
	uint32_t val = *tptr;
	ptr += sizeof(float);
	return *((float*)&tptr);
}

template<>
inline double Read(const char* &ptr)
{
	double x;
	memcpy(&x, ptr, sizeof(double));
	ptr += sizeof(double);
	return x;
}

inline void Skip(const char* &ptr, int len)
{
	ptr += len;
}

template<typename T>
inline T sqr(T x)
{
	return x * x;
}

// packet structure
struct Point
{
	float x, y, z;
} __attribute__((packed));

struct MarkerSet
{
	int32_t num_markers;
	Point markers[0];
} __attribute__((packed));

struct RigidBody
{
	int32_t id;
	float x, y, z;
	float qx, qy, qz, qw;

//	int32_t num_markers;
//	Point markers[0];
} __attribute__((packed));

struct LabelledMarker
{
	int32_t id;
	float x, y, z;
	float size;
	int16_t param;
} __attribute__((packed));

struct PacketHeader
{
	int16_t message_id;   // 7-mocap frame, 5-data description
	int16_t n_bytes;
	char payload[0];
} __attribute__((packed));

}

Eigen::Isometry3d initial_pose;
bool initialized = false;

void OptitrackBridge::AssignLatency()
{
	double avg = 0.0;
	for (int i = 0; i < INIT_QUEUE_SIZE; i++)
		avg += time_latencies_[i];
	avg /= INIT_QUEUE_SIZE;
	latency_ = avg;
	ROS_INFO("initialization finished. Latency is set to %fs", latency_);
}

void OptitrackBridge::PublishPose(const PoseF& pose, int frame_id, double ts)
{
	ros::Time rectified_timestamp;

	// put into queue
	{
		std::lock_guard<std::mutex> lock(queue_mutex_);

		// init phase
		if (warm_up_size_ < INIT_QUEUE_SIZE)
		{
			time_latencies_[warm_up_size_++] = ros::Time::now().toSec() - ts;
			if (warm_up_size_ == INIT_QUEUE_SIZE)
				AssignLatency();
			return;
		}

		rectified_timestamp = ros::Time::now();//ts + latency_);
		if (window_size_ < DIAG_QUEUE_SIZE)
		{
			timestamp_queue_[window_size_++] = rectified_timestamp;
		}
		else
		{
			timestamp_queue_[start_pos_] = rectified_timestamp;
			start_pos_ = (start_pos_ +1) % DIAG_QUEUE_SIZE;
		}
	}


  PoseF ppose = pose;
/*
  if (initialized == false)
  {
    initial_pose = Eigen::Translation3d(pose.x, pose.y, pose.z) *
                   Eigen::Quaterniond(pose.qw, pose.qx, pose.qy, pose.qz);
    initialized = true;
    return;
  }
  else
  {
    Eigen::Isometry3d current_pose = Eigen::Translation3d(pose.x, pose.y, pose.z) *
                                     Eigen::Quaterniond(pose.qw, pose.qx, pose.qy, pose.qz);
    
    Eigen::Isometry3d adjusted_pose = initial_pose.inverse() * current_pose;

    ppose.x = adjusted_pose.translation()[0];
    ppose.y = adjusted_pose.translation()[1];
    ppose.z = adjusted_pose.translation()[2];

    Eigen::Quaterniond quat = (Eigen::Quaterniond)adjusted_pose.linear();
    ppose.qx = quat.x();
    ppose.qy = quat.y();
    ppose.qz = quat.z();
    ppose.qw = quat.w();
  }
*/

	// advertise tf
	tf_msg_.transforms[0].header.stamp = rectified_timestamp;
	tf_msg_.transforms[0].transform.translation.x = ppose.x;
	tf_msg_.transforms[0].transform.translation.y = ppose.y;
	tf_msg_.transforms[0].transform.translation.z = ppose.z;
	tf_msg_.transforms[0].transform.rotation.x = ppose.qx;
	tf_msg_.transforms[0].transform.rotation.y = ppose.qy;
	tf_msg_.transforms[0].transform.rotation.z = ppose.qz;
	tf_msg_.transforms[0].transform.rotation.w = ppose.qw;
	tf_publisher_.publish(tf_msg_);

	// advertise pose
	pose_stamped_.pose.position.x = ppose.x;
	pose_stamped_.pose.position.y = ppose.y;
	pose_stamped_.pose.position.z = ppose.z;
	pose_stamped_.pose.orientation.x = ppose.qx;
	pose_stamped_.pose.orientation.y = ppose.qy;
	pose_stamped_.pose.orientation.z = ppose.qz;
	pose_stamped_.pose.orientation.w = ppose.qw;
	pose_stamped_.header.stamp = rectified_timestamp;

	pose_publisher_.publish(pose_stamped_);



}

void OptitrackBridge::HandlePacket(const void* buffer, int len)
{
	const PacketHeader* packet_header = (const PacketHeader*)buffer;
	if (packet_header->message_id == 7)
	{
		PoseF body_pose;
		bool valid = false;

		const char* buf_ptr = (const char*)&packet_header->payload;
		int frame_id = Read<int32_t>(buf_ptr);

//		ROS_INFO("Frame ID = %d", frame_id);

		// parse markersets
		const char* marker_ids[16]; // name of markers
		int num_markersets = Read<int32_t>(buf_ptr);

		for (int i = 0; i < num_markersets; i++)
		{
			const char* marker_name = buf_ptr;
			int marker_name_len = strlen(marker_name);
			Skip(buf_ptr, marker_name_len +1);

			marker_ids[i] = marker_name;

			const MarkerSet* markerset_data =
					reinterpret_cast<const MarkerSet*>(buf_ptr);

			Skip(buf_ptr, sizeof(MarkerSet) +
					sizeof(Point) * markerset_data->num_markers);
		}

		// parse unidentified markers
		int num_unidentified_markers = Read<int32_t>(buf_ptr);
		const Point* unidentified_markers =
				reinterpret_cast<const Point*>(buf_ptr);
		Skip(buf_ptr, sizeof(Point) * num_unidentified_markers);

		// parse rigid bodies
		int num_rigid_bodies = Read<int32_t>(buf_ptr);

		for (int i = 0; i < num_rigid_bodies; i++)
		{
			const RigidBody* rigid_body =
					reinterpret_cast<const RigidBody*>(buf_ptr);

			const char* rigid_body_name = marker_ids[i];

			/*if (rigid_body->num_markers > 100 || rigid_body->num_markers < 0)
			{
			     ROS_ERROR("Bad number of markers, probably a parse error");
                 ROS_ERROR_STREAM(rigid_body_name << " " << rigid_body->num_markers);
			     return;
			}*/

			Skip(buf_ptr, sizeof(RigidBody));
			// skip marker positions
			//Skip(buf_ptr, sizeof(Point) * rigid_body->num_markers);
			// skip marker ids
			//Skip(buf_ptr, sizeof(int32_t) * rigid_body->num_markers);
			// skip marker sizes
			//Skip(buf_ptr, sizeof(float) * rigid_body->num_markers);

			float mean_marker_error = Read<float>(buf_ptr);
			int validity = Read<int16_t>(buf_ptr);

			if (validity == 0)
				continue;

			if (strcmp(rigid_body_name, body_name_.c_str()) == 0)
			{
				// do memcpy instead of assignment b/c
				// arm doesn't support unaligned float point access
				memcpy(&body_pose.x, &rigid_body->x, sizeof(body_pose));

				// normalize quaternion
				float qlen = std::sqrt(
						sqr(body_pose.qx) +
						sqr(body_pose.qy) +
						sqr(body_pose.qz) +
						sqr(body_pose.qw));
				body_pose.qx /= qlen;
				body_pose.qy /= qlen;
				body_pose.qz /= qlen;
				body_pose.qw /= qlen;

				valid = true;
			}

			ROS_INFO("Pose %s, (%f,%f,%f), (%f,%f,%f,%f)",
					rigid_body_name,
					rigid_body->x, rigid_body->y, rigid_body->z,
					rigid_body->qx, rigid_body->qy, rigid_body->qz,rigid_body->qw);
		}

		// skip skeletons
		int num_skeletons = Read<int32_t>(buf_ptr);
		if (num_skeletons != 0)
		{
			ROS_ERROR("skeleton is not supported in the republisher.");
			return;
		}

		// parse labelled markers
		int num_labelled_markers = Read<int32_t>(buf_ptr);
		Skip(buf_ptr, sizeof(LabelledMarker) * num_labelled_markers);

		float latency = Read<float>(buf_ptr);

		unsigned timecode = Read<uint32_t>(buf_ptr);
		unsigned timecodesub = Read<uint32_t>(buf_ptr);

		double timestamp = Read<double>(buf_ptr);

		int param = Read<int16_t>(buf_ptr);
		int eod = Read<int32_t>(buf_ptr);

		if (valid)
			PublishPose(body_pose, frame_id, timestamp);
	}
}



OptitrackBridge::OptitrackBridge() :
	nh_private_("~")
{
	// parameters
	std::string multicast_addr_str =
			nh_private_.param<std::string>("address", "239.255.42.99");
	int port = nh_private_.param<int>("port", 1511);
	body_name_ = nh_private_.param<std::string>("body_name", "base_link");
	body_frame_id_ = nh_private_.param<std::string>("body_frame_id", body_name_);
	map_frame_id_ = nh_private_.param<std::string>("map_frame_id", "map");
	fps_ = nh_private_.param<int>("fps", 120);


	ROS_INFO("Subscribing from UDP multicast %s:%d",
		multicast_addr_str.c_str(), port);

	// subscribe to multicast socket
	if ((sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP)) < 0)
	{
		ROS_FATAL("cannot create socket.");
		exit(EXIT_FAILURE);
	}

	int sock_option = 1;
	unsigned sock_option_len;
	if (setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR,
		&sock_option, sizeof(sock_option)) != 0)
	{
		ROS_FATAL("cannot set socket reuseaddr option.");
		exit(EXIT_FAILURE);
	}

	sockaddr_in multicast_addr;
	memset(&multicast_addr, 0, sizeof(multicast_addr));
	multicast_addr.sin_family = AF_INET;
	multicast_addr.sin_addr.s_addr = inet_addr(multicast_addr_str.c_str());
	multicast_addr.sin_port = htons(port);
	if (bind(sock_, (sockaddr*)&multicast_addr, sizeof(multicast_addr)) != 0)
	{
		ROS_FATAL("cannot bind to multicast address.");
		exit(EXIT_FAILURE);
	}

	// join multicast group
	ip_mreq add_member_req;
	add_member_req.imr_multiaddr = multicast_addr.sin_addr;
	add_member_req.imr_interface.s_addr = INADDR_ANY;
	if (setsockopt(sock_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &add_member_req, sizeof(add_member_req)) != 0)
	{
		ROS_FATAL("cannot add to multicast group.");
		exit(EXIT_FAILURE);
	}

	// set buffer size
	sock_option = 0x100000;
	setsockopt(sock_, SOL_SOCKET, SO_RCVBUF, &sock_option, sizeof(sock_option));
	getsockopt(sock_, SOL_SOCKET, SO_RCVBUF, &sock_option, &sock_option_len);
	if (sock_option != 0x100000)
	{
		ROS_WARN("cannot set receive buffer size.");
	}

	// FIXME: set nonblocking

	// init messages
	window_size_ = 0;
	start_pos_ = 0;
	warm_up_size_ = 0;
	latency_ = 0.0;

	pose_stamped_.header.seq = 0;
	pose_stamped_.header.frame_id = map_frame_id_;

	tf_msg_.transforms.resize(1);
	tf_msg_.transforms[0].header.seq = 0;
	tf_msg_.transforms[0].header.frame_id = map_frame_id_;
	tf_msg_.transforms[0].child_frame_id = body_frame_id_;

	diag_array_msg_.status.resize(1);
	diag_array_msg_.status[0].name = "Optitrack external pose";
	diag_array_msg_.status[0].hardware_id = 1;

	// init publishers
	pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 8);
	tf_publisher_ = nh_.advertise<tf2_msgs::TFMessage>("tf", 8);
	diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 3);

	// run
	running_ = true;
    listening_thread_ = std::thread(&OptitrackBridge::Run, this);

    diagnostics_timer = nh_.createTimer(ros::Duration(0.5), &OptitrackBridge::DiagnosticCallback, this);
}

void OptitrackBridge::DiagnosticCallback(const ros::TimerEvent&)
{
	double real_fps;
	{
		std::lock_guard<std::mutex> lock(queue_mutex_);

		if (warm_up_size_ < INIT_QUEUE_SIZE)
			real_fps = -1.0;
		else
			real_fps = window_size_ / (ros::Time::now().toSec() - timestamp_queue_[start_pos_].toSec());
	}

	if (real_fps < -0.1)
	{
		char buf[128];
		sprintf(buf, "Optitrack not initialized, %d/%d pose messages needed", warm_up_size_, INIT_QUEUE_SIZE);
		diag_array_msg_.status[0].message = buf;
		diag_array_msg_.status[0].level = diagnostic_msgs::DiagnosticStatus::ERROR;
	}
	else if (real_fps < fps_ * 0.8)
	{
		char buf[128];
		sprintf(buf, "Optitrack publishing too slow, fps = %.2f, expected fps = %d", real_fps, fps_);
		diag_array_msg_.status[0].message = buf;
		diag_array_msg_.status[0].level = diagnostic_msgs::DiagnosticStatus::ERROR;

	}
	else
	{
		char buf[128];
		sprintf(buf, "Optitrack publishing works well, fps = %.2f", real_fps);
		diag_array_msg_.status[0].message = buf;
		diag_array_msg_.status[0].level = diagnostic_msgs::DiagnosticStatus::OK;
	}

	diag_array_msg_.header.stamp = ros::Time::now();
	diagnostics_pub_.publish(diag_array_msg_);

}

OptitrackBridge::~OptitrackBridge()
{
	running_ = false;
	if (listening_thread_.joinable())
	{
		ROS_INFO("waiting for pose listening thread to exit...");
		listening_thread_.join();
		ROS_INFO("pose listening thread exited.");
	}
}

void OptitrackBridge::Run()
{
	char* packet_buffer = new char[MAX_PACKET_SIZE];

    fd_set rfds;
    struct timeval tv;
    int retval;

    ROS_INFO("Listening thread created.");

    while (running_)
    {
		FD_ZERO(&rfds);
		FD_SET(sock_, &rfds);

	    // wait for 1 seconds
	    tv.tv_sec = 1;
	    tv.tv_usec = 0;

		retval = select(sock_ +1, &rfds, NULL, NULL, &tv);

		if (retval == -1)
			ROS_ERROR("select error.");
		else if (retval > 0)
		{
			if (FD_ISSET(sock_, &rfds))
			{
				int len = recv(sock_, packet_buffer, MAX_PACKET_SIZE, 0);
				if (len <= 0)
					ROS_ERROR("error recieving udp packet.");
				else
					HandlePacket(packet_buffer, len);
			}
			else
				ROS_ERROR("select error.");
		}
		else
			ROS_WARN("No optitrack pose data within 1 second.");
    }

	delete packet_buffer;
}

}

