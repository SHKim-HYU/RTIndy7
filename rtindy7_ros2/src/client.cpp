#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <iomanip>      // std::setprecision
#include <chrono>
#include <iostream>
#include <Poco/Net/DatagramSocket.h>
#include <Poco/Net/SocketAddress.h>
#define JOINTNUM 6

typedef struct UDPSTATE{
	double q[JOINTNUM];
	double dq[JOINTNUM];
	double ddq[JOINTNUM];
	double torque[JOINTNUM];
	double quat[4];
	double pos[4];
	double V[6];
	double dV[6];
    double s_time;
}UDPstate;
typedef struct UDPIndy7Info{
	UDPSTATE act;
	UDPSTATE des;
}UDPIndy7Info;


using namespace Eigen;
using namespace std;
class JointStatePublisherNode : public rclcpp::Node
{
public:
  JointStatePublisherNode()
  : Node("joint_state_publisher")
  {
    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    joint_state_timer_ = create_wall_timer(std::chrono::microseconds(100), std::bind(&JointStatePublisherNode::publishJointState, this));
	
	receiver = Poco::Net::SocketAddress("192.168.0.9", 9911);
  }

private:
  rclcpp::TimerBase::SharedPtr joint_state_timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  UDPIndy7Info data;
  Poco::Net::DatagramSocket client;
  Poco::Net::SocketAddress receiver;
  void publishJointState()
  {
	client.sendTo(&data, sizeof(UDPIndy7Info), receiver);
	client.receiveFrom(&data, sizeof(UDPIndy7Info), receiver);
    auto joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
    joint_state_msg->header.stamp = this->now();
    joint_state_msg->name = {"joint0", "joint1","joint2","joint3","joint4","joint5"};
    joint_state_msg->position = {data.act.q[0],data.act.q[1],data.act.q[2],data.act.q[3],data.act.q[4],data.act.q[5]};
    joint_state_msg->velocity = {data.act.dq[0],data.act.dq[1],data.act.dq[2],data.act.dq[3],data.act.dq[4],data.act.dq[5]};    
    joint_state_msg->effort = {data.act.torque[0],data.act.torque[1],data.act.torque[2],data.act.torque[3],data.act.torque[4],data.act.torque[5]};        
    joint_state_publisher_->publish(*joint_state_msg);
  }
};


int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::WallRate loop_rate(1000);
	auto node = std::make_shared<JointStatePublisherNode>();	

	while (rclcpp::ok()) {
		// Send data and receive the echoed value

                rclcpp::spin_some(node);
		loop_rate.sleep();
	}
	rclcpp::shutdown();
	return 0;
}
