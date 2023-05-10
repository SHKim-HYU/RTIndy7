#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <iomanip>      // std::setprecision
#include <ModernRobotics.h>
#include <modern_robotics.h>
#include <Indy7.h>
#include <chrono>
#include <iostream>

using namespace Eigen;
using namespace std;


class JointStatePublisherNode : public rclcpp::Node
{
public:
  JointStatePublisherNode()
  : Node("joint_state_publisher")
  {
    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    joint_state_timer_ = create_wall_timer(std::chrono::microseconds(1000), std::bind(&JointStatePublisherNode::publishJointState, this));
    q<<0.1,0.1,0.1,0.1,0.1,0.1;
    q_start = q;
    q_end<<1,1,1,1,1,1;
    g<< 0, 0, -9.8;

  }

private:
  rclcpp::TimerBase::SharedPtr joint_state_timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  Indy7 *indy7 = new Indy7("~/ros2_humble/src/mr_joint_space_control/MR_info.json");
  static unsigned int print_count;
  double t = 0;
  int CONTROL_RATE = 1000;
  int n = 6;
  double Tf = 5;
  double dt = 1.0/CONTROL_RATE;
  JVec q=JVec::Zero();
  JVec q_start =JVec::Zero();
  JVec q_end =JVec::Zero();
	JVec dq=JVec::Zero();
	JVec ddq=JVec::Zero();
	Vector6d Ftip = Vector6d::Zero();
	Vector3d g= Vector3d::Zero();
	Eigen::Matrix<double, 6, 6> Kp =Matrix6d::Identity();
  Eigen::Matrix<double, 6, 6> Kv =Matrix6d::Identity();
  JVec q_desired=JVec::Zero();
  JVec dq_desired=JVec::Zero();
  JVec ddq_desired=JVec::Zero();
  void publishJointState()
  {
    double scale = 5.0;
  for(int i=0; i<n; ++i)
  {
    switch(i)
    {
    case 0:
    case 1:
      Kp(i,i) = 70*scale;
      Kv(i,i) = 55*scale;
      break;
    case 2:
      Kp(i,i) = 40*scale;
      Kv(i,i) = 30*scale;
      break;
    case 3:
    case 4:
      Kp(i,i) = 25*scale;
      Kv(i,i) = 15*scale;
      break;
    case 5:
      Kp(i,i) = 18*scale;
      Kv(i,i) = 3*scale;
      break;
    }
  }    
		JVec taulist = JVec::Zero();
    q_desired = Desired_q(q_start, q_end, t, Tf, 5);
    dq_desired = Desired_dq(q_start, q_end, t, Tf, 5);
    ddq_desired = Desired_ddq(q_start, q_end, t, Tf, 5);
  // INVERSE DYNAMICS 
    JVec taugrav = InverseDynamics(q,dq,ddq,g,Ftip,indy7->Mlist,indy7->Glist,indy7->Slist);
    MassMat Mmat = MassMatrix(q, indy7->Mlist,indy7->Glist,indy7->Slist);
    JVec C =  VelQuadraticForces(q,dq,indy7->Mlist,indy7->Glist,indy7->Slist);
    JVec qddot_ref = ddq_desired+Kv*(dq_desired-dq) + Kp*(q_desired- q);
    JVec tauCTM  =Mmat*qddot_ref + taugrav;
    taulist = tauCTM;
   // FOWRAD DYNAMIC SIMULATION
		JVec ddq = ForwardDynamics(q,dq, taulist,
									g, Ftip, indy7->Mlist,
									indy7->Glist, indy7->Slist);
		EulerStep(q, dq, ddq, dt);

    // PUBLISH
    auto joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
    joint_state_msg->header.stamp = this->now();
    joint_state_msg->name = {"joint0", "joint1","joint2","joint3","joint4","joint5"};
    joint_state_msg->position = {q[0],q[1],q[2],q[3],q[4],q[5]};
    joint_state_publisher_->publish(*joint_state_msg);
    if(++print_count>=CONTROL_RATE/10){
      cout<<"t : "<< t << " - q_err : "<<(q_desired- q).transpose()<<endl;
      print_count = 0;
    }
    t=  t+dt;
  }
};
unsigned int JointStatePublisherNode::print_count = 0;




int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointStatePublisherNode>();
  rclcpp::WallRate loop_rate(1000);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
