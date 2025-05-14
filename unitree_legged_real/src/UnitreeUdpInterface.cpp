#include <unitree_legged_real/UnitreeUdpInterface.hpp>
#include <unitree_legged_real/convert.h>

using namespace UNITREE_LEGGED_SDK;

void UnitreeUdpRosInterface::highStateToFeetForces(const HighState& state,
                                                   const ros::Time& stamp,
                                                   unitree_legged_msgs::QuadrupedForceTorqueSensors& feet_forces_msg) {
    feet_forces_msg.header.stamp = stamp;

    // Unitree ordering is: RF, LF, RH, LH
    // or, using their definition: FR, FL, RR, RL
    feet_forces_msg.rf.force.z = state.footForce[0];
    feet_forces_msg.lf.force.z = state.footForce[1];
    feet_forces_msg.rh.force.z = state.footForce[2];
    feet_forces_msg.lh.force.z = state.footForce[3];
}

void UnitreeUdpRosInterface::highStateToTwistMsg(const HighState& state,
                                                 const ros::Time& stamp,
                                                 geometry_msgs::TwistStamped& twist)  {
    twist.header.stamp = stamp;

    // linear velocity, coming from internal odometry of the robot?
    twist.twist.linear.x = state.velocity[0];
    twist.twist.linear.y = state.velocity[1];
    twist.twist.linear.z = state.velocity[2];

    // take angular velocity from the IMU for now
    twist.twist.angular.x = state.imu.gyroscope[0];
    twist.twist.angular.y = state.imu.gyroscope[1];
    twist.twist.angular.z = state.imu.gyroscope[2];


}

void UnitreeUdpRosInterface::highStateToPoseMsg(const HighState& state,
                                                const ros::Time& stamp,
                                                geometry_msgs::PoseStamped& pose)  {

    pose.header.stamp = stamp;

    // the high state seems to have a position vector coming from the internal
    // odometry of the robot
    pose.pose.position.x = state.position[0];
    pose.pose.position.y = state.position[1];
    pose.pose.position.z = state.position[2];

    // take orientation from the imu quaternion, it seems not available elsewhere
    pose.pose.orientation.w = static_cast<double>(state.imu.quaternion[0]);
    pose.pose.orientation.x = static_cast<double>(state.imu.quaternion[1]);
    pose.pose.orientation.y = static_cast<double>(state.imu.quaternion[2]);
    pose.pose.orientation.z = static_cast<double>(state.imu.quaternion[3]);

}

void UnitreeUdpRosInterface::jointStateToRosMsg(const LowState &state,
                                                const ros::Time& stamp,
                        sensor_msgs::JointState& joint_state_msg)  {

    joint_state_msg.header.stamp = stamp;
    // assuming there are 12 actuators and the ordering is known
    // and the joint state message has already 12 elements
    for(size_t i = 0; i < 12; i++){
        joint_state_msg.position[i] = state.motorState[i].q;
        joint_state_msg.velocity[i] = state.motorState[i].dq;
        joint_state_msg.effort[i] = state.motorState[i].tauEst;
    }
}


void UnitreeUdpRosInterface::imuToRosMsg(const IMU &imu, const ros::Time &stamp,
                 sensor_msgs::Imu &imu_msg) {
  imu_msg.header.stamp = stamp;
  //imu_msg.header.frame_id = frame_id; // assume the frame id is given
  imu_msg.orientation.w = static_cast<double>(imu.quaternion[0]);
  imu_msg.orientation.x = static_cast<double>(imu.quaternion[1]);
  imu_msg.orientation.y = static_cast<double>(imu.quaternion[2]);
  imu_msg.orientation.z = static_cast<double>(imu.quaternion[3]);
  imu_msg.angular_velocity.x = static_cast<double>(imu.gyroscope[0]);
  imu_msg.angular_velocity.y = static_cast<double>(imu.gyroscope[1]);
  imu_msg.angular_velocity.z = static_cast<double>(imu.gyroscope[2]);
  imu_msg.linear_acceleration.x = static_cast<double>(imu.accelerometer[0]);
  imu_msg.linear_acceleration.y = static_cast<double>(imu.accelerometer[1]);
  imu_msg.linear_acceleration.z =
      -static_cast<double>(imu.accelerometer[2]); // Note the negative sign
}

UnitreeUdpRosInterface::UnitreeUdpRosInterface(ros::NodeHandle &nh)
    : low_udp(UNITREE_LEGGED_SDK::LOWLEVEL),
      high_udp(UNITREE_LEGGED_SDK::HIGHLEVEL), nh_(nh){


  // initialize all the publishers
  joint_state_pub = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
  pub_high = nh_.advertise<unitree_legged_msgs::HighStateStamped>("/aliengo_ros/high_state",10);
  pub_low  = nh_.advertise<unitree_legged_msgs::LowStateStamped>("/aliengo_ros/low_state",10);
  imu_pub  = nh_.advertise<sensor_msgs::Imu>("/aliengo_ros/imu",10);
  pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("/aliengo_ros/pose",10);
  twist_pub = nh_.advertise<geometry_msgs::TwistStamped>("/aliengo_ros/twist",10);
  feet_forces_pub = nh_.advertise<unitree_legged_msgs::QuadrupedForceTorqueSensors>("/aliengo_ros/feet_forces",10);

  // prepare common fields for messages
  imu_msg.header.frame_id = "imu_link";

  // those are the standard names used by Unitree ... not great, I know
  joint_state_msg.name = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                          "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                          "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
                          "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};

  // make all fields of the appropriate size
  joint_state_msg.position = std::vector<double>(12,0);
  joint_state_msg.velocity = std::vector<double>(12,0);
  joint_state_msg.effort = std::vector<double>(12,0);
}

void UnitreeUdpRosInterface::lowUdpRecv() {
    low_udp.Recv();
}

void UnitreeUdpRosInterface::lowUdpGetRecv() {

  // best we can do is to take the time now, the sdk doesn't provide one
  stamp = ros::Time::now();
  low_udp.GetRecv(low_state);

  // using SDK converter
  low_state_msg.state = state2rosMsg(low_state);
  low_state_msg.header.stamp = stamp;

  pub_low.publish(low_state_msg);

  jointStateToRosMsg(low_state,stamp,joint_state_msg);
  joint_state_pub.publish(joint_state_msg);

}

void UnitreeUdpRosInterface::highUdpRecv() {
  int r = high_udp.Recv();
  // best we can do is to take the time now, the sdk doesn't provide one
  stamp = ros::Time::now();
  high_udp.GetRecv(high_state);
  std::cerr << "Received: " << r << std::endl;

  high_state_msg.state = state2rosMsg(high_state);
  high_state_msg.header.stamp = stamp;
  pub_high.publish(high_state_msg);

  imuToRosMsg(high_state.imu, stamp, imu_msg);
  imu_pub.publish(imu_msg);

  highStateToPoseMsg(high_state, stamp, pose_msg);
  pose_pub.publish(pose_msg);

  highStateToTwistMsg(high_state, stamp, twist_msg);
  twist_pub.publish(twist_msg);

  highStateToFeetForces(high_state, stamp, feet_forces_msg);
  feet_forces_pub.publish(feet_forces_msg);
}
