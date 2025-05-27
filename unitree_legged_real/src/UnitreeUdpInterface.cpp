#include <unitree_legged_real/UnitreeUdpInterface.hpp>
#include <unitree_legged_real/convert.h>

using namespace UNITREE_LEGGED_SDK;

void UnitreeUdpRosInterface::lowStateToFeetForces(const LowState& state,
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
    // default ordering for Unitree robots in HyQ conventions:
    // RF, LF, RH, LH
    // joint ordering per leg is the same as HyQ conventions:
    size_t jid[12] = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};
    joint_state_msg.header.stamp = stamp;
    // assuming the joint state message has already 12 elements allocated
    for(size_t i = 0; i < 12; i++){
        joint_state_msg.position[i] = state.motorState[jid[i]].q;
        joint_state_msg.velocity[i] = state.motorState[jid[i]].dq;
        joint_state_msg.effort[i] = state.motorState[jid[i]].tauEst;
    }
}


void UnitreeUdpRosInterface::imuToRosMsg(const IMU &imu, const ros::Time &stamp,
                 sensor_msgs::Imu &imu_msg)
{
  imu_msg.header.stamp = stamp;
  // assume the frame id is given, so not filling it in
  imu_msg.orientation.w = static_cast<double>(imu.quaternion[0]);
  imu_msg.orientation.x = static_cast<double>(imu.quaternion[1]);
  imu_msg.orientation.y = static_cast<double>(imu.quaternion[2]);
  imu_msg.orientation.z = static_cast<double>(imu.quaternion[3]);
  imu_msg.angular_velocity.x = static_cast<double>(imu.gyroscope[0]);
  imu_msg.angular_velocity.y = static_cast<double>(imu.gyroscope[1]);
  imu_msg.angular_velocity.z = static_cast<double>(imu.gyroscope[2]);
  imu_msg.linear_acceleration.x = static_cast<double>(imu.accelerometer[0]);
  imu_msg.linear_acceleration.y = static_cast<double>(imu.accelerometer[1]);
  imu_msg.linear_acceleration.z = static_cast<double>(imu.accelerometer[2]);
}

UnitreeUdpRosInterface::UnitreeUdpRosInterface(ros::NodeHandle &nh)
    : low_udp(UNITREE_LEGGED_SDK::LOWLEVEL), nh_(nh)
{
  // Send an empty command at start. This is needed for some reason.
  LowCmd cmd = {0};
  low_udp.InitCmdData(cmd);
  low_udp.SetSend(cmd);
  low_udp.Send();
  
  // initialize all the publishers
  joint_state_pub = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);  
  pub_low  = nh_.advertise<unitree_legged_msgs::LowStateStamped>("/aliengo_ros/low_state",10);
  imu_pub  = nh_.advertise<sensor_msgs::Imu>("/aliengo_ros/imu",10);
  feet_forces_pub = nh_.advertise<unitree_legged_msgs::QuadrupedForceTorqueSensors>("/aliengo_ros/feet_forces",10);

  // prepare common fields for messages
  imu_msg.header.frame_id = "imu_link";

  // Use same joint ordering for HyQ/ANYmal, but keep Unitree names.
  //
  joint_state_msg.name = {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                          "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                          "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
                          "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};

  // make all fields of the appropriate size
  joint_state_msg.position = std::vector<double>(12,0);
  joint_state_msg.velocity = std::vector<double>(12,0);
  joint_state_msg.effort = std::vector<double>(12,0);
}

void UnitreeUdpRosInterface::lowUdpRecv() {
    // Not sure whether the separation between getRecv() and Recv() calls
    // in two separate threads is really needed or not, keeping it for now
    // to follow the example provided by the SDK
    low_udp.Recv();  
}

void UnitreeUdpRosInterface::lowUdpGetRecv() {
  // best we can do is to take the time now, the sdk doesn't provide one
  stamp = ros::Time::now();
  low_udp.GetRecv(low_state);

  // using SDK converter for the low state
  low_state_msg.state = state2rosMsg(low_state);
  low_state_msg.header.stamp = stamp;

  // publish an augmented version of the low state message including a header
  // all other messages are derived from this one and could potentially
  // translated somwhere else, but we publish them here for now
  pub_low.publish(low_state_msg);

  // convert and publish commonly available ROS messages: joint states, IMU
  jointStateToRosMsg(low_state,stamp,joint_state_msg);
  joint_state_pub.publish(joint_state_msg);

  imuToRosMsg(low_state.imu, stamp, imu_msg);
  imu_pub.publish(imu_msg);

  // convert and publish the feet forces as custom message identical to
  // pronto_msgs::QuadrupedForceTorqueSensors, but defined inside
  // unitree_legged_msgs. The trick works because the MD5 checksum will be the
  // same, so we don't add a dependency on pronto_msgs
  lowStateToFeetForces(low_state, stamp, feet_forces_msg);
  feet_forces_pub.publish(feet_forces_msg);
}
