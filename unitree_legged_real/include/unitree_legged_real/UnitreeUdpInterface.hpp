// unitree communication sdk
#include <unitree_legged_sdk/comm.h>
#include <unitree_legged_sdk/udp.h>

// ROS msgs
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>

// Unitree ROS msgs
#include <unitree_legged_msgs/HighStateStamped.h>
#include <unitree_legged_msgs/LowStateStamped.h>
// this is actually taken from pronto_msgs
#include <unitree_legged_msgs/QuadrupedForceTorqueSensors.h>

// Other ROS stuff
#include <ros/node_handle.h>
#include <ros/publisher.h>

// standard library
#include <atomic>

class UnitreeUdpRosInterface {
public:
  UNITREE_LEGGED_SDK::UDP low_udp;
  UNITREE_LEGGED_SDK::UDP high_udp;

  // protect those states while they are being written/read
  UNITREE_LEGGED_SDK::HighState high_state;
  UNITREE_LEGGED_SDK::LowState low_state;

  const int LOW_CMD_LENGTH = 610;
  const int LOW_STATE_LENGTH = 771;

public:
  UnitreeUdpRosInterface(ros::NodeHandle &nh);

  void lowUdpRecv();

  void highUdpRecv();

  static void imuToRosMsg(const UNITREE_LEGGED_SDK::IMU &imu,
                          const ros::Time &stamp, sensor_msgs::Imu &imu_msg);

  static void jointStateToRosMsg(const UNITREE_LEGGED_SDK::LowState &state,
                                 const ros::Time &stamp,
                                 sensor_msgs::JointState &joint_state_msg);

  static void highStateToTwistMsg(const UNITREE_LEGGED_SDK::HighState &state,
                                  const ros::Time &stamp,
                                  geometry_msgs::TwistStamped &twist);

  static void highStateToPoseMsg(const UNITREE_LEGGED_SDK::HighState &state,
                                 const ros::Time &stamp,
                                 geometry_msgs::PoseStamped &pose);

  static void highStateToFeetForces(
      const UNITREE_LEGGED_SDK::HighState &state, const ros::Time &stamp,
      unitree_legged_msgs::QuadrupedForceTorqueSensors &feet_forces_msg);

private:
  ros::Publisher pub_high;
  ros::Publisher pub_low;
  ros::Publisher joint_state_pub;
  ros::Publisher imu_pub;
  ros::Publisher odom_pub;
  ros::Publisher pose_pub;
  ros::Publisher twist_pub;
  ros::Publisher feet_forces_pub;

  ros::NodeHandle &nh_;

  sensor_msgs::JointState joint_state_msg;
  sensor_msgs::Imu imu_msg;
  geometry_msgs::PoseStamped pose_msg;
  geometry_msgs::TwistStamped twist_msg;
  ros::Time stamp;
  unitree_legged_msgs::LowStateStamped low_state_msg;
  unitree_legged_msgs::HighStateStamped high_state_msg;

  unitree_legged_msgs::QuadrupedForceTorqueSensors feet_forces_msg;
};
