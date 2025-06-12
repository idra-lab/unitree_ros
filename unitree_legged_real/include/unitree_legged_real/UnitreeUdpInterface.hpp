// unitree communication sdk
#include <unitree_legged_sdk/comm.h>
#include <unitree_legged_sdk/udp.h>

// ROS msgs
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

// Unitree ROS msgs
#include <unitree_legged_msgs/LowStateStamped.h>
// this is actually a copy of pronto_msgs::QuadrupedForceTorqueSensors
#include <unitree_legged_msgs/QuadrupedForceTorqueSensors.h>

// Other ROS stuff
#include <ros/node_handle.h>
#include <ros/publisher.h>

class UnitreeUdpRosInterface {
private:
  UNITREE_LEGGED_SDK::UDP low_udp;  
  UNITREE_LEGGED_SDK::LowState low_state;  

public:
  UnitreeUdpRosInterface(ros::NodeHandle &nh);

  void lowCmdCallback(const sensor_msgs::JointState::ConstPtr &joint_cmd);
  /**
   * @brief lowUdpRecv  wrapper that calls the SDK's udp.Recv() function
   */
  void lowUdpRecv();
  /**
   * @brief lowUdpGetRecv wrapper that calls the SDK's udp.GetRecv() function
   */
  void lowUdpGetRecv();

  void lowUdpSend();

  /**
   * @brief imuToRosMsg converts the IMU data structure defined in the SDK
   * into a ROS IMU message. The timestamp for the message is given as variable
   * @param imu the input IMU data structure as defined in UNITREE_LEGGED_SDK
   * @param stamp the timestamp for the ROS message
   * @param imu_msg the output IMU ROS message
   */
  static void imuToRosMsg(const UNITREE_LEGGED_SDK::IMU &imu,
                          const ros::Time &stamp, sensor_msgs::Imu &imu_msg);

  /**
   * @brief jointStateToRosMsg takes from a LowState the joint states and
   * converts them into a ROS JointState message with the \param stamp
   * @param state
   * @param stamp
   * @param joint_state_msg
   * @warning no checks are made on the size of the joint states, which should
   * be already allocated properly. Joint names are not initialized either.
   */
  static void jointStateToRosMsg(const UNITREE_LEGGED_SDK::LowState &state,
                                 const ros::Time &stamp,
                                 sensor_msgs::JointState &joint_state_msg);

  /**
   * @brief highStateToTwistMsg as the name suggests. NOT USED.
   * @param state
   * @param stamp
   * @param twist
   */
  static void highStateToTwistMsg(const UNITREE_LEGGED_SDK::HighState &state,
                                  const ros::Time &stamp,
                                  geometry_msgs::TwistStamped &twist);

  /**
   * @brief highStateToPoseMsg as the name suggests. NOT USED.
   * @param state
   * @param stamp
   * @param pose
   */
  static void highStateToPoseMsg(const UNITREE_LEGGED_SDK::HighState &state,
                                 const ros::Time &stamp,
                                 geometry_msgs::PoseStamped &pose);
  /**
   * @brief lowStateToFeetForces
   * @param state
   * @param stamp
   * @param feet_forces_msg
   */
  static void lowStateToFeetForces(const UNITREE_LEGGED_SDK::LowState &state,
                                    const ros::Time &stamp,
      unitree_legged_msgs::QuadrupedForceTorqueSensors &feet_forces_msg);

private:  
  ros::Publisher pub_low;
  ros::Publisher joint_state_pub;
  ros::Publisher imu_pub;
  ros::Publisher feet_forces_pub;

  ros::NodeHandle &nh_;

  ros::Time stamp;
  unitree_legged_msgs::LowStateStamped low_state_msg;
  sensor_msgs::JointState joint_state_msg;
  sensor_msgs::Imu imu_msg;
  unitree_legged_msgs::QuadrupedForceTorqueSensors feet_forces_msg;
  UNITREE_LEGGED_SDK::LowCmd cmd = {0};
};
