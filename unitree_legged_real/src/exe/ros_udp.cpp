#include <unitree_legged_real/UnitreeUdpInterface.hpp>
#include <unitree_legged_sdk/loop.h>

using namespace UNITREE_LEGGED_SDK;

int main(int argc, char **argv) {
  ros::init(argc, argv, "ros_udp");

  ros::NodeHandle nh;
  UnitreeUdpRosInterface udp(nh);

  LoopFunc loop_udpGetRecv("low_udp_recv", 0.002, 3,
                        boost::bind(&UnitreeUdpRosInterface::lowUdpGetRecv, &udp));

  //LoopFunc loop_highUdpRecv("high_udp_recv", 0.002, 3,
  //                      boost::bind(&UnitreeUdpRosInterface::highUdpRecv, &udp));

  LoopFunc loop_udpRecv("low_udp_recv", 0.002, 3,
                        boost::bind(&UnitreeUdpRosInterface::lowUdpRecv, &udp));

  loop_udpRecv.start();
  loop_udpGetRecv.start();
  //loop_highUdpRecv.start();

  ros::spin();

  //loop_highUdpRecv.shutdown();
  loop_udpRecv.shutdown();
  loop_udpGetRecv.shutdown();

  return 0;
}
