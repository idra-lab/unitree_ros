#include <unitree_legged_real/UnitreeUdpInterface.hpp>
#include <unitree_legged_sdk/loop.h>

using namespace UNITREE_LEGGED_SDK;

int main(int argc, char **argv) {
  ros::init(argc, argv, "ros_udp");

  ros::NodeHandle nh;
  UnitreeUdpRosInterface udp(nh);
  InitEnvironment();

  // Not sure whether the separation between getRecv() and Recv() calls
  // is really needed or not, keeping it for now
  LoopFunc loop_udpGetRecv("low_udp_get_recv", 0.002, 3,
                        boost::bind(&UnitreeUdpRosInterface::lowUdpGetRecv, &udp));

  LoopFunc loop_udpRecv("low_udp_recv", 0.002, 3,
                        boost::bind(&UnitreeUdpRosInterface::lowUdpRecv, &udp));

  LoopFunc loop_udpSend("low_udp_send", 0.002, 3,
                        boost::bind(&UnitreeUdpRosInterface::lowUdpSend, &udp));
  // Apparently it is not possible to start in low and high state
  // simultaneously, choosing to get only the low state for now

  loop_udpGetRecv.start();
  loop_udpRecv.start();
  loop_udpSend.start();


  ros::spin();

  return 0;
}
