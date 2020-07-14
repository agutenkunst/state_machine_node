#include <ros/ros.h>
#include <state_machine_node/state_machine_node.h>

using namespace state_machine_node;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "state_machine_node");
  ros::NodeHandle nh;
  StateMachineNode state_machine_node{nh};

  ros::spin();

  std::cout << "Done\n";

  return EXIT_SUCCESS;
}
