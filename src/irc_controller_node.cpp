#include <irc_controller/IRC_Controller.hpp>

using namespace irc_controller;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "irc_controller_node");
  
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  
  IRC_Controller node;
  node.init(nh, private_nh);
  
  ros::spin();
  return 0;
}
