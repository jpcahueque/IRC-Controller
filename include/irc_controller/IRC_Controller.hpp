#ifndef __IRC_CONTROLLER_HPP__
#define __IRC_CONTROLLER_HPP__

#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <irc_controller/RC.h>

#include <std_srvs/Empty.h>
#include <irc_controller/Bind.h>

namespace irc_controller
{
#if 0
}
#endif

class IRC_Controller
{
private:  
  boost::asio::io_service io_service;
  boost::asio::ip::udp::udp::socket socket;
  boost::asio::ip::udp::udp::endpoint receiver_endpoint;
  
  boost::mutex mutex;

  double period;
  double watchdog;
  
  ros::NodeHandle nh;
  
  ros::Subscriber RC0Sub, RC1Sub, RC2Sub, RC3Sub;

  ros::WallTimer timer, bindingTimer,
    watchdogTimer[4];
  
  boost::array<uint8_t, 62> udp_frame, udp_binding_frame;

  ros::ServiceServer startBindingService, stopBindingService;
  
  void rcCallback(const irc_controller::RCConstPtr & msg, int id);
  
  void rc0Callback(const irc_controller::RCConstPtr & msg);
  void rc1Callback(const irc_controller::RCConstPtr & msg);
  void rc2Callback(const irc_controller::RCConstPtr & msg);
  void rc3Callback(const irc_controller::RCConstPtr & msg);
  
  void timerCallback(const ros::WallTimerEvent& event);
  void bindingTimerCallback(const ros::WallTimerEvent& event);
  void watchdogTimerCallback(const ros::WallTimerEvent& event, int id);
  
public:
  IRC_Controller();

  void init(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  
  bool startBinding(irc_controller::Bind::Request  &req,
                    irc_controller::Bind::Response  &res);
  bool stopBinding(std_srvs::Empty::Request  &req,
                   std_srvs::Empty::Response  &res);
  
}; // class IRC_Controller
} // namespace irc_controller

#endif // __IRC_CONTROLLER_HPP__
