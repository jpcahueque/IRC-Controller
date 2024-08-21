#include <irc_controller/IRC_Controller.hpp>
#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <netinet/in.h>

using boost::asio::ip::udp;
using namespace std;
using namespace ros;


namespace irc_controller
{
#if 0
}
#endif

IRC_Controller::IRC_Controller()
  : socket(io_service)
{
}


void IRC_Controller::init(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
{
  std::cout << "inside IRC controller Init" << std::endl; 
  std::string host, port;
  private_nh.param("host", host, string("192.168.1.255"));
  private_nh.param("port", port, string("2500"));

  private_nh.param("period", period, 0.022);
  private_nh.param("watchdog", watchdog, 1.);

  bool broadcast;
  private_nh.param("broadcast", broadcast, true);

  
  udp::resolver resolver(io_service);
  udp::udp::resolver::query query(udp::v4(), host, port);
  
  receiver_endpoint = *resolver.resolve(query);
  
  socket.open(udp::v4());
  socket.set_option(boost::asio::socket_base::broadcast(broadcast));

  for(int i=0; i<62; i++)
    udp_frame[i] = 0;
  udp_frame[1] = 4;
  
  for(int k=0; k<4; k++)
  {
    udp_frame[2+15*k] = 0x03;
    udp_frame[3+15*k] = 0x10;
  }
  
  timer = nh.createWallTimer(ros::WallDuration(period), &IRC_Controller::timerCallback, this);
  bindingTimer = nh.createWallTimer(ros::WallDuration(period), &IRC_Controller::bindingTimerCallback, this, false, false);
  for(int i=0; i<4; i++)
  {
    watchdogTimer[i] = nh.createWallTimer(ros::WallDuration(watchdog), boost::bind(&IRC_Controller::watchdogTimerCallback, this, _1, i), true, false);    
  }

  RC0Sub = nh.subscribe("rc0", 1, &IRC_Controller::rc0Callback, this);
  RC1Sub = nh.subscribe("rc1", 1, &IRC_Controller::rc1Callback, this);
  RC2Sub = nh.subscribe("rc2", 1, &IRC_Controller::rc2Callback, this);
  RC3Sub = nh.subscribe("rc3", 1, &IRC_Controller::rc3Callback, this);

  startBindingService = nh.advertiseService("start_binding", &IRC_Controller::startBinding, this);
  stopBindingService = nh.advertiseService("stop_binding", &IRC_Controller::stopBinding, this);
  std::cout << "end of irc controller init " << std::endl; 
}

void IRC_Controller::watchdogTimerCallback(const ros::WallTimerEvent& event, int id)
{
  boost::mutex::scoped_lock lock(mutex);
  watchdogTimer[id].stop();

  uint16_t* data = (uint16_t*)(&udp_frame[5+id*15]);
  
  udp_frame[2+id*15]=0x03;
  
  for(uint16_t i=0; i<6; i++)
  {
    data[i] = 0x0000;
  }
}

void IRC_Controller::rcCallback(const irc_controller::RCConstPtr & msg, int id)
{
  boost::mutex::scoped_lock lock(mutex);
  
  udp_frame[2+id*15]=0x02;

  uint16_t* data = (uint16_t*)(&udp_frame[5+id*15]);

  for(uint16_t i=0; i<6; i++)
  {
    //data[i] = htons((i << 10) | ( msg->values[i] & 0x03ff));
    data[i] = htons( msg->values[i] & 0x03ff);
  }
  watchdogTimer[id].setPeriod(ros::WallDuration(watchdog));
  watchdogTimer[id].start();
}

void IRC_Controller::rc0Callback(const irc_controller::RCConstPtr & msg)
{
  rcCallback(msg, 0);
}
void IRC_Controller::rc1Callback(const irc_controller::RCConstPtr & msg)
{
  rcCallback(msg, 1);
}
void IRC_Controller::rc2Callback(const irc_controller::RCConstPtr & msg)
{
  rcCallback(msg, 2);
}
void IRC_Controller::rc3Callback(const irc_controller::RCConstPtr & msg)
{
  rcCallback(msg, 3);
}


void IRC_Controller::timerCallback(const ros::WallTimerEvent& event)
{
  boost::mutex::scoped_lock lock(mutex);
    
  socket.send_to(boost::asio::buffer(udp_frame), receiver_endpoint);
  
  
//   cerr << std::hex;
//   for(int i=0; i<62; i++)
//     {
//       cerr << (int) udp_frame[i] << ":";
//     }
//   cerr << std::dec << endl;
  
}

void IRC_Controller::bindingTimerCallback(const ros::WallTimerEvent& event)
{
  socket.send_to(boost::asio::buffer(udp_binding_frame), receiver_endpoint);
}

bool IRC_Controller::startBinding(irc_controller::Bind::Request  &req,
                                  irc_controller::Bind::Response  &res)
{
  ROS_INFO_STREAM("start sending binding frames at the id " << (int) req.id );
  timer.stop();
  for(int i=0; i<62; i++)
    udp_binding_frame[i] = 0;
  udp_binding_frame[1] = 4;

  // send OFF to all emitters
  for(int k=0; k<4; k++)
    udp_binding_frame[2+k*15] = 0x03;
  socket.send_to(boost::asio::buffer(udp_binding_frame), receiver_endpoint);
  
  // send binding command
  udp_binding_frame[3+15*req.id] = 0x90;
  socket.send_to(boost::asio::buffer(udp_binding_frame), receiver_endpoint);

  // send ON to the binding emitter
  udp_binding_frame[2+15*req.id] = 0x02;
  socket.send_to(boost::asio::buffer(udp_binding_frame), receiver_endpoint);  
  
  // send normal mode to the binding emiter
  udp_binding_frame[2+15*req.id] = 0x00;
  
  bindingTimer.setPeriod(ros::WallDuration(period));
  bindingTimer.start();
  return true;
}

bool IRC_Controller::stopBinding(std_srvs::Empty::Request  &req,
                                 std_srvs::Empty::Response  &res)
{
  ROS_INFO("stop sending binding frames");
  bindingTimer.stop();
  
  timer.setPeriod(ros::WallDuration(period));
  timer.start();
  return true;
}

} // namespace irc_controller
