#ifndef STATE_MACHINE_NODE_H
#define STATE_MACHINE_NODE_H

#include <ros/ros.h>
#include <ros/service_server.h>
#include <std_srvs/Trigger.h>

#include <thread>
#include <mutex>
#include <atomic>

#include <state_machine_node/msm_state_machine.h>

#include <iostream>

#include <boost/asio.hpp>
#include <boost/bind.hpp>


using boost::asio::ip::udp;

namespace state_machine_node
{

class StateMachineNode
{
public:
  explicit StateMachineNode(ros::NodeHandle& nh);
  virtual ~StateMachineNode();

private:
  /* ROS Stuff*/
  ros::ServiceServer start_service_;
  ros::ServiceServer close_service_;

  bool start(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool close(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  /* StateMachine Stuff*/
  protocol_state_machine sm_; /* State machine implementing some protocol characteristics */
  std::mutex sm_access_mutex_;

  /* UDP Stuff*/
  boost::asio::io_service io_service_;
  udp::endpoint udp_read_endpoint_{udp::v4(), 5004};  /**< Endpoint for reading from UDP. */
  udp::socket socket_read_{io_service_, udp_read_endpoint_};               /**< Socket used for reading from UDP. */

  std::unique_ptr<std::thread> reading_thread_;
  std::atomic_bool reading_thread_abort_flag_{false};

  bool readUdp();
};

}  // namespace state_machine_node
#endif  // STATE_MACHINE_NODE_H
