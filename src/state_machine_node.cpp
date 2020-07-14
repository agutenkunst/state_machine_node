#include <state_machine_node/state_machine_node.h>

#include <chrono>

namespace state_machine_node
{
  StateMachineNode::StateMachineNode(ros::NodeHandle& nh)
  {
    sm_.start();

    start_service_ = nh.advertiseService("start", &StateMachineNode::start, this);
    close_service_ = nh.advertiseService("close", &StateMachineNode::close, this);

    reading_thread_.reset(new std::thread(&StateMachineNode::readUdp, this));
  }

  StateMachineNode::~StateMachineNode()
  {
    start_service_.shutdown();
    if(reading_thread_->joinable())
    {
      reading_thread_abort_flag_ = true;
      reading_thread_->join();
    }

    sm_.stop();
  }

  bool StateMachineNode::start(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
  {
    std::unique_lock<std::mutex>(sm_access_mutex_);
    sm_.process_event(ros_service_start_event());

    return true;
  }

  bool StateMachineNode::close(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
  {
    std::unique_lock<std::mutex>(sm_access_mutex_);
    sm_.process_event(ros_service_close_event());

    return true;
  }

  bool StateMachineNode::readUdp()
  {
    try
    {
      while(ros::ok() && !reading_thread_abort_flag_)
      {
        if(socket_read_.available() == 0)
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          continue;
        }

        boost::array<char, 128> recv_buf;
        size_t len = socket_read_.receive_from(boost::asio::buffer(recv_buf), udp_read_endpoint_);

        std::string msg_received{recv_buf.data(), len};

        // React with event to the message (potentially could pass data as well....)
        if(msg_received == "ACK")
        {
          std::unique_lock<std::mutex>(sm_access_mutex_);
          sm_.process_event(udp_start_ack_recv_event());
        }
      }
    }
    catch (std::exception& e)
    {
      std::cerr << e.what() << std::endl;
    }
  }

}