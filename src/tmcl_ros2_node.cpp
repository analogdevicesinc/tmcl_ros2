/**
 * Copyright (c) 2023 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tmcl_ros2/tmcl_ros2.h"

TmclRos2 *p_tmcl_ros2 = nullptr;
bool g_shutdown_signal = false;

void graceful_shutdown();
void signal_callback_handler(int signum);

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("tmcl_ros2_node");

  RCLCPP_INFO_STREAM(node->get_logger(),"Starting " << node->get_name() << " ...");

  RCLCPP_DEBUG_STREAM(node->get_logger(),"Installing new signal handlers...");
  rclcpp::uninstall_signal_handlers();
  std::signal(SIGINT, signal_callback_handler);
  std::signal(SIGTERM, signal_callback_handler);
  std::signal(SIGKILL, signal_callback_handler);

  try
  {
    p_tmcl_ros2 = new TmclRos2(node);

    if(p_tmcl_ros2->init())
    {
      RCLCPP_INFO_STREAM(node->get_logger(),"Initialization successful.");
      while(!g_shutdown_signal && !p_tmcl_ros2->getRetriesExceededStatus())
      {
        rclcpp::spin_some(node);
      }
      RCLCPP_WARN_STREAM(node->get_logger(),"Exiting main loop...");
      throw-1;
    }
    else
    {
      RCLCPP_ERROR_STREAM(node->get_logger(),"Error initializing node... shutting down");
      throw-1;
    }
  }
  catch(...)
  {
    graceful_shutdown();
  }
  return 0;
}

void graceful_shutdown()
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("tmcl_ros2_node shutdown"),"Initiating graceful shutdown...");
  if(p_tmcl_ros2->deInit())
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("tmcl_ros2_node shutdown"),"Successfully de-initialized TMC");
  }
  delete p_tmcl_ros2;
  p_tmcl_ros2 = nullptr;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("tmcl_ros2_node shutdown"),"Successfully shutdown...");
  rclcpp::shutdown();
}

void signal_callback_handler(int signum)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("tmcl_ros2_node shutdown"),"Caught signal: " << signum << ". Terminating...");
  g_shutdown_signal = true;
}
