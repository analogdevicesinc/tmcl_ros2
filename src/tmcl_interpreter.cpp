/**
 * Copyright (c) 2023 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tmcl_ros2/tmcl_interpreter.h"

#include <algorithm>
#include <iterator>

////////////////////////////////////////////////////////////////////////////////

/* Constructor */
TmclInterpreter::TmclInterpreter(tmcl_interface_t tmcl_interface, tmcl_cfg_t tmcl_cfg,
uint16_t timeout_ms, uint8_t comm_exec_cmd_retries, const char *logger_prefix):
  interface_enabled_(false), 
  timeout_ms_(timeout_ms), 
  comm_exec_cmd_retries_(comm_exec_cmd_retries),
  logger_prefix_(logger_prefix),
  logger_(rclcpp::get_logger((logger_prefix_ + ".TmclInterpreter").c_str()))
{
  RCLCPP_INFO_STREAM(logger_,"[" <<  __func__ << "] called");
  tmcl_interface_ = tmcl_interface;

  if(tmcl_interface_ == TMCL_INTERFACE_CAN)
  {
    tmcl_cfg_ = tmcl_cfg;
    tmcl_cfg_.p_socket_can = nullptr;
  }
  b_retries_exceeded_ = false;
}

/* Destructor */
TmclInterpreter::~TmclInterpreter()
{
  RCLCPP_INFO_STREAM(logger_,"[" <<  __func__ << "] called");
  if(tmcl_interface_ == TMCL_INTERFACE_CAN)
  {
    tmcl_cfg_.interface_name = "";
    tmcl_cfg_.tx_id = 0;
    tmcl_cfg_.rx_id = 0;
  }
  interface_enabled_ = false;
  timeout_ms_ = 0;
}

/* Reset interface */
bool TmclInterpreter::resetInterface()
{
  RCLCPP_INFO_STREAM(logger_,"[" <<  __func__ << "] called");

  bool b_result = false;

  if(tmcl_interface_ == TMCL_INTERFACE_CAN)
  {
    tmcl_cfg_.p_socket_can = new SocketCAN(logger_prefix_.c_str());
    if(tmcl_cfg_.p_socket_can->initialize(tmcl_cfg_.interface_name.c_str()))
    {
      RCLCPP_INFO_STREAM(logger_,"[" << __func__ <<"] called");
      interface_enabled_ = true;
      b_result = true;
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(logger_,"[" << __func__ <<"] Interface not yet supported");
  }

  return b_result;
}

/* Execute command */
bool TmclInterpreter::executeCmd(tmcl_cmd_t cmd, uint8_t type, uint8_t motor, int32_t *val)
{
  RCLCPP_DEBUG_STREAM(logger_,"[" <<  __func__ << "] called");
  bool b_result = false;
  uint8_t n_retries = comm_exec_cmd_retries_;
  uint8_t rx_msg[TMCL_MSG_SZ] = { 0 };
  uint32_t rx_msg_id = 0;
  uint8_t rx_msg_sz = 0;
  tmcl_msg_t tmcl_msg;

  if(interface_enabled_)
  {
    if(tmcl_interface_ == TMCL_INTERFACE_CAN)
    {
      tmcl_msg.tx_id = tmcl_cfg_.tx_id;
      tmcl_msg.rx_id = tmcl_cfg_.rx_id;
      tmcl_msg.cmd = cmd;
      tmcl_msg.type = type;
      tmcl_msg.motor = motor;
      tmcl_msg.value[0] = (*val & 0xFF000000) >> 24;
      tmcl_msg.value[1] = (*val & 0x00FF0000) >> 16;
      tmcl_msg.value[2] = (*val & 0x0000FF00) >> 8;
      tmcl_msg.value[3] = (*val & 0x000000FF);
      // Setting cmd, type, motor, value is always needed every call to execute_cmd()
      uint8_t tx_msg[TMCL_MSG_SZ] = {tmcl_msg.cmd, tmcl_msg.type, tmcl_msg.motor, \
        tmcl_msg.value[0], tmcl_msg.value[1], tmcl_msg.value[2], tmcl_msg.value[3]};
      auto start_time = std::chrono::system_clock::now();
      auto end_time = start_time;

      while (0 < n_retries)
      {
        RCLCPP_DEBUG(logger_,"[%s] [T%d] before execute_cmd(), value=%d sending [0x%02x 0x%02x " \
          "0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x]", __func__, n_retries, *val, tmcl_msg.tx_id, \
          tmcl_msg.cmd, tmcl_msg.type, tmcl_msg.motor, tmcl_msg.value[0], tmcl_msg.value[1], \
          tmcl_msg.value[2], tmcl_msg.value[3]);

        /* Send TMCL command */
        if(tmcl_cfg_.p_socket_can->writeFrame(tmcl_msg.tx_id, tx_msg, TMCL_MSG_SZ))
        {
          while(timeout_ms_ > \
            std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count())
          {
            if(tmcl_cfg_.p_socket_can->framesAvailable() && \
               tmcl_cfg_.p_socket_can->readFrame(&rx_msg_id, rx_msg, &rx_msg_sz))
            {
              /* A response for the TMCL command is received */
              if((rx_msg_id == tmcl_msg.rx_id) && 
                (rx_msg_sz == TMCL_MSG_SZ) &&
                (rx_msg[0] == tmcl_msg.tx_id) &&
                (rx_msg[2] == tmcl_msg.cmd))
              {
                tmcl_msg.sts = (tmcl_sts_t)rx_msg[1];
                tmcl_msg.value[0] = rx_msg[3];
                tmcl_msg.value[1] = rx_msg[4];
                tmcl_msg.value[2] = rx_msg[5];
                tmcl_msg.value[3] = rx_msg[6];

                RCLCPP_DEBUG(logger_,"[%s] [T%d] execute_cmd() success, received [0x%02x 0x%02x" \
                  " 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x]", __func__, n_retries, tmcl_msg.tx_id, \
                  tmcl_msg.sts, tmcl_msg.cmd, tmcl_msg.value[0], tmcl_msg.value[1], \
                  tmcl_msg.value[2], tmcl_msg.value[3]);
                *val = (tmcl_msg.value[0] << 24) + (tmcl_msg.value[1] << 16) +\
                 (tmcl_msg.value[2] << 8) + tmcl_msg.value[3];
                b_result = true;
                break;
              }
              else
              {
                RCLCPP_DEBUG(logger_,"[%s] [T%d] execute_cmd() failed, received [0x%02x 0x%02x" \
                  " 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x]", __func__, n_retries, rx_msg[0], \
                  (tmcl_sts_t)rx_msg[1], rx_msg[2], rx_msg[3], rx_msg[4], rx_msg[5], rx_msg[6]);
              }
            }
            end_time = std::chrono::system_clock::now();
          }
        }
        
        if(!b_result)
        {
          n_retries--;
        }
        else
        {
          break;
        }
      }

      if(n_retries == 0)
      {
        b_retries_exceeded_ = true;
        RCLCPP_ERROR_STREAM(logger_,"[" << __func__ << "] Retries exceeded");
      }
    }
    else
    {
      RCLCPP_ERROR_STREAM(logger_,"[" << __func__ << "] Interface not yet supported");
    }
  }
  return b_result;
}

bool TmclInterpreter::executeCmd(tmcl_cmd_t cmd, const char* type, uint8_t motor, int32_t *val)
{
  bool b_result = false;
  int ap_index = 0;
  int gp_index = 0;
  int type_int = 0;

  // Find in ap_name
  std::vector<std::string>::iterator it_begin_ap = ap_name_.begin();
  std::vector<std::string>::iterator it_end_ap = ap_name_.end();
  std::vector<std::string>::iterator it_find_ap = std::find(it_begin_ap, it_end_ap, type);
  if(it_find_ap != it_end_ap)
  {
    // Found type in ap_name_;
    ap_index = std::distance(it_begin_ap,it_find_ap);
    RCLCPP_DEBUG_STREAM(logger_,"Found "<< type << "in AP_name");

    type_int = ap_type_[ap_index];
    b_result = executeCmd(cmd,type_int,motor,val);
  }
  else
  {
    RCLCPP_DEBUG_STREAM(logger_,"Did not find "<< type << "in AP_name");
  }

  if(!b_result)
  {
    // Find in gp_name
    std::vector<std::string>::iterator it_begin_gp = gp_name_.begin();
    std::vector<std::string>::iterator it_end_gp = gp_name_.end();
    std::vector<std::string>::iterator it_find_gp = std::find(it_begin_gp, it_end_gp, type);
    if(it_find_gp != it_end_gp)
    {
      // Found type in ap_name_;
      gp_index = std::distance(it_begin_gp,it_find_gp);
      RCLCPP_DEBUG_STREAM(logger_,"Found "<< type << " in GP_name");

      type_int = gp_type_[gp_index];
      b_result = executeCmd(cmd,type_int,motor,val);
    }
    else
    {
      RCLCPP_DEBUG_STREAM(logger_,"Did not find "<< type << " in GP_name");
    }
  }
  return b_result;
}

/* Shutdown interface */
bool TmclInterpreter::shutdownInterface()
{
  RCLCPP_INFO_STREAM(logger_,"[" <<  __func__ << "] called");
  bool b_result = false;

  if(tmcl_interface_ == TMCL_INTERFACE_CAN)
  {
    if(tmcl_cfg_.p_socket_can != nullptr)
    {
      tmcl_cfg_.p_socket_can->deinitialize();
      delete tmcl_cfg_.p_socket_can;
      tmcl_cfg_.p_socket_can = nullptr;
    }
    interface_enabled_ = false;
    b_result = true;
  }
  return b_result;
}

void TmclInterpreter::setAp(std::vector<std::string> ap_name, std::vector<int64_t> ap_type)
{
  RCLCPP_DEBUG_STREAM(logger_,"[" <<  __func__ << "] called");
  ap_name_.clear();
  ap_type_.clear();
  ap_name_=ap_name;
  ap_type_=ap_type;
  for(auto i: ap_name_)
  {
    RCLCPP_DEBUG_STREAM(logger_,i);
  }
  return;
}

void TmclInterpreter::setGp(std::vector<std::string> gp_name, std::vector<int64_t> gp_type)
{
  RCLCPP_DEBUG_STREAM(logger_,"[" <<  __func__ << "] called");
  gp_name_.clear();
  gp_type_.clear();
  gp_name_=gp_name;
  gp_type_=gp_type;
  for(auto i: gp_name_)
  {
    RCLCPP_DEBUG_STREAM(logger_,i);
  }
  return;
}

bool TmclInterpreter::getRetriesExceededStatus()
{
  return b_retries_exceeded_;
}
