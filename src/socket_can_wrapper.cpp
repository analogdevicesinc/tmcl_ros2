/**
 * Copyright (c) 2023 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <unistd.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"

#include <tmcl_ros2/socket_can_wrapper.h>

////////////////////////////////////////////////////////////////////////////////

SocketCAN::SocketCAN(const char *logger_prefix):
  logger_prefix_(logger_prefix),
  logger_(rclcpp::get_logger((logger_prefix_ + ".SocketCAN").c_str()))
{
  RCLCPP_DEBUG_STREAM(logger_,"[" <<  __func__ << "] called");
  can_socket_ = -1;
  interface_name_ = "";
}

SocketCAN::~SocketCAN()
{
  RCLCPP_DEBUG_STREAM(logger_,"[" <<  __func__ << "] called");
}

bool SocketCAN::initialize(const char *interface_name)
{
  bool b_result = false;
  struct sockaddr_can addr;
  struct ifreq ifr;
  int flags;

  RCLCPP_INFO_STREAM(logger_,"[" <<  __func__ << "] called");

#ifdef DUMMYMODE
  RCLCPP_INFO_STREAM(logger_,"DUMMY " << __func__ << ", always TRUE");
  b_result = true;
#else
  if(can_socket_ == -1)
  {
    this->interface_name_ = interface_name;

    /* Create the socket */
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ != -1)
    {
      /* Get the interface index by interface name */
      addr.can_family = AF_CAN;
      strcpy(ifr.ifr_name, interface_name_.c_str());
      if(ioctl(can_socket_, SIOCGIFINDEX, &ifr) != -1)
      {
        addr.can_ifindex = ifr.ifr_ifindex;

        /* Set the non-blocking socket flag */
        flags = fcntl(can_socket_, F_GETFL, 0);
        if(flags != -1)
        {
          fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK);

          /* Bind the socket */
          if(bind(can_socket_, (struct sockaddr *) &addr, sizeof(addr)) == 0)
          {
              b_result = true;
          }
          else
          {
            RCLCPP_ERROR_STREAM(logger_,"[" << __func__ << "] Binding the CAN socket failed");
            close(can_socket_);
            can_socket_ = -1;
          }
        }
        else
        {
          RCLCPP_ERROR_STREAM(logger_,"[" << __func__ << "] Getting F_GETFL flag failed");
          close(can_socket_);
          can_socket_ = -1;
        }
      }
      else
      {
        RCLCPP_ERROR_STREAM(logger_,"[" << __func__ << "] Getting the interface index failed");
        close(can_socket_);
        can_socket_ = -1;
      }
    }
    else
    {
      RCLCPP_ERROR_STREAM(logger_,"[" << __func__ << "] Creating a CAN socket failed");
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(logger_,"[" << __func__ << "] There is an opened CAN socket already");
  }
#endif

  return b_result;
}

void SocketCAN::deinitialize()
{
  RCLCPP_INFO_STREAM(logger_,"[" <<  __func__ << "] called");

#ifdef DUMMYMODE
  RCLCPP_INFO_STREAM(logger_,"DUMMY " << __func__ << ", always TRUE");
#else
  if(can_socket_ != -1)
  {
    close(can_socket_);
    can_socket_ = -1;
  }
#endif

  return;
}

bool SocketCAN::framesAvailable()
{
  bool b_result = false;

  RCLCPP_DEBUG_STREAM(logger_,"[" <<  __func__ << "] called");

#ifdef DUMMYMODE
  RCLCPP_DEBUG_STREAM(logger_,"DUMMY " << __func__ << ", always TRUE");
  b_result = true;
#else

  if(can_socket_ != -1)
  {
    struct pollfd pfd;

    pfd.fd = can_socket_;
    pfd.revents = 0;
    pfd.events = POLLIN;

    /* Poll the CAN socket for incoming data without a timeout */
    RCLCPP_DEBUG_STREAM(logger_,"[" << __func__ << "] Waiting for frame...");
    if (poll(&pfd, 1, 0) == 1)
    {
      RCLCPP_DEBUG_STREAM(logger_,"[" << __func__ << "] Frame ready...");
      /* Ignore all events except for incoming data */
      if (pfd.revents == POLLIN)
      {
        b_result = true;
      }
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(logger_,"[" << __func__ << "] Error: No opened CAN socket");
  }
#endif
  return b_result;
}

bool SocketCAN::readFrame(uint32_t *id, uint8_t *data, uint8_t *size)
{
  bool b_result = false;

  RCLCPP_DEBUG_STREAM(logger_,"[" <<  __func__ << "] called");

#ifdef DUMMYMODE
  RCLCPP_DEBUG_STREAM(logger_,"DUMMY " << __func__ << ", always TRUE");
  b_result = true;
#else

  if(can_socket_ != -1)
  {
    can_frame frame;

    ssize_t count = read(can_socket_, &frame, sizeof(can_frame));
    if (count == sizeof(can_frame))
    {
      *id = frame.can_id;
      *size = frame.can_dlc;
      memcpy(data, frame.data, frame.can_dlc);
      b_result = true;
    }
    else
    {
      RCLCPP_ERROR_STREAM(logger_,"[" << __func__ << "] Error: Incorrect size of CAN frame " << \
        "(size = " << count << ")");
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(logger_,"[" << __func__ << "] Error: No opened CAN socket");
  }
#endif
  return b_result;
}

bool SocketCAN::writeFrame(uint32_t id, uint8_t *data, uint8_t size)
{
  bool b_result = false;

  RCLCPP_DEBUG_STREAM(logger_,"[" <<  __func__ << "] called");

#ifdef DUMMYMODE
  RCLCPP_DEBUG_STREAM(logger_,"DUMMY " << __func__ << ", always TRUE");
  b_result = true;
#else

  if (can_socket_ != -1)
  {
    can_frame frame;

    frame.can_id = id;
    frame.can_dlc = size;
    memcpy(frame.data, data, frame.can_dlc);

    ssize_t count = write(can_socket_, &frame, sizeof(can_frame));

    if (count == sizeof(can_frame))
    {
      b_result = true;
    }
    else
    {
      RCLCPP_ERROR_STREAM(logger_,"[" << __func__ << "] Error: Incorrect size of CAN frame " << \
        "(size = " << count << ")");
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(logger_,"[" << __func__ << "] Error: No opened CAN socket");
  }
#endif
  return b_result;
}
