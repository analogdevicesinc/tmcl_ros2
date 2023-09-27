/**
 * Copyright (c) 2023 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#ifndef SOCKET_CAN_WRAPPER_H
#define SOCKET_CAN_WRAPPER_H

class SocketCAN
{
public:
  /* Constructor */
  SocketCAN(const char *logger_prefix);

  /* Destructor */
  ~SocketCAN();

  /* Initialize CAN with specified interface name */
  bool initialize(const char *interface_name);

  /* De-initialize CAN */
  void deinitialize();

  /* Check if CAN frames are available */
  bool framesAvailable();

  /* Rx CAN frames */
  bool readFrame(uint32_t *id, uint8_t *data, uint8_t *size);

  /* Tx CAN frames */
  bool writeFrame(uint32_t id, uint8_t *data, uint8_t size);

private:
  int can_socket_;
  std::string interface_name_;
  std::string logger_prefix_;
  rclcpp::Logger logger_;
};

#endif // SOCKET_CAN_WRAPPER_H
