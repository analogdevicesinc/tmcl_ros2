/**
 * Copyright (c) 2023 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#ifndef TMCL_INTERPRETER_H
#define TMCL_INTERPRETER_H

#include <future>
#include <chrono>
#include <vector>

#include "tmcl_ros2/socket_can_wrapper.h"

/** A TMCL Tx/Rx format will always be lke the following:
  * - Tx: | [[Module Address]] |     <Command>    |  <Type>  |  <Motor>  | <Value> | <Value> | <Value> | <Value> | [[Checksum]]
  * - Rx: | [[Reply  Address]] | <Module Address> | <Status> | <Command> | <Value> | <Value> | <Value> | <Value> | [[Checksum]]
  *
  * Specifically, for CAN interface:
  *  - Both Tx and Rx length will just be 7 bytes.
  *    Those enclosed with [[]] are not part of the message (they will automatically be added on the hardware-level).
  *  - Module Address: CAN-ID of the Sender
  *  - Reply Address: CAN-ID of the Receiver
  *  - Command, Type, Motor, Status can be either of the values enumerated below
  **/
const uint8_t TMCL_MSG_SZ = 7;
const uint8_t TMCL_MSG_VALUE_SZ = 4;

/*******************************************************************************/
/*                        Interface related settings                           */
/*******************************************************************************/

/* Supported interfaces */
typedef enum
{
  TMCL_INTERFACE_CAN = 0, /* 0 - CAN Interface */
  TMCL_INTERFACE_MAX,     /* This should not be used */
} tmcl_interface_t;

/* Supported interface's configurations */
/* Note: So far, these are all needed for CAN. Once other interfaces are added,
         convert this tmcl_cfg_t to a union then include different tmcl_cfg_<interface>_t as members. */
typedef struct 
{
  SocketCAN *p_socket_can;
  std::string interface_name;
  uint16_t tx_id;
  uint16_t rx_id;
} tmcl_cfg_t;

/*******************************************************************************/
/*                      TMCL protocol related settings                         */
/*******************************************************************************/
/* Possible values for <Command> */ 
typedef enum
{
  TMCL_CMD_ROR = 1,           /*   1 - Rotate Right */
  TMCL_CMD_ROL,               /*   1 - Rotate Left */
  TMCL_CMD_MST,               /*   3 - Stop motor movement */
  TMCL_CMD_MVP,               /*   4 - Move to position */
  TMCL_CMD_SAP,               /*   5 - Set axis parameter */
  TMCL_CMD_GAP,               /*   6 - Get axis parameter */
  TMCL_CMD_SGP = 9,           /*   9 - Set global parameter */
  TMCL_CMD_GGP,               /*  10 - Get global parameter */
  TMCL_CMD_APPGFWV = 136,     /* 136 - Get firmware version */
  TMCL_CMD_MAX                /* This should not be used */
} tmcl_cmd_t;

/* Possible values for <Status> */
typedef enum
{
  TMCL_STS_ERR_CHKSUM = 1,
  TMCL_STS_ERR_CMD,
  TMCL_STS_ERR_TYPE,
  TMCL_STS_ERR_VAL,
  TMCL_STS_ERR_EEPROM_LCK,
  TMCL_STS_ERR_CMD_NA,
  TMCL_STS_ERR_NONE = 100,
  TMCL_STS_CMD_LOADED,
  TMCL_STS_MAX            /* This should not be used */
} tmcl_sts_t;

/* Definition for info needed to execute 1 TMCL command */
typedef struct
{
  uint16_t tx_id;                     /* Module Address */
  uint16_t rx_id;                     /* <Reply Address */
  tmcl_cmd_t cmd;                     /* <Command> */
  uint8_t type;                       /* <Type> */
  uint8_t motor;                 /* <Motor> */
  uint8_t value[TMCL_MSG_VALUE_SZ];   /* 4-byte value */
  tmcl_sts_t sts;
} tmcl_msg_t;


/* Class definition for TMCL Interpreter */
class TmclInterpreter 
{
public:
  /* Constructor */
  TmclInterpreter(tmcl_interface_t tmcl_interface, tmcl_cfg_t tmcl_cfg, uint16_t timeout_ms,
    uint8_t comm_exec_cmd_retries, const char *logger_prefix_);

  /* Destructor */
  ~TmclInterpreter();

/* Reset interface */
  bool resetInterface();

/* Execute command (Direct mode) */
  bool executeCmd(tmcl_cmd_t cmd, uint8_t type, uint8_t motor, int32_t *val);
  bool executeCmd(tmcl_cmd_t cmd, const char* type, uint8_t motor, int32_t *val);

/* Shutdown interface */
  bool shutdownInterface();

  void setAp(std::vector<std::string> ap_name, std::vector<int64_t> ap_type);
  void setGp(std::vector<std::string> gp_name, std::vector<int64_t> gp_type);

  bool getRetriesExceededStatus();

private:
  tmcl_interface_t tmcl_interface_;
  tmcl_cfg_t tmcl_cfg_;
  bool interface_enabled_;
  uint16_t timeout_ms_;
  uint8_t comm_exec_cmd_retries_;
  std::vector<std::string> ap_name_;
  std::vector<int64_t> ap_type_;
  std::vector<std::string> gp_name_;
  std::vector<int64_t> gp_type_;
  std::string logger_prefix_;
  rclcpp::Logger logger_;
  bool b_retries_exceeded_;
};

#endif /* _TMCL_INTERPRETER_H */
