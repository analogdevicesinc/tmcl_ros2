
/**
 * Copyright (c) 2023 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#ifndef TMCL_ROS2_H
#define TMCL_ROS2_H

#include "tmcl_ros2/tmcl_bldc_motor.h"
#include "tmcl_ros2/tmcl_stepper_motor.h"
#include "adi_tmcl/srv/tmc_custom_cmd.hpp"
#include "adi_tmcl/srv/tmc_gap_all.hpp"
#include "adi_tmcl/srv/tmc_ggp_all.hpp"

class TmclRos2
{
public:
  TmclRos2(rclcpp::Node::SharedPtr node);
  ~TmclRos2();

  bool init();

  bool deInit();

  bool getRetriesExceededStatus();

private:
  rclcpp::Node::SharedPtr p_node_;
  std::vector<Motor*> p_motor_;
  TmclInterpreter* p_tmcl_interpreter_;
  bool param_adhoc_mode_;
  std::vector<int64_t> param_en_motors_;
  uint8_t param_auto_start_additional_delay_;
  std::vector<int64_t> param_ap_type_;
  std::vector<std::string> param_ap_name_;
  std::vector<int64_t> param_gp_type_;
  std::vector<std::string> param_gp_name_;

  rclcpp::Service<adi_tmcl::srv::TmcCustomCmd>::SharedPtr tmcl_custom_cmd_service_server_;
  rclcpp::Service<adi_tmcl::srv::TmcGapAll>::SharedPtr tmcl_gap_service_server_;
  rclcpp::Service<adi_tmcl::srv::TmcGgpAll>::SharedPtr tmcl_ggp_service_server_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr p_param_callback_handle_;

  uint32_t module_number_;
  uint16_t fw_major_;
  uint16_t fw_minor_;
  bool b_allow_en_motors_change_;

  void initCommInterfaceParams();
  void initGeneralParams();
  bool initAxisAndGlobalParameters();
  void createMotor();
  void initServiceServer();
  void tmclCustomCmdCallback(const std::shared_ptr<adi_tmcl::srv::TmcCustomCmd::Request> req,
    const std::shared_ptr<adi_tmcl::srv::TmcCustomCmd::Response> res);
  void tmclGapAllCallback(const std::shared_ptr<adi_tmcl::srv::TmcGapAll::Request> req,
    const std::shared_ptr<adi_tmcl::srv::TmcGapAll::Response> res);
  void tmclGgpAllCallback(const std::shared_ptr<adi_tmcl::srv::TmcGgpAll::Request> req,
    const std::shared_ptr<adi_tmcl::srv::TmcGgpAll::Response> res);
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> \
    &parameters);

};

/*******************************************************************************/
/*                      Constants and Enumerations                             */
/*******************************************************************************/

/* Possible values for <Module Type> */
const uint8_t TMCM_BLDC = 6;

/* Parameter limits and default values */
const uint8_t TXRX_ID_MIN = 0;
const uint8_t TXRX_ID_MAX = 255;
const uint8_t TX_ID_DEFAULT = 1;
const uint8_t RX_ID_DEFAULT = 2;
const uint16_t TIMEOUT_MS_MIN = 0;
const uint16_t TIMEOUT_MS_MAX = 5000;
const uint16_t TIMEOUT_MS_DEFAULT = 10;
const uint8_t EXEC_CMD_RETRIES_MAX = 3;
const uint8_t EXEC_CMD_RETRIES_DEFAULT = 1;

const uint8_t AUTO_START_ADDITIONAL_DELAY_MAX = 60;
const uint8_t AUTO_START_ADDITIONAL_DELAY_DEFAULT = 0;

/* LUT row indeces of get-able/set-able General Parameters */
typedef enum
{
  IDX_ADHOC_MODE = 0,
  IDX_EN_MOTORS,
  IDX_AUTO_START_ADDITIONAL_DELAY,
  IDX_AXIS_PARAMETERS_TYPE,
  IDX_AXIS_PARAMETERS_NAME,
  IDX_GLOBAL_PARAMETERS_TYPE,
  IDX_GLOBAL_PARAMETERS_NAME,
  TMCL_GENERAL_PARAMS_LUT_MAX          /* This should not be used */
}tmcl_general_params_lut_t;

/* Parameter Names of get-able/set-able General Parameters */
static const std::vector<std::string> general_params_ = {
  "adhoc_mode",
  "en_motors",
  "auto_start_additional_delay",
  "AP_type",
  "AP_name",
  "GP_type",
  "GP_name",
  "MAX"         /* This should not be used */
};

/* LUT row indeces of Custom Service Server Commands */
typedef enum
{
  IDX_SAP = 0,
  IDX_GAP,
  IDX_SGP,
  IDX_GGP
}tmcl_custom_cmd_lut_t;

/* Custom Service Server Commands */
static const std::vector<std::string> tmcl_custom_cmd_ = {
  "SAP",
  "GAP",
  "SGP",
  "GGP"
};


#endif // TMCL_ROS2_H
