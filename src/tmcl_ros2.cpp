/**
 * Copyright (c) 2023 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tmcl_ros2/tmcl_ros2.h"

#include <string>

using std::placeholders::_1;


TmclRos2::TmclRos2(rclcpp::Node::SharedPtr node):
  p_node_(node),
  p_motor_(1),
  p_tmcl_interpreter_(nullptr),
  p_param_callback_handle_(nullptr)
{
  RCLCPP_INFO_STREAM(p_node_->get_logger(),"[TmclRos2::" <<  __func__ << "] called");
  b_allow_en_motors_change_ = true;
}

TmclRos2::~TmclRos2()
{
  RCLCPP_INFO_STREAM(p_node_->get_logger(),"[TmclRos2::" <<  __func__ << "] called");
  p_node_ = nullptr;
  for(uint64_t i = 0; i < p_motor_.size(); i++)
  {
    delete p_motor_[i];
    p_motor_[i] = nullptr;
  }
  delete p_tmcl_interpreter_;
  p_tmcl_interpreter_ = nullptr;
}

bool TmclRos2::init()
{
  RCLCPP_INFO_STREAM(p_node_->get_logger(),"[TmclRos2::" <<  __func__ << "] called");
  bool b_success = false;
  tmcl_interface_t tmcl_interface;
  tmcl_cfg_t tmcl_cfg;
  int timeout_ms;
  uint8_t n_retries;

  p_param_callback_handle_=p_node_->add_on_set_parameters_callback(std::bind(\
    &TmclRos2::parametersCallback, this, std::placeholders::_1));

  this->initCommInterfaceParams();

  tmcl_interface = static_cast<tmcl_interface_t>(p_node_->get_parameter(comm_interface_params_\
    [IDX_COMM_INTERFACE]).as_int());
  tmcl_cfg.interface_name = p_node_->get_parameter(comm_interface_params_\
    [IDX_COMM_INTERFACE_NAME]).as_string();
  tmcl_cfg.tx_id = p_node_->get_parameter(comm_interface_params_[IDX_COMM_TX_ID]).as_int();
  tmcl_cfg.rx_id = p_node_->get_parameter(comm_interface_params_[IDX_COMM_RX_ID]).as_int();
  timeout_ms = p_node_->get_parameter(comm_interface_params_[IDX_COMM_TIMEOUT_MS]).as_int();
  n_retries = p_node_->get_parameter(comm_interface_params_[IDX_COMM_EXEC_CMD_RETRIES]).as_int();
  std::string node_logger_name=p_node_->get_logger().get_name();
  p_tmcl_interpreter_ = new TmclInterpreter(tmcl_interface,tmcl_cfg,timeout_ms,n_retries, \
    node_logger_name.c_str());

  if(!p_tmcl_interpreter_->resetInterface())
  {
    RCLCPP_ERROR_STREAM(p_node_->get_logger(),"Reset interface failed");
    return b_success;
  }

  int32_t val = 0;
  const uint8_t motor_number = 0;
  uint8_t string_input = 1;
  if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_APPGFWV,string_input,motor_number, &val))
  {
    RCLCPP_INFO(p_node_->get_logger(),"Able to get Firmware Version: 0x%08X", val);
  }
  else
  {
    RCLCPP_ERROR_STREAM(p_node_->get_logger(),"Error getting FW version");
  }

  module_number_ = (val & 0xFFFF0000) >> 16;
  fw_major_ = (val & 0x0000FF00) >> 8;
  fw_minor_ = (val & 0x000000FF);

  RCLCPP_INFO_STREAM(p_node_->get_logger(), "Module Number: " << module_number_);
  RCLCPP_INFO_STREAM(p_node_->get_logger(), "FW Version: " << fw_major_ << "." << fw_minor_);
  this->initGeneralParams();

  b_success = this->initAxisAndGlobalParameters();

  if(b_success)
  {
    if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GGP,"auto start mode",motor_number, &val))
    {
      RCLCPP_DEBUG(p_node_->get_logger(),"auto start mode: 0x%02X", val);
      // If TMCL application will start during power up of board
      if(val == 1)
      {
        int auto_start_additional_delay=p_node_->get_parameter(\
          general_params_[IDX_AUTO_START_ADDITIONAL_DELAY]).as_int();
        RCLCPP_INFO_STREAM(p_node_->get_logger(),"Auto start mode is enabled");
        RCLCPP_INFO_STREAM(p_node_->get_logger(),"Wait " << (2.0 + auto_start_additional_delay) << \
          " secs to autostart TMCL Program");
        uint8_t delay_s = 2+auto_start_additional_delay;
        rclcpp::sleep_for(std::chrono::seconds(delay_s));
      }
    }
    else
    {
      RCLCPP_WARN_STREAM(p_node_->get_logger(),"Error getting auto start mode");
    }

    this->createMotor();
    this->initServiceServer();
  }
  return b_success;
}

void TmclRos2::initCommInterfaceParams()
{
  RCLCPP_INFO_STREAM(p_node_->get_logger(),"[TmclRos2::" <<  __func__ << "] called");
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  rcl_interfaces::msg::IntegerRange param_int_range;

  param_desc.name = comm_interface_params_[IDX_COMM_INTERFACE];
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  param_desc.description = "Interface used between the PC and the TMC";
  param_desc.additional_constraints = "Supported Interface/s are: CAN - 0";
  param_desc.read_only = true;

  param_int_range.from_value = TMCL_INTERFACE_CAN;
  param_int_range.step = 1;
  param_int_range.to_value = TMCL_INTERFACE_MAX;
  param_desc.integer_range.push_back(param_int_range);

  p_node_->declare_parameter(param_desc.name,static_cast<int>(TMCL_INTERFACE_CAN),param_desc);
  param_desc.integer_range.clear();


  param_desc.name = comm_interface_params_[IDX_COMM_INTERFACE_NAME];
  param_desc.type = rclcpp::ParameterType::PARAMETER_STRING;
  param_desc.description = "Name of the interface or device as detected by the PC";
  param_desc.additional_constraints = "Possible values: can0, can1, can2, ..., etc";
  param_desc.read_only = true;

  p_node_->declare_parameter(param_desc.name,"can0",param_desc);


  param_desc.name = comm_interface_params_[IDX_COMM_TX_ID];
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  param_desc.description = "Currently applicable only for CAN interface (Tx ID)";
  param_desc.additional_constraints.clear();
  param_desc.read_only = true;

  param_int_range.from_value = TXRX_ID_MIN;
  param_int_range.step = 1;
  param_int_range.to_value = TXRX_ID_MAX;
  param_desc.integer_range.push_back(param_int_range);

  p_node_->declare_parameter(param_desc.name,TX_ID_DEFAULT,param_desc);
  param_desc.integer_range.clear();


  param_desc.name = comm_interface_params_[IDX_COMM_RX_ID];
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  param_desc.description = "Currently applicable only for CAN interface (Rx ID)";
  param_desc.additional_constraints.clear();
  param_desc.read_only = true;

  param_int_range.from_value = TXRX_ID_MIN;
  param_int_range.step = 1;
  param_int_range.to_value = TXRX_ID_MAX;
  param_desc.integer_range.push_back(param_int_range);

  p_node_->declare_parameter(param_desc.name,RX_ID_DEFAULT,param_desc);
  param_desc.integer_range.clear();


  param_desc.name = comm_interface_params_[IDX_COMM_TIMEOUT_MS];
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  param_desc.description = "Indicates how long should the node will wait for the rx data";
  param_desc.additional_constraints = "Unit: milliseconds";
  param_desc.read_only = true;

  param_int_range.from_value = TIMEOUT_MS_MIN;
  param_int_range.step = 1;
  param_int_range.to_value = TIMEOUT_MS_MAX;
  param_desc.integer_range.push_back(param_int_range);

  p_node_->declare_parameter(param_desc.name,TIMEOUT_MS_DEFAULT,param_desc);
  param_desc.integer_range.clear();


  param_desc.name = comm_interface_params_[IDX_COMM_EXEC_CMD_RETRIES];
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  param_desc.description = "Indicates how many the node will retry sending data before shutting \
    down if no data received";
  param_desc.additional_constraints.clear();
  param_desc.read_only = true;

  param_int_range.from_value = EXEC_CMD_RETRIES_DEFAULT;
  param_int_range.step = 1;
  param_int_range.to_value = EXEC_CMD_RETRIES_MAX;
  param_desc.integer_range.push_back(param_int_range);

  p_node_->declare_parameter(param_desc.name,EXEC_CMD_RETRIES_DEFAULT,param_desc);

}

rcl_interfaces::msg::SetParametersResult TmclRos2::parametersCallback(const \
  std::vector<rclcpp::Parameter> &parameters)
{
  RCLCPP_DEBUG_STREAM(p_node_->get_logger(),"[TmclRos2::" <<  __func__ << "] called");
  rcl_interfaces::msg::SetParametersResult result;
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  bool b_valid_type = false;
  for(const auto &parameter : parameters)
  {
    RCLCPP_DEBUG_STREAM(p_node_->get_logger(),"Parameter: " << parameter.get_name());
    RCLCPP_DEBUG_STREAM(p_node_->get_logger(),"Parameter Type: " << parameter.get_type());
    if(p_node_->has_parameter(parameter.get_name()))
    {
      param_desc = p_node_->describe_parameter(parameter.get_name());
      if(parameter.get_type() == param_desc.type)
      {
        b_valid_type = true;
      }
      else // All other params
      {
        result.successful = false;
        result.reason = "Incorrect parameter type.";
      }
    }
    else
    {
      // Assume this callback is called during declaration of parameters.
      RCLCPP_INFO_STREAM(p_node_->get_logger(),"Declaring "<< parameter.get_name() << \
        ". Using values from YAML/Launch file.");
      b_valid_type = true;
    }

    // Validate en_motors[]
    if(b_valid_type && parameter.get_name() == general_params_[IDX_EN_MOTORS])
    {
      if(b_allow_en_motors_change_)
      {
        result.successful = true;
        result.reason = "Allow declaration/setting of " + general_params_[IDX_EN_MOTORS];
        RCLCPP_DEBUG_STREAM(p_node_->get_logger(),"Allow declaration/setting of " << \
          general_params_[IDX_EN_MOTORS]);
      }
      else
      {
        result.successful = false;
        result.reason = "Cannot change " + general_params_[IDX_EN_MOTORS];
        RCLCPP_DEBUG_STREAM(p_node_->get_logger(),"Cannot change " << \
          general_params_[IDX_EN_MOTORS]);
      }
    }
    else if(b_valid_type)
    {
      result.successful = true;
      result.reason = "Success";
    }
  }

  return result;
}

void TmclRos2::initGeneralParams()
{
  RCLCPP_INFO_STREAM(p_node_->get_logger(),"[TmclRos2::" <<  __func__ << "] called");
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  rcl_interfaces::msg::IntegerRange param_int_range;
  uint8_t total_motors = 0;

  param_desc.name = general_params_[IDX_ADHOC_MODE];
  param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
  param_desc.description = "Enable/Disable adhoc mode";
  param_desc.read_only = true;
  p_node_->declare_parameter(param_desc.name,false,param_desc);
  param_adhoc_mode_ = p_node_->get_parameter(param_desc.name).as_bool();

  param_desc.name = general_params_[IDX_EN_MOTORS];
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY;
  param_desc.description = "Array for enabled motors";
  param_desc.read_only = false;
  std::vector<int64_t> default_en_motors={};
  p_node_->declare_parameter(param_desc.name,default_en_motors,param_desc);
  param_en_motors_ = p_node_->get_parameter(param_desc.name).as_integer_array();

  total_motors = module_number_ / 1000;
  p_motor_.resize(total_motors, nullptr);

  if(param_en_motors_.size() > total_motors)
  {
    RCLCPP_WARN_STREAM(p_node_->get_logger(),"Exceeded Maximum Motors. Ignoring excess...");
    param_en_motors_.resize(total_motors);
    b_allow_en_motors_change_ = true;
    p_node_->set_parameter(rclcpp::Parameter(general_params_[IDX_EN_MOTORS],param_en_motors_));
    b_allow_en_motors_change_ = false;
  }
  else if(param_en_motors_.size() < total_motors)
  {
    RCLCPP_WARN_STREAM(p_node_->get_logger(),"Did not set other motors. Disabling others...");
    param_en_motors_.resize(total_motors,0);
    b_allow_en_motors_change_ = true;
    p_node_->set_parameter(rclcpp::Parameter(general_params_[IDX_EN_MOTORS],param_en_motors_));
    b_allow_en_motors_change_ = false;
  }
  else
  {
    b_allow_en_motors_change_ = false;
  }

  param_desc.name = general_params_[IDX_AUTO_START_ADDITIONAL_DELAY];
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  param_desc.description = "Added delay if auto start mode is enabled. You can add delay if your \
    TMCL program needs to run over 2 seconds.";
  param_desc.read_only = true;

  param_int_range.from_value = AUTO_START_ADDITIONAL_DELAY_DEFAULT;
  param_int_range.step = 1;
  param_int_range.to_value = AUTO_START_ADDITIONAL_DELAY_MAX;
  param_desc.integer_range.push_back(param_int_range);

  p_node_->declare_parameter(param_desc.name,AUTO_START_ADDITIONAL_DELAY_DEFAULT,param_desc);
  param_auto_start_additional_delay_ = p_node_->get_parameter(param_desc.name).as_int();
  param_desc.integer_range.clear();

  return;
}

bool TmclRos2::initAxisAndGlobalParameters()
{
  RCLCPP_INFO_STREAM(p_node_->get_logger(),"[TmclRos2::" <<  __func__ << "] called");
  bool b_success = false;
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  std::vector<int64_t> ap_type_default={};
  std::vector<std::string> ap_name_default={};
  std::vector<int64_t> gp_type_default={};
  std::vector<std::string> gp_name_default={};

  param_desc.name = general_params_[IDX_AXIS_PARAMETERS_TYPE];
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY;
  param_desc.description = "Array for Axis Parameter Types";
  param_desc.read_only = true;
  p_node_->declare_parameter(param_desc.name,ap_type_default,param_desc);
  param_ap_type_ = p_node_->get_parameter(param_desc.name).as_integer_array();

  param_desc.name = general_params_[IDX_AXIS_PARAMETERS_NAME];
  param_desc.type = rclcpp::ParameterType::PARAMETER_STRING_ARRAY;
  param_desc.description = "Array for Axis Parameter Names";
  param_desc.read_only = true;
  p_node_->declare_parameter(param_desc.name,ap_name_default,param_desc);
  param_ap_name_ = p_node_->get_parameter(param_desc.name).as_string_array();
  
  if(param_ap_name_.empty())
  {
    RCLCPP_ERROR_STREAM(p_node_->get_logger(),"Missing parameter: 'AP_name: []'");
    return b_success;
  }
  if(param_ap_type_.empty())
  {
    RCLCPP_ERROR_STREAM(p_node_->get_logger(),"Missing parameter: 'AP_type: []'");
    return b_success;
  }

  if(param_ap_type_.size() != param_ap_name_.size())
  {
    RCLCPP_ERROR_STREAM(p_node_->get_logger(),"AP_name has " << param_ap_name_.size() << \
      "  elements, while AP_type has " << param_ap_type_.size() << " elements");
    return b_success;
  }

  p_tmcl_interpreter_->setAp(param_ap_name_,param_ap_type_);

  param_desc.name = general_params_[IDX_GLOBAL_PARAMETERS_TYPE];
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY;
  param_desc.description = "Array for Global Parameter Types";
  param_desc.read_only = true;
  p_node_->declare_parameter(param_desc.name,gp_type_default,param_desc);
  param_gp_type_ = p_node_->get_parameter(param_desc.name).as_integer_array();

  param_desc.name = general_params_[IDX_GLOBAL_PARAMETERS_NAME];
  param_desc.type = rclcpp::ParameterType::PARAMETER_STRING_ARRAY;
  param_desc.description = "Array for Global Parameter Names";
  param_desc.read_only = true;
  p_node_->declare_parameter(param_desc.name,gp_name_default,param_desc);
  param_gp_name_ = p_node_->get_parameter(param_desc.name).as_string_array();

  if(param_gp_type_.empty())
  {
    RCLCPP_ERROR_STREAM(p_node_->get_logger(),"Missing parameter: 'GP_type: []'");
    return b_success;
  }
  if(param_gp_name_.empty())
  {
    RCLCPP_ERROR_STREAM(p_node_->get_logger(),"Missing parameter: 'GP_name: []'");
    return b_success;
  }

  if(param_gp_type_.size() != param_gp_name_.size())
  {
    RCLCPP_ERROR_STREAM(p_node_->get_logger(),"GP_name has " << param_gp_name_.size() << \
      "  elements, while GP_type has " << param_gp_type_.size() << " elements");
    return b_success;
  }

  p_tmcl_interpreter_->setGp(param_gp_name_,param_gp_type_);

  b_success = true;
  return b_success;
}

void TmclRos2::createMotor()
{
  RCLCPP_INFO_STREAM(p_node_->get_logger(),"[TmclRos2::" <<  __func__ << "] called");
  uint8_t module_type = 0;

  if(param_adhoc_mode_)
  {
    RCLCPP_WARN_STREAM(p_node_->get_logger(),"Running in adhoc mode.");
    for(uint16_t motor_num=0; motor_num < param_en_motors_.size(); motor_num++)
    {
      // If enabled motor
      if(param_en_motors_[motor_num] == 1)
      {
        RCLCPP_DEBUG_STREAM(p_node_->get_logger(),"Enabled motor"<< motor_num);
        p_motor_[motor_num] = new Motor(p_node_, p_tmcl_interpreter_, motor_num, module_number_);
        p_motor_[motor_num]->init();
      }
      else
      {
        RCLCPP_WARN_STREAM(p_node_->get_logger(),"Disabled motor"<< motor_num);
      }
    }
  }
  else
  {
    // Parse the hundreth digit of the module number.
    module_type=static_cast<uint8_t>( (module_number_ % 1000)/100 );
    if(TMCM_BLDC == module_type)
    {
      for(uint16_t motor_num=0; motor_num < param_en_motors_.size(); motor_num++)
      {
        // If enabled motor
        if(param_en_motors_[motor_num] == 1)
        {
          RCLCPP_DEBUG_STREAM(p_node_->get_logger(),"(BLDC) Enabled motor"<< motor_num);
          p_motor_[motor_num] = new BldcMotor(p_node_, p_tmcl_interpreter_, motor_num, module_number_);
          p_motor_[motor_num]->init();
        }
        else
        {
          RCLCPP_WARN_STREAM(p_node_->get_logger(),"(BLDC) Disabled motor"<< motor_num);
        }
      }
    }
    else // All other values are assumed TMCM_STEPPER
    {
      for(uint16_t motor_num=0; motor_num < param_en_motors_.size(); motor_num++)
      {
        // If enabled motor
        if(param_en_motors_[motor_num] == 1)
        {
          RCLCPP_DEBUG_STREAM(p_node_->get_logger(),"(Stepper) Enabled motor"<< motor_num);
          p_motor_[motor_num] = new StepperMotor(p_node_, p_tmcl_interpreter_, motor_num, module_number_);
          p_motor_[motor_num]->init();
        }
        else
        {
          RCLCPP_WARN_STREAM(p_node_->get_logger(),"(Stepper) Disabled motor"<< motor_num);
        }
      }
    }
  }
  return;
}

void TmclRos2::initServiceServer()
{
  std::string node_namespace = p_node_->get_namespace();
  // Remove unecessary "/" for empty namespace
  if(node_namespace.compare("/") == 0) // namespace is empty or "/"
  {
    node_namespace.clear();
  }
  std::string custom_cmd_srv_name = node_namespace + "/tmcl_custom_cmd";
  tmcl_custom_cmd_service_server_ = p_node_->create_service<tmcl_ros2::srv::TmcCustomCmd>\
    (custom_cmd_srv_name, std::bind(&TmclRos2::tmclCustomCmdCallback, this, std::placeholders::_1,\
    std::placeholders::_2));

  std::string gap_srv_name = node_namespace + "/tmcl_gap_all";
  tmcl_gap_service_server_ = p_node_->create_service<tmcl_ros2::srv::TmcGapAll>\
    (gap_srv_name, std::bind(&TmclRos2::tmclGapAllCallback, this, std::placeholders::_1, \
    std::placeholders::_2));

  std::string ggp_srv_name = node_namespace + "/tmcl_ggp_all";
  tmcl_ggp_service_server_ = p_node_->create_service<tmcl_ros2::srv::TmcGgpAll>\
    (ggp_srv_name, std::bind(&TmclRos2::tmclGgpAllCallback, this, std::placeholders::_1, \
    std::placeholders::_2));
}

void TmclRos2::tmclCustomCmdCallback(const std::shared_ptr<tmcl_ros2::srv::TmcCustomCmd::Request> \
  req, const std::shared_ptr<tmcl_ros2::srv::TmcCustomCmd::Response> res)
{
  int32_t val=0;
  uint8_t motor_num = static_cast<uint8_t>(req->motor_num);

  res->result = false;
  if(tmcl_custom_cmd_[IDX_SAP] == req->instruction)
  {
    val = req->value;
    RCLCPP_DEBUG(p_node_->get_logger(), "Setting Axis Parameter");
    if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_SAP, req->instruction_type, motor_num, &val))
    {
      res->output = val;
      res->result= true;
    }
    else
    {
      RCLCPP_ERROR_STREAM(p_node_->get_logger(),"Fail to Set Axis Parameter");
    }
  }
  else if(tmcl_custom_cmd_[IDX_GAP] == req->instruction)
  {
    RCLCPP_DEBUG(p_node_->get_logger(), "Getting Axis Parameter Value");
    if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, req->instruction_type, motor_num, &val))
    {
      res->output = val;
      res->result= true;
    }
    else
    {
      RCLCPP_ERROR_STREAM(p_node_->get_logger(),"Fail to Get Axis Parameter");
    }
  }
  else if(tmcl_custom_cmd_[IDX_SGP] == req->instruction)
  {
    val = req->value;
    RCLCPP_DEBUG(p_node_->get_logger(), "Setting Global Parameter");
    if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_SGP, req->instruction_type, motor_num, &val))
    {
      res->output = val;
      res->result= true;
    }
    else
    {
      RCLCPP_ERROR_STREAM(p_node_->get_logger(),"Fail to Set Global Parameter");
    }
  }
  else if(tmcl_custom_cmd_[IDX_GGP] == req->instruction)
  {
    RCLCPP_DEBUG(p_node_->get_logger(), "Getting Global Parameter Value");
    if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GGP, req->instruction_type, motor_num, &val))
    {
      res->output = val;
      res->result= true;
    }
    else
    {
      RCLCPP_ERROR_STREAM(p_node_->get_logger(),"Fail to Get Global Parameter");
    }
  }
  else
  {
    RCLCPP_WARN_STREAM(p_node_->get_logger(), req->instruction.c_str() << " is unsupported.");
  }
}

void TmclRos2::tmclGapAllCallback(const std::shared_ptr<tmcl_ros2::srv::TmcGapAll::Request> req, \
  const std::shared_ptr<tmcl_ros2::srv::TmcGapAll::Response> res)
{
  uint16_t total_motors = 0;

  total_motors = module_number_ / 1000;
  res->param.resize(param_ap_name_.size());

  if(req->motor_num < total_motors)
  {
    res->success = true;
    RCLCPP_INFO_STREAM(p_node_->get_logger(),"Getting all Axis Parameters for motor" \
      << req->motor_num);
    for(uint32_t i = 0; i < param_ap_type_.size(); i++)
    {
      int32_t val=0;
      if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, param_ap_type_[i], req->motor_num, &val))
      {
        res->param[i].name = param_ap_name_[i];
        res->param[i].value = val;
      }
      else
      {
        res->success = false;
        res->param[i].name = param_ap_name_[i] + ">>> ERROR";
        RCLCPP_ERROR_STREAM(p_node_->get_logger(),"Fail to Axis Parameter '" << param_ap_name_[i] \
          <<"' for motor" << req->motor_num);
      }
    }
  }
  else
  {
    res->success = false;
  }
}

void TmclRos2::tmclGgpAllCallback(const std::shared_ptr<tmcl_ros2::srv::TmcGgpAll::Request> req, \
  const std::shared_ptr<tmcl_ros2::srv::TmcGgpAll::Response> res)
{
  (void)req; // unused

  res->param.resize(param_gp_name_.size());
  RCLCPP_INFO_STREAM(p_node_->get_logger(),"Getting all Global Parameters");

  res->success = true;
  for(uint32_t i = 0; i < param_gp_type_.size(); i++)
  {
    int32_t val=0;
    if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GGP, param_gp_type_[i], 0, &val))
    {
      res->param[i].name = param_gp_name_[i];
      res->param[i].value = val;
    }
    else
    {
      res->success = false;
      res->param[i].name = param_gp_name_[i] + ">>> ERROR";
      RCLCPP_ERROR_STREAM(p_node_->get_logger(),"Fail to Global Parameter '" << param_gp_name_[i] <<"'");
    }
  }
}

bool TmclRos2::deInit()
{
  RCLCPP_INFO_STREAM(p_node_->get_logger(),"[TmclRos2::" <<  __func__ << "] called");
  bool b_success=false;
  if(p_node_->has_parameter(general_params_[IDX_EN_MOTORS]))
  {
    int32_t val = 0;

    if(!this->getRetriesExceededStatus())
    {
      for(uint8_t index = 0; index < param_en_motors_.size(); index++)
      {
        if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_MST, static_cast<uint8_t>(0x00), index, &val))
        {
          RCLCPP_INFO(p_node_->get_logger(),"[%s] Stopped motor %d before shutting down...", __func__, index);
        }
        else
        {
          RCLCPP_WARN(p_node_->get_logger(),"[%s] Stopping of motor failed", __func__);
        }
      }
    }
    else
    {
      RCLCPP_ERROR_STREAM(p_node_->get_logger(),"[TmclRos2::" <<  __func__ << "] timeout reached (board unresponsive), skipping motor stop commands");
    }
  }

  if(p_tmcl_interpreter_ != nullptr && p_tmcl_interpreter_->shutdownInterface())
  {
    b_success=true;
    RCLCPP_INFO_STREAM(p_node_->get_logger(),"[TmclRos2::" <<  __func__ << "] shutdownInterface() success");
  }
  else
  {
    RCLCPP_ERROR_STREAM(p_node_->get_logger(),"[TmclRos2::" <<  __func__ << "] shutdownInterface() failed");

  }
  return b_success;
}

bool TmclRos2::getRetriesExceededStatus()
{
  bool b_success = true;

  if(p_tmcl_interpreter_ != nullptr)
  {
    b_success = p_tmcl_interpreter_->getRetriesExceededStatus();
  }

  return b_success;
}
