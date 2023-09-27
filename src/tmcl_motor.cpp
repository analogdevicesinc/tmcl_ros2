/**
 * Copyright (c) 2023 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tmcl_ros2/tmcl_motor.h"

using std::placeholders::_1;

Motor::Motor(rclcpp::Node::SharedPtr p_node, TmclInterpreter* p_tmcl_interpreter,
  uint8_t motor_number, uint32_t module_number):
  p_node_(p_node),
  p_tmcl_interpreter_(p_tmcl_interpreter),
  module_number_(module_number),
  motor_number_(motor_number)
{
  RCLCPP_INFO_STREAM(p_node_->get_logger(),this->getMotorName() << " [Motor::" << __func__ << "]");
  param_comm_interface_name_ = p_node_->get_parameter(\
    comm_interface_params_[IDX_COMM_INTERFACE_NAME]).as_string();
}

Motor::~Motor()
{
  RCLCPP_INFO_STREAM(p_node_->get_logger(),this->getMotorName() << " [Motor::" << __func__ << "]");
  p_tmcl_interpreter_ = nullptr;
  p_node_ = nullptr;
}

void Motor::init()
{
  RCLCPP_INFO_STREAM(p_node_->get_logger(),this->getMotorName() << " [Motor::" << __func__ << "]");
  this->initMotorParams();
  this->initPublisherParams();
  this->initPublisher();
  this->initSubscriberParams();
  this->initSubscribers();
  RCLCPP_INFO_STREAM(p_node_->get_logger(),this->getMotorName() << "[Motor::" << __func__ << \
    "] Initialized");
}

void Motor::initMotorParams()
{
  RCLCPP_INFO_STREAM(p_node_->get_logger(),this->getMotorName() << " [Motor::" << __func__ << "]");
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  rcl_interfaces::msg::IntegerRange param_int_range;
  param_desc.name = this->getMotorName() + "." + hardware_params_[IDX_WHEEL_DIAMETER];
  param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
  param_desc.description = "Wheel diameter";
  param_desc.read_only = true;
  p_node_->declare_parameter(param_desc.name,0.0,param_desc);
  param_wheel_diameter_ = p_node_->get_parameter(param_desc.name).as_double();

  param_desc.name = this->getMotorName() + "." + additional_ratio_params_[IDX_ADDITIONAL_RATIO_VEL];
  param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
  param_desc.description = "Additional Ratio for Velocity Command";
  param_desc.read_only = true;
  p_node_->declare_parameter(param_desc.name,1.0,param_desc);
  param_additional_ratio_vel_ = p_node_->get_parameter(param_desc.name).as_double();

  param_desc.name = this->getMotorName() + "." + additional_ratio_params_[IDX_ADDITIONAL_RATIO_POS];
  param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
  param_desc.description = "Additional Ratio for Position Command";
  param_desc.read_only = true;
  p_node_->declare_parameter(param_desc.name,1.0,param_desc);
  param_additional_ratio_pos_ = p_node_->get_parameter(param_desc.name).as_double();

  param_desc.name = this->getMotorName() + "." + additional_ratio_params_[IDX_ADDITIONAL_RATIO_TRQ];
  param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
  param_desc.description = "Additional Ratio for Torque Command";
  param_desc.read_only = true;
  p_node_->declare_parameter(param_desc.name,1.0,param_desc);
  param_additional_ratio_trq_  = p_node_->get_parameter(param_desc.name).as_double();

  return;
}

void Motor::initPublisherParams()
{
  RCLCPP_INFO_STREAM(p_node_->get_logger(),this->getMotorName() << " [Motor::" << __func__ << "]");
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  rcl_interfaces::msg::IntegerRange param_int_range;

  param_desc.name = this->getMotorName() + "." + ros_topic_params_[IDX_EN_PUB_TMC_INFO];
  param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
  param_desc.description = "Enables/disables publishing of TMC information";
  param_desc.read_only = true;
  p_node_->declare_parameter(param_desc.name,true,param_desc);
  param_en_pub_tmc_info_ = p_node_->get_parameter(param_desc.name).as_bool();

  if(param_en_pub_tmc_info_)
  {
    param_desc.name = this->getMotorName() + "." + ros_topic_params_[IDX_TMC_INFO_TOPIC];
    param_desc.type = rclcpp::ParameterType::PARAMETER_STRING;
    param_desc.description = "Topic name for TMC Information publisher";
    param_desc.read_only = true;
    std::string param_default_str = "/tmc_info_" + std::to_string(motor_number_);
    p_node_->declare_parameter(param_desc.name,param_default_str,param_desc);
    param_tmc_info_topic_ = p_node_->get_parameter(param_desc.name).as_string();

    param_desc.name = this->getMotorName() + "." + ros_topic_params_[IDX_PUB_RATE_TMC_INFO];
    param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
    param_desc.description = "Publish rate (Hz) of TMC Information";
    param_desc.read_only = true;

    param_int_range.from_value = PUB_RATE_MIN;
    param_int_range.step = 1;
    param_int_range.to_value = PUB_RATE_MAX;
    param_desc.integer_range.push_back(param_int_range);

    p_node_->declare_parameter(param_desc.name,PUB_RATE_DEFAULT,param_desc);
    param_desc.integer_range.clear();
    param_pub_rate_tmc_info_ = p_node_->get_parameter(param_desc.name).as_int();

    param_desc.name = this->getMotorName() + "." + ros_topic_params_[IDX_PUB_ACTUAL_VEL];
    param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
    param_desc.description = "\n \
      True - include actual velocity in TMC Information \n \
      False - exclude actual velocity in TMC Information";
    param_desc.read_only = true;

    p_node_->declare_parameter(param_desc.name,true,param_desc);
    param_pub_actual_vel_ = p_node_->get_parameter(param_desc.name).as_bool();


    param_desc.name = this->getMotorName() + "." + ros_topic_params_[IDX_PUB_ACTUAL_TRQ];
    param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
    param_desc.description = "\n \
      True - include actual torque in TMC Information \n \
      False - exclude actual torque in TMC Information";
    param_desc.read_only = true;

    p_node_->declare_parameter(param_desc.name,true,param_desc);
    param_pub_actual_trq_ = p_node_->get_parameter(param_desc.name).as_bool();

    param_desc.name = this->getMotorName() + "." + ros_topic_params_[IDX_PUB_ACTUAL_POS];
    param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
    param_desc.description = "\n \
      True - include actual position in TMC Information \n \
      False - exclude actual position in TMC Information";
    param_desc.read_only = true;

    p_node_->declare_parameter(param_desc.name,true,param_desc);
    param_pub_actual_pos_ = p_node_->get_parameter(param_desc.name).as_bool();

    // Set Frame ID
    std::string node_namespace = p_node_->get_namespace();
    // Remove unecessary "/"
    if(node_namespace.compare("/") == 0) // namespace is empty or "/"
    {
      tmc_info_frame_id_ = "tmcm" + std::to_string(module_number_) + "_mtr" + \
        std::to_string(this->getMotorNumber()) + "_frame";
    }
    else
    {
      node_namespace.erase(node_namespace.begin());
      tmc_info_frame_id_ = node_namespace + "/tmcm" + std::to_string(module_number_) + "_mtr" + \
        std::to_string(this->getMotorNumber()) + "_frame";
    }
  }
  else
  {
    RCLCPP_WARN_STREAM(p_node_->get_logger(), p_node_->get_name() << "/tmc_info_" << motor_number_
      << " will not be published.");
  }

  return;
}

void Motor::initPublisher()
{
  RCLCPP_INFO_STREAM(p_node_->get_logger(),this->getMotorName() << " [Motor::" << __func__ << "]");
  int period_ms = 0;
  if(param_en_pub_tmc_info_)
  {
    publisher_ = p_node_->create_publisher<tmcl_ros2::msg::TmcInfo>(param_tmc_info_topic_, 10);

    period_ms = (1000/param_pub_rate_tmc_info_);
    RCLCPP_DEBUG_STREAM(p_node_->get_logger(),"rate= " << param_pub_rate_tmc_info_ <<
      "; period_ms= " << period_ms);
    publisher_timer_ = p_node_->create_wall_timer(std::chrono::milliseconds(period_ms),
      std::bind(&Motor::pubTimerCallback, this));
  }
  else
  {
    RCLCPP_WARN_STREAM(p_node_->get_logger(), p_node_->get_name() << "/tmc_info_" << motor_number_
      << " not published.");
  }

  return;
}

void Motor::pubTimerCallback()
{
  RCLCPP_DEBUG_STREAM(p_node_->get_logger(),this->getMotorName() << " [Motor::" << __func__ << "]");
  int32_t val = 0;
  auto message = tmcl_ros2::msg::TmcInfo();

  message.header.stamp = p_node_->now();
  message.header.frame_id = tmc_info_frame_id_;
  message.interface_name = param_comm_interface_name_;
  message.motor_num = static_cast<int>(this->getMotorNumber());
  /* Initialize message to 0 first */
  message.board_voltage = 0.0;
  message.status_flag = 0;
  message.velocity = 0.0;
  message.position = 0;
  message.torque = 0;

  if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "SupplyVoltage", this->getMotorNumber(), &val))
  {
    RCLCPP_DEBUG(p_node_->get_logger(), "SupplyVoltage 0x%02x",val);
    message.board_voltage = val / 10; // Convert mV to V
  }
  else
  {
    RCLCPP_ERROR_STREAM_ONCE(p_node_->get_logger(),"Fail to get SupplyVoltage");
  }

  if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "StatusFlags", this->getMotorNumber(), &val))
  {
    RCLCPP_DEBUG(p_node_->get_logger(), "StatusFlags: 0x%02x", val);
    message.status_flag = val;
  }
  else
  {
    RCLCPP_ERROR_STREAM_ONCE(p_node_->get_logger(),"Fail to get StatusFlags");
  }

  if(param_pub_actual_vel_)
  {
    if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "ActualVelocity", this->getMotorNumber(),
        &val))
    {
      RCLCPP_DEBUG(p_node_->get_logger(), "ActualVelocity 0x%02x",val);
      message.velocity = val * param_additional_ratio_vel_;
    }
    else
    {
      RCLCPP_ERROR_STREAM_ONCE(p_node_->get_logger(),this->getMotorName() <<
        " Fail to get ActualVelocity");
    }
  }

  if(param_pub_actual_pos_)
  {
    if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "ActualPosition", this->getMotorNumber(),
        &val))
    {
      RCLCPP_DEBUG(p_node_->get_logger(), "ActualPosition 0x%02x",val);
      message.position = static_cast<int32_t>(val * param_additional_ratio_pos_);
    }
    else
    {
      RCLCPP_ERROR_STREAM_ONCE(p_node_->get_logger(),this->getMotorName() <<
        " Fail to get ActualPosition");
    }
  }

  if(param_pub_actual_trq_)
  {
    if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "ActualTorque", this->getMotorNumber(), &val))
    {
      RCLCPP_DEBUG(p_node_->get_logger(), "ActualTorque 0x%02x",val);
      message.torque = static_cast<int32_t>(val * param_additional_ratio_trq_);
    }
    else
    {
      RCLCPP_ERROR_STREAM_ONCE(p_node_->get_logger(),this->getMotorName() <<
        " Fail to get ActualTorque");
    }
  }

  publisher_->publish(message);
}

void Motor::initSubscriberParams()
{
  RCLCPP_INFO_STREAM(p_node_->get_logger(),this->getMotorName() << " [Motor::" << __func__ << "]");
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  rcl_interfaces::msg::IntegerRange param_int_range;
  std::string param_default_str = "";

  param_desc.name = this->getMotorName() + "." + ros_topic_params_[IDX_TMC_CMD_VEL_TOPIC];
  param_desc.type = rclcpp::ParameterType::PARAMETER_STRING;
  param_desc.description = "Twist topics that will be the source of target velocity \
    to be set on the TMC";
  param_desc.read_only = true;
  param_default_str = "/cmd_vel_" + std::to_string(this->getMotorNumber());
  p_node_->declare_parameter(param_desc.name,param_default_str,param_desc);
  param_tmc_cmd_vel_topic_ = p_node_->get_parameter(param_desc.name).as_string();

  param_desc.name = this->getMotorName() + "." + ros_topic_params_[IDX_TMC_CMD_ABSPOS_TOPIC];
  param_desc.type = rclcpp::ParameterType::PARAMETER_STRING;
  param_desc.description = "Int32 topics that will be the source of target position \
    to be set on the TMC";
  param_desc.read_only = true;
  param_default_str = "/cmd_abspos_" + std::to_string(this->getMotorNumber());
  p_node_->declare_parameter(param_desc.name,param_default_str,param_desc);
  param_tmc_cmd_abspos_topic_ = p_node_->get_parameter(param_desc.name).as_string();

  param_desc.name = this->getMotorName() + "." + ros_topic_params_[IDX_TMC_CMD_RELPOS_TOPIC];
  param_desc.type = rclcpp::ParameterType::PARAMETER_STRING;
  param_desc.description = "Int32 topics that will be the source of target position \
    to be set on the TMC";
  param_desc.read_only = true;
  param_default_str = "/cmd_relpos_" + std::to_string(this->getMotorNumber());
  p_node_->declare_parameter(param_desc.name,param_default_str,param_desc);
  param_tmc_cmd_relpos_topic_ = p_node_->get_parameter(param_desc.name).as_string();

  param_desc.name = this->getMotorName() + "." + ros_topic_params_[IDX_TMC_CMD_TRQ_TOPIC];
  param_desc.type = rclcpp::ParameterType::PARAMETER_STRING;
  param_desc.description = "Int32 topics that will be the source of target torque \
    to be set on the TMC";
  param_desc.read_only = true;
  param_default_str = "/cmd_trq_" + std::to_string(this->getMotorNumber());
  p_node_->declare_parameter(param_desc.name,param_default_str,param_desc);
  param_tmc_cmd_trq_topic_ = p_node_->get_parameter(param_desc.name).as_string();
}

void Motor::initSubscribers()
{
  RCLCPP_INFO_STREAM(p_node_->get_logger(),this->getMotorName() << " [Motor::" << __func__ << "]");
  subscription_cmd_vel_=p_node_->create_subscription<geometry_msgs::msg::Twist>(
    param_tmc_cmd_vel_topic_, 10, std::bind(&Motor::cmdVelSubscriberCallback, this, _1));
  /* Print unit */
  RCLCPP_INFO(p_node_->get_logger(),"========================================");
  RCLCPP_INFO(p_node_->get_logger(),"Subscribed to %s:", param_tmc_cmd_vel_topic_.c_str());
  RCLCPP_INFO_STREAM(p_node_->get_logger(),"  Velocity unit: Refer to datasheet");

  subscription_cmd_abspos_=p_node_->create_subscription<std_msgs::msg::Int32>(
    param_tmc_cmd_abspos_topic_, 10, std::bind(&Motor::cmdAbsposSubscriberCallback, this, _1));

  subscription_cmd_relpos_=p_node_->create_subscription<std_msgs::msg::Int32>(
    param_tmc_cmd_relpos_topic_, 10, std::bind(&Motor::cmdRelposSubscriberCallback, this, _1));
  /* Print unit */
  RCLCPP_INFO(p_node_->get_logger(),"========================================");
  RCLCPP_INFO(p_node_->get_logger(),"Subscribed to %s and %s:",
    param_tmc_cmd_abspos_topic_.c_str(), param_tmc_cmd_relpos_topic_.c_str());
  RCLCPP_INFO_STREAM(p_node_->get_logger(),"  Position unit: Refer to datasheet");

  subscription_cmd_trq_=p_node_->create_subscription<std_msgs::msg::Int32>(
    param_tmc_cmd_trq_topic_, 10, std::bind(&Motor::cmdTrqSubscriberCallback, this, _1));
  /* Print unit */
  RCLCPP_INFO(p_node_->get_logger(),"========================================");
  RCLCPP_INFO(p_node_->get_logger(),"Subscribed to %s:", param_tmc_cmd_trq_topic_.c_str());
  RCLCPP_INFO_STREAM(p_node_->get_logger(),"  Torque unit: Refer to datasheet");
  RCLCPP_INFO(p_node_->get_logger(),"========================================");
}

void Motor::cmdVelSubscriberCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  RCLCPP_DEBUG_STREAM(p_node_->get_logger(),this->getMotorName() << " [Motor::" << __func__ << "]");
  float val = msg->linear.x;
  int32_t board_val = 0;
  tmcl_cmd_t vel_cmd = TMCL_CMD_ROR;

  board_val = val / param_additional_ratio_vel_;

  RCLCPP_DEBUG_STREAM(p_node_->get_logger(), this->getMotorName() << "Setting cmd_vel, received: "
    << val << " board_val: " << board_val);

  if(val < 0)
  {
    board_val = board_val * -1; // Change sign to positive
    vel_cmd = TMCL_CMD_ROL; // Rotate Left
  }
  else
  {
    vel_cmd = TMCL_CMD_ROR; // Rotate Right
  }

  if(p_tmcl_interpreter_->executeCmd(vel_cmd, static_cast<uint8_t>(0x00), this->getMotorNumber(),
      &board_val))
  {
    RCLCPP_DEBUG_STREAM(p_node_->get_logger(),"\nSubscriber callback " << __func__  <<
      " exited successfully");
  }
  else
  {
    RCLCPP_ERROR_STREAM(p_node_->get_logger(),"Fail to set TargetVelocity");
  }
  return;
}

void Motor::cmdRelposSubscriberCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  RCLCPP_DEBUG_STREAM(p_node_->get_logger(),this->getMotorName() << "[Motor::" << __func__ << "]");
  int32_t val = msg->data;

  val = val / param_additional_ratio_pos_;

  RCLCPP_DEBUG_STREAM(p_node_->get_logger(), this->getMotorName() << "Setting cmd_relpos, " <<
    "received: "<< msg->data << " board_val: " << val);
  const uint8_t REL = 1;
  if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_MVP, REL, this->getMotorNumber(), &val))
  {
    RCLCPP_DEBUG_STREAM(p_node_->get_logger(),"\nSubscriber callback " << __func__  <<
      " exited successfully");
  }
  else
  {
    RCLCPP_ERROR_STREAM(p_node_->get_logger(),"Fail to set Relative TargetPosition");
  }
  return;
}

void Motor::cmdAbsposSubscriberCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  RCLCPP_DEBUG_STREAM(p_node_->get_logger(),this->getMotorName() << " [Motor::" << __func__ << "]");
  int32_t val = msg->data;

  val = val / param_additional_ratio_pos_;

  RCLCPP_DEBUG_STREAM(p_node_->get_logger(), this->getMotorName() << "Setting cmd_abspos, " <<
    "received: "<< msg->data << " board_val: " << val);

  const uint8_t ABS = 0;
  if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_MVP, ABS, this->getMotorNumber(), &val))
  {
    RCLCPP_DEBUG_STREAM(p_node_->get_logger(),"\nSubscriber callback " << __func__  <<
      " exited successfully");
  }
  else
  {
    RCLCPP_ERROR_STREAM(p_node_->get_logger(),"Fail to set Absolute TargetPosition");
  }
  return;
}

void Motor::cmdTrqSubscriberCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  RCLCPP_DEBUG_STREAM(p_node_->get_logger(),this->getMotorName() << " [Motor::" << __func__ << "]");
  int32_t val = msg->data;

  val = val / param_additional_ratio_trq_;
  RCLCPP_DEBUG_STREAM(p_node_->get_logger(), this->getMotorName() << "Setting cmd_trq, "
    "received: "<< msg->data << " board_val: " << val);

  if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_SAP, "TargetTorque", this->getMotorNumber(), &val))
  {
    RCLCPP_DEBUG_STREAM(p_node_->get_logger(),"\nSubscriber callback " << __func__  <<
      " exited successfully");
  }
  else
  {
    RCLCPP_ERROR_STREAM(p_node_->get_logger(),"Fail to set TargetTorque");
  }
  return;
}

std::string Motor::getMotorName()
{
  std::string name;
  name = "motor" + std::to_string(motor_number_);

  return name;
}

uint8_t Motor::getMotorNumber()
{
  return motor_number_;
}
