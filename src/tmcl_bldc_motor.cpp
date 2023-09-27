/**
 * Copyright (c) 2023 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tmcl_ros2/tmcl_bldc_motor.h"

using std::placeholders::_1;

BldcMotor::BldcMotor(rclcpp::Node::SharedPtr p_node, TmclInterpreter* p_tmcl_interpreter,
  uint8_t motor_number, uint32_t module_number) :
  Motor(p_node,p_tmcl_interpreter,motor_number, module_number)
{
  RCLCPP_INFO_STREAM(p_node_->get_logger(),this->getMotorName() << " [BldcMotor::"
    << __func__ << "]");
  comm_mode_ = BLDC_MODE_DISABLED;
  encoder_steps_ = 0;
  position_scaler_m_ = 0;
  comm_mode_=this->getCommutationMode();
}

BldcMotor::~BldcMotor()
{
  RCLCPP_INFO_STREAM(p_node_->get_logger(),this->getMotorName() << " [BldcMotor::"
    << __func__ << "]");
}

void BldcMotor::init()
{
  RCLCPP_INFO_STREAM(p_node_->get_logger(),this->getMotorName() << " [BldcMotor::"
    << __func__ << "]");
  Motor::initMotorParams();
  Motor::initPublisherParams();
  this->initPublisher();

  if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "PositionScalerM", this->getMotorNumber(),
      &position_scaler_m_))
  {
    RCLCPP_DEBUG(p_node_->get_logger(), "PositionScalerM 0x%02x",position_scaler_m_);
  }
  else
  {
    position_scaler_m_ = 0;
    RCLCPP_WARN_STREAM(p_node_->get_logger(),"Fail to get PositionScalerM; Setting to 0.");
    if(comm_mode_ >= BLDC_MODE_CLOSEDLOOP)
    {
      if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "EncoderSteps", this->getMotorNumber(),
          &encoder_steps_))
      {
        RCLCPP_DEBUG(p_node_->get_logger(), "EncoderSteps 0x%02x",encoder_steps_);
      }
      else
      {
        encoder_steps_ = 0;
        RCLCPP_WARN_STREAM(p_node_->get_logger(),"Fail to get EncoderSteps; Setting to 0");
      }
    }
  }

  this->initSubscriberParams();
  this->initSubscribers();

  RCLCPP_INFO_STREAM(p_node_->get_logger(),this->getMotorName() << "[BldcMotor::" << __func__ <<
    "] Initialized");
  return;
}

void BldcMotor::initPublisher()
{
  RCLCPP_INFO_STREAM(p_node_->get_logger(),this->getMotorName() << " [BldcMotor::"
    << __func__ << "]");
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  int period_ms = 0;

  if(!p_node_->has_parameter("StatusFlags.RegName"))
  {
    param_desc.name = "StatusFlags.RegName";
    param_desc.type = rclcpp::ParameterType::PARAMETER_STRING_ARRAY;
    param_desc.description = "Register name arry of status flags";
    param_desc.additional_constraints.clear();
    param_desc.read_only = true;

    std::vector<std::string> default_reg_name={};
    p_node_->declare_parameter(param_desc.name,default_reg_name,param_desc);
  }
  param_statusflags_regname_ = p_node_->get_parameter("StatusFlags.RegName").as_string_array();
  if(!p_node_->has_parameter("StatusFlags.RegShift"))
  {
    param_desc.name = "StatusFlags.RegShift";
    param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY;
    param_desc.description = "Register shift array of status flags";
    param_desc.additional_constraints.clear();
    param_desc.read_only = true;

    std::vector<int64_t> default_reg_shift={};
    p_node_->declare_parameter(param_desc.name,default_reg_shift,param_desc);
  }
  param_statusflags_regshift_ = p_node_->get_parameter("StatusFlags.RegShift").as_integer_array();

  if(param_en_pub_tmc_info_)
  {
    publisher_ = p_node_->create_publisher<tmcl_ros2::msg::TmcInfo>(param_tmc_info_topic_, 10);

    period_ms = (1000/param_pub_rate_tmc_info_);
    RCLCPP_DEBUG_STREAM(p_node_->get_logger(),"rate= " << std::to_string(param_pub_rate_tmc_info_)
      << "; period_ms= " << period_ms);
    publisher_timer_ = p_node_->create_wall_timer(std::chrono::milliseconds(period_ms),
      std::bind(&BldcMotor::pubTimerCallback, this));
  }
  else
  {
    RCLCPP_WARN_STREAM(p_node_->get_logger(), p_node_->get_name() << "/tmc_info_" <<
      this->getMotorNumber() << " not published.");
  }

  return;
}

void BldcMotor::pubTimerCallback()
{
  RCLCPP_DEBUG_STREAM(p_node_->get_logger(),this->getMotorName() << " [BldcMotor::"
    << __func__ << "]");
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

  uint16_t len_of_regshift = param_statusflags_regshift_.size();
  for(uint16_t i = 0; i < len_of_regshift; i++)
  {
    int bit_mask = 1 << param_statusflags_regshift_[i];
    if(bit_mask == (bit_mask & message.status_flag))
    {
      message.status.append("[" + param_statusflags_regname_[i] + "] ");
    }
  }

  if(param_pub_actual_vel_)
  {
    if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "ActualVelocity", this->getMotorNumber(),
        &val))
    {
      RCLCPP_DEBUG(p_node_->get_logger(), "ActualVelocity 0x%02x",val);
      if(0.0 == param_wheel_diameter_)
      {
        message.velocity = val * param_additional_ratio_vel_;
      }
      else
      {
        message.velocity = val * (((PI * param_wheel_diameter_) / SEC_TO_MIN) *
          param_additional_ratio_vel_);
      }
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
      if(position_scaler_m_ > 0)
      {
        message.position = val * ((ANGULAR_FULL_ROTATION / (float)position_scaler_m_)
          * param_additional_ratio_pos_);
      }
      else if(encoder_steps_ > 0)
      {
        message.position = val * ((ANGULAR_FULL_ROTATION / (float)encoder_steps_)
          * param_additional_ratio_pos_);
      }
      else
      {
        message.position = val * param_additional_ratio_pos_;
      }
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
      message.torque = val * param_additional_ratio_trq_;
    }
    else
    {
      RCLCPP_ERROR_STREAM_ONCE(p_node_->get_logger(),this->getMotorName() <<
        " Fail to get ActualTorque");
    }
  }

  publisher_->publish(message);
}

tmcl_bldc_comm_mode_t BldcMotor::getCommutationMode()
{
  RCLCPP_DEBUG_STREAM(p_node_->get_logger(),this->getMotorName() << " [BldcMotor::"
    << __func__ << "]");
  tmcl_bldc_comm_mode_t comm_mode = BLDC_MODE_DISABLED;
  int32_t val;

  if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "CommutationMode", this->getMotorNumber(), &val))
  {
    comm_mode = static_cast<tmcl_bldc_comm_mode_t>(val);
    RCLCPP_INFO_STREAM(p_node_->get_logger(), "CommutationMode: " << comm_mode);
  }
  else
  {
    RCLCPP_ERROR_STREAM(p_node_->get_logger(),"Fail to get CommutationMode. Setting to disabled.");
  }

  return comm_mode;
}

void BldcMotor::initSubscriberParams()
{
  RCLCPP_INFO_STREAM(p_node_->get_logger(),this->getMotorName() << " [BldcMotor::"
    << __func__ << "]");
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  rcl_interfaces::msg::IntegerRange param_int_range;
  std::string param_default_str = "";

  if(comm_mode_ == BLDC_MODE_DISABLED)
  {
    RCLCPP_WARN_STREAM(p_node_->get_logger(),"Commutation Mode is DISABLED; No subscriber params "
      << "to initialize.");
  }
  else if(comm_mode_ >= BLDC_MODE_OPENLOOP)
  {
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

    if(comm_mode_ >= BLDC_MODE_CLOSEDLOOP)
    {
      param_desc.name = this->getMotorName() + "." + ros_topic_params_[IDX_TMC_CMD_TRQ_TOPIC];
      param_desc.type = rclcpp::ParameterType::PARAMETER_STRING;
      param_desc.description = "Int32 topics that will be the source of target torque \
        to be set on the TMC";
      param_desc.read_only = true;
      param_default_str = "/cmd_trq_" + std::to_string(this->getMotorNumber());
      p_node_->declare_parameter(param_desc.name,param_default_str,param_desc);
      param_tmc_cmd_trq_topic_ = p_node_->get_parameter(param_desc.name).as_string();
    }
  }

}

void BldcMotor::initSubscribers()
{
  RCLCPP_INFO_STREAM(p_node_->get_logger(),this->getMotorName() << " [BldcMotor::"
    << __func__ << "]");
  if(comm_mode_ == BLDC_MODE_DISABLED)
  {
    RCLCPP_WARN_STREAM(p_node_->get_logger(),"Commutation Mode is DISABLED; No subscriber " <<
      "params to initialize.");
  }
  else if(comm_mode_ >= BLDC_MODE_OPENLOOP)
  {
    subscription_cmd_vel_=p_node_->create_subscription<geometry_msgs::msg::Twist>(
      param_tmc_cmd_vel_topic_, 10, std::bind(&BldcMotor::cmdVelSubscriberCallback, this, _1));
    /* Print unit */
    RCLCPP_INFO(p_node_->get_logger(),"========================================");
    RCLCPP_INFO(p_node_->get_logger(),"Subscribed to %s:", param_tmc_cmd_vel_topic_.c_str());
    if(0.0 == param_wheel_diameter_)
    {
      RCLCPP_INFO_STREAM(p_node_->get_logger(),"  Velocity unit: rpm");
    }
    else
    {
      RCLCPP_INFO_STREAM(p_node_->get_logger(),"  Velocity unit: m/s");
    }

    subscription_cmd_abspos_=p_node_->create_subscription<std_msgs::msg::Int32>(
      param_tmc_cmd_abspos_topic_, 10, std::bind(&BldcMotor::cmdAbsposSubscriberCallback,
      this, _1));

    subscription_cmd_relpos_=p_node_->create_subscription<std_msgs::msg::Int32>(
      param_tmc_cmd_relpos_topic_, 10, std::bind(&BldcMotor::cmdRelposSubscriberCallback,
      this, _1));

    /* Print unit */
    RCLCPP_INFO(p_node_->get_logger(),"========================================");
    RCLCPP_INFO(p_node_->get_logger(),"Subscribed to %s and %s:",
      param_tmc_cmd_abspos_topic_.c_str(), param_tmc_cmd_relpos_topic_.c_str());
    if(position_scaler_m_ == 0 && encoder_steps_ == 0)
    {
      RCLCPP_INFO_STREAM(p_node_->get_logger(),"  Position unit: pulses");
    }
    else
    {
      RCLCPP_INFO_STREAM(p_node_->get_logger(),"  Position unit: angular degrees");
    }

    if(comm_mode_ >= BLDC_MODE_CLOSEDLOOP)
    {
      subscription_cmd_trq_=p_node_->create_subscription<std_msgs::msg::Int32>(
          param_tmc_cmd_trq_topic_, 10, std::bind(&BldcMotor::cmdTrqSubscriberCallback, this, _1));
      /* Print unit */
      RCLCPP_INFO(p_node_->get_logger(),"========================================");
      RCLCPP_INFO(p_node_->get_logger(),"Subscribed to %s:", param_tmc_cmd_trq_topic_.c_str());
      RCLCPP_INFO_STREAM(p_node_->get_logger(),"  Torque unit: mA");
    }
  }
  RCLCPP_INFO(p_node_->get_logger(),"========================================");
}

void BldcMotor::cmdVelSubscriberCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  RCLCPP_DEBUG_STREAM(p_node_->get_logger(),this->getMotorName() << " [BldcMotor::"
    << __func__ << "]");
  float val = msg->linear.x;
  int32_t rpm_val = 0;
  tmcl_cmd_t vel_cmd = TMCL_CMD_ROR;

  if(0.0 == param_wheel_diameter_)
  {
    rpm_val = val / param_additional_ratio_vel_;
  }
  else
  {
    rpm_val = val * (SEC_TO_MIN / (PI * param_wheel_diameter_)) * (1 / param_additional_ratio_vel_);
  }

  RCLCPP_DEBUG_STREAM(p_node_->get_logger(), this->getMotorName() << "Setting cmd_vel, received: "
    << val << " board_val: " << rpm_val);

  if(val < 0)
  {
    rpm_val = rpm_val * -1; // Change sign to positive
    vel_cmd = TMCL_CMD_ROL; // Rotate Left
  }
  else
  {
    vel_cmd = TMCL_CMD_ROR; // Rotate Right
  }

  if(p_tmcl_interpreter_->executeCmd(vel_cmd, static_cast<uint8_t>(0x00), this->getMotorNumber(),
      &rpm_val))
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

void BldcMotor::cmdRelposSubscriberCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  RCLCPP_DEBUG_STREAM(p_node_->get_logger(),this->getMotorName() << " [BldcMotor::"
    << __func__ << "]");
  float convert_const_deg = 0.00;
  int32_t val = msg->data;
  int32_t unit_val = 0;

  if(position_scaler_m_ > 0)
  {
    convert_const_deg = ((float)position_scaler_m_ / ANGULAR_FULL_ROTATION) *
      (1 / param_additional_ratio_pos_);
  }
  else if(encoder_steps_ > 0)
  {
    convert_const_deg = ((float)encoder_steps_ / ANGULAR_FULL_ROTATION) *
      (1 / param_additional_ratio_pos_);
  }
  else
  {
    convert_const_deg = 1 / param_additional_ratio_pos_;
  }

  unit_val = val * convert_const_deg;

  RCLCPP_DEBUG_STREAM(p_node_->get_logger(), this->getMotorName() << "Setting cmd_relpos, " <<
    "received: "<< val << " board_val: " << unit_val);
  const uint8_t REL = 1;
  if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_MVP, REL, this->getMotorNumber(), &unit_val))
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

void BldcMotor::cmdAbsposSubscriberCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  RCLCPP_DEBUG_STREAM(p_node_->get_logger(),this->getMotorName() << " [BldcMotor::"
    << __func__ << "]");
  float convert_const_deg = 0.00;
  int32_t val = msg->data;
  int32_t unit_val = 0;

  if(position_scaler_m_ > 0)
  {
    convert_const_deg = ((float)position_scaler_m_ / ANGULAR_FULL_ROTATION) *
      (1 / param_additional_ratio_pos_);
  }
  else if(encoder_steps_ > 0)
  {
    convert_const_deg = ((float)encoder_steps_ / ANGULAR_FULL_ROTATION) *
      (1 / param_additional_ratio_pos_);
  }
  else
  {
    convert_const_deg = 1 / param_additional_ratio_pos_;
  }

  unit_val = val * convert_const_deg;
  RCLCPP_DEBUG_STREAM(p_node_->get_logger(), this->getMotorName() << "Setting cmd_abspos, "
    "received: "<< val << " board_val: " << unit_val);

  const uint8_t ABS = 0;
  if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_MVP, ABS, this->getMotorNumber(), &unit_val))
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
