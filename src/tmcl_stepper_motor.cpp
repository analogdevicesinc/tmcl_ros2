/**
 * Copyright (c) 2023 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tmcl_ros2/tmcl_stepper_motor.h"

using std::placeholders::_1;

StepperMotor::StepperMotor(rclcpp::Node::SharedPtr p_node, TmclInterpreter* p_tmcl_interpreter,
  uint8_t motor_number, uint32_t module_number) :
  Motor(p_node,p_tmcl_interpreter,motor_number, module_number)
{
  RCLCPP_INFO_STREAM(p_node_->get_logger(),this->getMotorName() << " [StepperMotor::"
    << __func__ << "]");
  mode_ = STEPPER_MODE_OPENLOOP;
  mode_=this->getStepperMode();
}

StepperMotor::~StepperMotor()
{
  RCLCPP_INFO_STREAM(p_node_->get_logger(),this->getMotorName() <<" [StepperMotor::"
    << __func__ << "]");
}

void StepperMotor::init()
{
  RCLCPP_INFO_STREAM(p_node_->get_logger(),this->getMotorName() << " [StepperMotor::"
    << __func__ << "]");
  int32_t val = 0;
  Motor::initMotorParams();
  Motor::initPublisherParams();
  this->initPublisher();

  if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "fullstep resolution", this->getMotorNumber(),
      &val))
  {
    motor_full_step_resolution_ = val;
    RCLCPP_DEBUG(p_node_->get_logger(), "fullstep resolution 0x%02x",motor_full_step_resolution_);
  }
  else if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "motor full step resolution",
            this->getMotorNumber(), &val))
  {
    motor_full_step_resolution_ = val;
    RCLCPP_DEBUG(p_node_->get_logger(), "motor full step resolution 0x%02x",
      motor_full_step_resolution_);
  }
  else if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "MotorFullStepResolution",
            this->getMotorNumber(), &val))
  {
    motor_full_step_resolution_ = val;
    RCLCPP_DEBUG(p_node_->get_logger(), "MotorFullStepResolution 0x%02x",
      motor_full_step_resolution_);
  }
  else
  {
    motor_full_step_resolution_ = 0;
    RCLCPP_WARN_STREAM(p_node_->get_logger(),"Fail to get fullstep resolution/motor" \
      " full step resolution; Setting to 0.");
  }

  if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "MicrostepResolution", this->getMotorNumber(),
      &val))
  {
    RCLCPP_DEBUG(p_node_->get_logger(), "MicrostepResolution 0x%02x",val);
    if(2300 == this->module_number_)
    {
      // Received value is actual resolution
      microstep_resolution_ = val;
    }
    else // Default handling
    {
      // Received value if bit shift
      microstep_resolution_ = 1 << val;
    }
    RCLCPP_DEBUG(p_node_->get_logger(), "MicrostepResolution: %d",microstep_resolution_);
  }
  else
  {
    microstep_resolution_ = 0;
    RCLCPP_WARN_STREAM(p_node_->get_logger(),"Fail to get MicrostepResolution; Setting to 0.");
  }

  this->initSubscriberParams();
  this->initSubscribers();

  RCLCPP_INFO_STREAM(p_node_->get_logger(),this->getMotorName() << "[StepperMotor::" << __func__ <<
    "] Initialized");
  return;
}

void StepperMotor::initPublisher()
{
  RCLCPP_INFO_STREAM(p_node_->get_logger(),this->getMotorName() << " [StepperMotor::"
    << __func__ << "]");
  int period_ms = 0;

  if(param_en_pub_tmc_info_)
  {
    publisher_ = p_node_->create_publisher<adi_tmcl::msg::TmcInfo>(param_tmc_info_topic_, 10);
    period_ms = (1000/param_pub_rate_tmc_info_);
    RCLCPP_DEBUG_STREAM(p_node_->get_logger(),"rate= " << std::to_string(param_pub_rate_tmc_info_)
      << "; period_ms= " << period_ms);
    publisher_timer_ = p_node_->create_wall_timer(std::chrono::milliseconds(period_ms),
      std::bind(&StepperMotor::pubTimerCallback, this));
  }
  else
  {
    RCLCPP_WARN_STREAM(p_node_->get_logger(), p_node_->get_name() << "/tmc_info_" <<
      this->getMotorNumber() << " not published.");
  }

  return;
}

void StepperMotor::pubTimerCallback()
{
  RCLCPP_DEBUG_STREAM(p_node_->get_logger(),this->getMotorName() << " [StepperMotor::"
    << __func__ << "]");
  auto message = adi_tmcl::msg::TmcInfo();
  int32_t val = 0;

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

  // No Board voltage
  message.board_voltage = NAN;

  if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "DrvStatusFlags", this->getMotorNumber(), &val))
  {
    RCLCPP_DEBUG(p_node_->get_logger(), "DrvStatusFlags: 0x%02x", val);
    message.status.append("[DrvStatusFlags: " + std::to_string(val) + "] ");
  }
  else if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "driver error flags",
            this->getMotorNumber(), &val))
  {
    RCLCPP_DEBUG(p_node_->get_logger(), "driver error flags: 0x%02x", val);
    message.status.append("[driver error flags: " + std::to_string(val) + "] ");
  }
  else
  {
    RCLCPP_ERROR_STREAM_ONCE(p_node_->get_logger(),"Fail to get DrvStatusFlags/driver error flags");
  }

  if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "extended error flags", this->getMotorNumber(),
      &val))
  {
    RCLCPP_DEBUG(p_node_->get_logger(), "extended error flags: 0x%02x", val);
    message.status.append("[extended error flags: " + std::to_string(val) + "] ");
  }
  else
  {
    RCLCPP_ERROR_STREAM_ONCE(p_node_->get_logger(),"Fail to get extended error flags");
  }

  if(param_pub_actual_vel_)
  {
    if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "ActualVelocity", this->getMotorNumber(),
        &val))
    {
      RCLCPP_DEBUG(p_node_->get_logger(), "ActualVelocity 0x%02x",val);
      if(0.0 == param_wheel_diameter_ || 0 == microstep_resolution_ ||
          0 == motor_full_step_resolution_)
      {
        message.velocity = val * param_additional_ratio_vel_;
      }
      else
      {
        message.velocity = val * (((PI * param_wheel_diameter_) /
          (microstep_resolution_ * (float)motor_full_step_resolution_)) *
          param_additional_ratio_vel_);
      }
    }
    else
    {
      RCLCPP_ERROR_STREAM_ONCE(p_node_->get_logger(),"Fail to get ActualVelocity");
    }
  }

  if(param_pub_actual_pos_)
  {
    if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "ActualPosition", this->getMotorNumber(),
        &val))
    {
      RCLCPP_DEBUG(p_node_->get_logger(), "ActualPosition 0x%02x",val);
      if(0 == microstep_resolution_ || 0 == motor_full_step_resolution_)
      {
        message.position = val * param_additional_ratio_pos_;
      }
      else
      {
        message.position = val * ((ANGULAR_FULL_ROTATION /
          (microstep_resolution_ * (float)motor_full_step_resolution_)) *
          param_additional_ratio_pos_);
      }
    }
    else
    {
      RCLCPP_ERROR_STREAM_ONCE(p_node_->get_logger(),"Fail to get ActualPosition");
    }
  }

  if(param_pub_actual_trq_)
  {
    if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "Torque", this->getMotorNumber(), &val))
    {
      RCLCPP_DEBUG(p_node_->get_logger(), "Torque 0x%02x",val);
      message.torque = val * param_additional_ratio_trq_;
    }
    else
    {
      RCLCPP_ERROR_STREAM_ONCE(p_node_->get_logger(),"Fail to get Torque");
    }
  }

  publisher_->publish(message);

}

tmcl_stepper_mode_t StepperMotor::getStepperMode()
{
  RCLCPP_DEBUG_STREAM(p_node_->get_logger(),this->getMotorName() << " [StepperMotor::"
    << __func__ << "]");
  tmcl_stepper_mode_t mode = STEPPER_MODE_OPENLOOP;
  int32_t val;

  if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "closed loop", this->getMotorNumber(), &val))
  {
    mode = static_cast<tmcl_stepper_mode_t>(val);
    RCLCPP_DEBUG_STREAM(p_node_->get_logger(), "closed loop: " << mode);
  }
  else
  {
    RCLCPP_WARN_STREAM(p_node_->get_logger(),"Fail to get closed loop");
  }

  return mode;
}

void StepperMotor::initSubscriberParams()
{
  RCLCPP_INFO_STREAM(p_node_->get_logger(),this->getMotorName() << " [StepperMotor::"
     << __func__ << "]");
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  rcl_interfaces::msg::IntegerRange param_int_range;
  std::string param_default_str = "";

  if(mode_ >= STEPPER_MODE_OPENLOOP)
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

    if(mode_ == STEPPER_MODE_CLOSEDLOOP)
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

void StepperMotor::initSubscribers()
{
  RCLCPP_INFO_STREAM(p_node_->get_logger(),this->getMotorName() << " [StepperMotor::"
    << __func__ << "]");
  int32_t val = 0;

  if(mode_ >= STEPPER_MODE_OPENLOOP)
  {
    subscription_cmd_vel_=p_node_->create_subscription<geometry_msgs::msg::Twist>(
      param_tmc_cmd_vel_topic_, 10, std::bind(&StepperMotor::cmdVelSubscriberCallback, this, _1));
    /* Print unit */
    RCLCPP_INFO(p_node_->get_logger(),"========================================");
    RCLCPP_INFO(p_node_->get_logger(),"Subscribed to %s:", param_tmc_cmd_vel_topic_.c_str());
    if(0.0 == param_wheel_diameter_ || 0 == microstep_resolution_ ||
        0 == motor_full_step_resolution_)
    {
      RCLCPP_INFO_STREAM(p_node_->get_logger(),"  Velocity unit: pps");
    }
    else
    {
      RCLCPP_INFO_STREAM(p_node_->get_logger(),"  Velocity unit: m/s");
    }

    subscription_cmd_abspos_=p_node_->create_subscription<std_msgs::msg::Int32>(
      param_tmc_cmd_abspos_topic_, 10, std::bind(&StepperMotor::cmdAbsposSubscriberCallback,
      this, _1));

    subscription_cmd_relpos_=p_node_->create_subscription<std_msgs::msg::Int32>(
      param_tmc_cmd_relpos_topic_, 10, std::bind(&StepperMotor::cmdRelposSubscriberCallback,
      this, _1));

    /* Print unit */
    RCLCPP_INFO(p_node_->get_logger(),"========================================");
    RCLCPP_INFO(p_node_->get_logger(),"Subscribed to %s and %s:",
      param_tmc_cmd_abspos_topic_.c_str(), param_tmc_cmd_relpos_topic_.c_str());
    if(0 == microstep_resolution_ || 0 == motor_full_step_resolution_)
    {
      RCLCPP_INFO_STREAM(p_node_->get_logger(),"  Position unit: pulses");
    }
    else
    {
      RCLCPP_INFO_STREAM(p_node_->get_logger(),"  Position unit: angular degrees");
    }

    if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "relative positioning option",
    this->getMotorNumber(), &val))
    {
      RCLCPP_DEBUG(p_node_->get_logger(), "relative positioning option 0x%02x",val);
    }
    RCLCPP_INFO_STREAM(p_node_->get_logger(),"NOTE: " << param_tmc_cmd_relpos_topic_ <<
      " depends on the Relative Positioning Option Axis Parameter.");
    RCLCPP_INFO_STREAM(p_node_->get_logger(),"      Currently set as " << val <<
      ". Refer to datasheet on how to configure.");

    if(mode_ == STEPPER_MODE_CLOSEDLOOP)
    {
      subscription_cmd_trq_=p_node_->create_subscription<std_msgs::msg::Int32>(
        param_tmc_cmd_trq_topic_, 10, std::bind(&StepperMotor::cmdTrqSubscriberCallback, this, _1));
       /* Print unit */
      RCLCPP_INFO(p_node_->get_logger(),"========================================");
      RCLCPP_INFO(p_node_->get_logger(),"Subscribed to %s:", param_tmc_cmd_trq_topic_.c_str());
      RCLCPP_INFO_STREAM(p_node_->get_logger(),"  Torque unit: mA");
    }
  }
  RCLCPP_INFO(p_node_->get_logger(),"========================================");
}

void StepperMotor::cmdVelSubscriberCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  RCLCPP_DEBUG_STREAM(p_node_->get_logger(),this->getMotorName() << " [StepperMotor::"
    << __func__ << "]");
  float val = msg->linear.x;
  int32_t pps_val = 0;
  tmcl_cmd_t vel_cmd = TMCL_CMD_ROR;

  if(0.0 == param_wheel_diameter_ || 0 == microstep_resolution_ || 0 == motor_full_step_resolution_)
  {
    pps_val = val / param_additional_ratio_vel_;
  }
  else
  {
    pps_val = val * (((microstep_resolution_ * (float)motor_full_step_resolution_) /
      (PI * param_wheel_diameter_)) * (1 / param_additional_ratio_vel_));
  }

  RCLCPP_DEBUG_STREAM(p_node_->get_logger(), this->getMotorName() << "Setting cmd_vel, received: "
    << val << " board_val: " << pps_val);

  if(val < 0)
  {
    pps_val = pps_val * -1; // Change sign to positive
    vel_cmd = TMCL_CMD_ROL; // Rotate Left
  }
  else
  {
    vel_cmd = TMCL_CMD_ROR; // Rotate Right
  }

  if(p_tmcl_interpreter_->executeCmd(vel_cmd, static_cast<uint8_t>(0x00), this->getMotorNumber(),
      &pps_val))
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

void StepperMotor::cmdRelposSubscriberCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  RCLCPP_DEBUG_STREAM(p_node_->get_logger(),this->getMotorName() << " [StepperMotor::"
    << __func__ << "]");
  float convert_const_deg = 0.00;
  int32_t val = msg->data;
  int32_t unit_val = 0;

  if(0 < microstep_resolution_ && 0 < motor_full_step_resolution_)
  {
    convert_const_deg = ((microstep_resolution_ * (float)motor_full_step_resolution_) /
      ANGULAR_FULL_ROTATION) * (1 / param_additional_ratio_pos_);
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

void StepperMotor::cmdAbsposSubscriberCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  RCLCPP_DEBUG_STREAM(p_node_->get_logger(),this->getMotorName() << " [StepperMotor::"
    << __func__ << "]");
  float convert_const_deg = 0.00;
  int32_t val = msg->data;
  int32_t unit_val = 0;

  if(0 < microstep_resolution_ && 0 < motor_full_step_resolution_)
  {
    convert_const_deg = ((microstep_resolution_ * (float)motor_full_step_resolution_) /
      ANGULAR_FULL_ROTATION) * (1 / param_additional_ratio_pos_);
  }
  else
  {
    convert_const_deg = 1 / param_additional_ratio_pos_;
  }

  unit_val = val * convert_const_deg;
  RCLCPP_DEBUG_STREAM(p_node_->get_logger(), this->getMotorName() << "Setting cmd_abspos, " <<
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

void StepperMotor::cmdTrqSubscriberCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  RCLCPP_DEBUG_STREAM(p_node_->get_logger(),this->getMotorName() << " [StepperMotor::"
    << __func__ << "]");
  int32_t val = msg->data;

  val = val / param_additional_ratio_trq_;
  RCLCPP_DEBUG_STREAM(p_node_->get_logger(), this->getMotorName() << "Setting cmd_trq, " <<
    "received: "<< msg->data << " board_val: " << val);

  if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_SAP, "torque", this->getMotorNumber(), &val))
  {
    RCLCPP_DEBUG_STREAM(p_node_->get_logger(),"\nSubscriber callback " << __func__  <<
      " exited successfully");
  }
  else
  {
    RCLCPP_ERROR_STREAM(p_node_->get_logger(),"Fail to set torque");
  }
  return;
}

