/**
 * Copyright (c) 2023 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#ifndef TMCL_BLDC_MOTOR_H
#define TMCL_BLDC_MOTOR_H

#include "tmcl_ros2/tmcl_motor.h"

/*******************************************************************************/
/*                      Constants and Enumerations                             */
/*******************************************************************************/

/* Possible values for <Bldc Commutation Mode> */
typedef enum
{
  BLDC_MODE_DISABLED = 0,
  BLDC_MODE_OPENLOOP,
  BLDC_MODE_CLOSEDLOOP
} tmcl_bldc_comm_mode_t;


class BldcMotor : public Motor
{
public:
  BldcMotor(rclcpp::Node::SharedPtr p_node, TmclInterpreter* p_tmcl_interpreter,
    uint8_t motor_number, uint32_t module_number);
  virtual ~BldcMotor();

  void init() override;

private:
  tmcl_bldc_comm_mode_t comm_mode_;
  int32_t position_scaler_m_;
  int32_t encoder_steps_;
  std::vector<std::string> param_statusflags_regname_;
  std::vector<int64_t> param_statusflags_regshift_;

  void initPublisher() override;
  void pubTimerCallback() override;
  void initSubscriberParams() override;
  void initSubscribers() override;
  void cmdVelSubscriberCallback(const geometry_msgs::msg::Twist::SharedPtr msg) override;
  void cmdAbsposSubscriberCallback(const std_msgs::msg::Int32::SharedPtr msg) override;
  void cmdRelposSubscriberCallback(const std_msgs::msg::Int32::SharedPtr msg) override;
  tmcl_bldc_comm_mode_t getCommutationMode();
};

#endif //TMCL_BLDC_MOTOR_H
