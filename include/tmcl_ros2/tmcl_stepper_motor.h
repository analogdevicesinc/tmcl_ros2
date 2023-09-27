/**
 * Copyright (c) 2023 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#ifndef TMCL_STEPPER_MOTOR_H
#define TMCL_STEPPER_MOTOR_H

#include "tmcl_ros2/tmcl_motor.h"

/*******************************************************************************/
/*                      Constants and Enumerations                             */
/*******************************************************************************/

/* Possible values for <Stepper Mode> */
typedef enum
{
  STEPPER_MODE_OPENLOOP = 0,
  STEPPER_MODE_CLOSEDLOOP
} tmcl_stepper_mode_t;

class StepperMotor : public Motor
{
public:
  StepperMotor(rclcpp::Node::SharedPtr p_node, TmclInterpreter* p_tmcl_interpreter,
    uint8_t motor_number, uint32_t module_number);
  virtual ~StepperMotor();

private:
  tmcl_stepper_mode_t mode_;
  uint32_t motor_full_step_resolution_;
  uint16_t microstep_resolution_;

  void init() override;
  void initPublisher() override;
  void pubTimerCallback() override;
  void initSubscriberParams() override;
  void initSubscribers() override;
  void cmdVelSubscriberCallback(const geometry_msgs::msg::Twist::SharedPtr msg) override;
  void cmdAbsposSubscriberCallback(const std_msgs::msg::Int32::SharedPtr msg) override;
  void cmdRelposSubscriberCallback(const std_msgs::msg::Int32::SharedPtr msg) override;
  void cmdTrqSubscriberCallback(const std_msgs::msg::Int32::SharedPtr msg) override;

  tmcl_stepper_mode_t getStepperMode();
};


#endif //TMCL_STEPPER_MOTOR_H
