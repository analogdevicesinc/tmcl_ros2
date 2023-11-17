/**
 * Copyright (c) 2023 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#ifndef TMCL_MOTOR_H
#define TMCL_MOTOR_H

#include "tmcl_ros2/tmcl_interpreter.h"
#include "tmcl_ros2/tmcl_common.h"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "adi_tmcl/msg/tmc_info.hpp"

class Motor
{
public:
  Motor(rclcpp::Node::SharedPtr p_node, TmclInterpreter* p_tmcl_interpreter,
    uint8_t motor_number, uint32_t module_number);
  virtual ~Motor();

  uint8_t getMotorNumber();
  std::string getMotorName();
  virtual void init();

protected:
  rclcpp::Node::SharedPtr p_node_;
  TmclInterpreter* p_tmcl_interpreter_;
  rclcpp::Publisher<adi_tmcl::msg::TmcInfo>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr publisher_timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_cmd_vel_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_cmd_abspos_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_cmd_relpos_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_cmd_trq_;
  bool param_en_pub_tmc_info_;
  std::string param_tmc_info_topic_;
  uint8_t param_pub_rate_tmc_info_;
  bool param_pub_actual_vel_;
  bool param_pub_actual_pos_;
  bool param_pub_actual_trq_;
  std::string param_tmc_cmd_vel_topic_;
  std::string param_tmc_cmd_abspos_topic_;
  std::string param_tmc_cmd_relpos_topic_;
  std::string param_tmc_cmd_trq_topic_;
  double param_wheel_diameter_;
  double param_additional_ratio_vel_;
  double param_additional_ratio_pos_;
  double param_additional_ratio_trq_;
  std::string param_comm_interface_name_;
  uint32_t module_number_;
  std::string tmc_info_frame_id_;

  void initMotorParams();
  void initPublisherParams();
  virtual void cmdTrqSubscriberCallback(const std_msgs::msg::Int32::SharedPtr msg);
private:
  uint8_t motor_number_;

  virtual void initPublisher();
  virtual void pubTimerCallback();
  virtual void initSubscriberParams();
  virtual void initSubscribers();
  virtual void cmdVelSubscriberCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  virtual void cmdAbsposSubscriberCallback(const std_msgs::msg::Int32::SharedPtr msg);
  virtual void cmdRelposSubscriberCallback(const std_msgs::msg::Int32::SharedPtr msg);

};

/*******************************************************************************/
/*                      Constants and Enumerations                             */
/*******************************************************************************/

/* Conversion constants */

/* Derived from converting linear velocity (ROS velocity unit) to rpm (TMC board velocity unit)*/
const uint8_t SEC_TO_MIN = 60;
const float PI = 3.1415926535;

/* Used for converting degrees (general position/angular unit) to steps (TMC board position/angular unit)*/
const uint16_t ANGULAR_FULL_ROTATION = 360;

/* Parameter limits and default values */
const uint8_t PUB_RATE_MIN = 1;
const uint8_t PUB_RATE_MAX = 100;
const uint8_t PUB_RATE_DEFAULT = 10;

/* LUT row indeces of get-able/set-able ROS Topic-related Parameters */
typedef enum
{
  IDX_EN_PUB_TMC_INFO = 0,
  IDX_TMC_INFO_TOPIC,
  IDX_PUB_RATE_TMC_INFO,
  IDX_PUB_ACTUAL_VEL,
  IDX_PUB_ACTUAL_TRQ,
  IDX_PUB_ACTUAL_POS,
  IDX_TMC_CMD_VEL_TOPIC,
  IDX_TMC_CMD_ABSPOS_TOPIC,
  IDX_TMC_CMD_RELPOS_TOPIC,
  IDX_TMC_CMD_TRQ_TOPIC,
  TMCL_ROS_TOPIC_PARAMS_LUT_MAX          /* This should not be used */
}tmcl_ros_topic_params_lut_t;

/* Parameter Names of get-able/set-able General Parameters */
static const std::vector<std::string> ros_topic_params_ = {
  "en_pub_tmc_info",
  "tmc_info_topic",
  "pub_rate_tmc_info",
  "pub_actual_vel",
  "pub_actual_trq",
  "pub_actual_pos",
  "tmc_cmd_vel_topic",
  "tmc_cmd_abspos_topic",
  "tmc_cmd_relpos_topic",
  "tmc_cmd_trq_topic",
  "MAX"         /* This should not be used */
};

/* LUT row indeces of get-able/set-able Hardware Parameters */
typedef enum
{
  IDX_WHEEL_DIAMETER = 0,
  TMCL_HARDWARE_PARAMS_LUT_MAX          /* This should not be used */
}tmcl_hardware_params_lut_t;

/* Parameter Names of get-able/set-able Hardware Parameters */
static const std::vector<std::string> hardware_params_ = {
  "wheel_diameter",
  "MAX"         /* This should not be used */
};

/* LUT row indeces of get-able/set-able Additional Ratio Params */
typedef enum
{
  IDX_ADDITIONAL_RATIO_VEL = 0,
  IDX_ADDITIONAL_RATIO_POS,
  IDX_ADDITIONAL_RATIO_TRQ,
  TMCL_ADDITIONAL_RATIO_PARAMS_LUT_MAX          /* This should not be used */
}tmcl_additional_ratio_params_lut_t;

/* Parameter Names of get-able/set-able Additional Ratio Params */
static const std::vector<std::string> additional_ratio_params_ = {
  "additional_ratio_vel",
  "additional_ratio_pos",
  "additional_ratio_trq",
  "MAX"         /* This should not be used */
};

#endif //TMCL_MOTOR_H
