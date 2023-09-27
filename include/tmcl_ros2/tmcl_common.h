/**
 * Copyright (c) 2023 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#ifndef TMCL_COMMON_H
#define TMCL_COMMON_H

/*******************************************************************************/
/*                      Constants and Enumerations                             */
/*******************************************************************************/


/* LUT row indeces of get-able/set-able Communication Interface Parameters */
typedef enum
{
  IDX_COMM_INTERFACE = 0,
  IDX_COMM_INTERFACE_NAME,
  IDX_COMM_TX_ID,
  IDX_COMM_RX_ID,
  IDX_COMM_TIMEOUT_MS,
  IDX_COMM_EXEC_CMD_RETRIES,
  TMCL_INTERFACE_PARAMS_LUT_MAX           /* This should not be used */
}tmcl_interface_params_lut_t;

/* Parameter Names of get-able/set-able Communication Interface Parameters */
static const std::vector<std::string>  comm_interface_params_ = {
  "comm_interface",
  "comm_interface_name",
  "comm_tx_id",
  "comm_rx_id",
  "comm_timeout_ms",
  "comm_exec_cmd_retries",
  "MAX"         /* This should not be used */
};

#endif //TMCL_COMMON_H
