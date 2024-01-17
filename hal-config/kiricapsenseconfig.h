/*
 * kirisakicapsenseconfig.h
 *
 *  Created on: Dec 15, 2023
 *      Author: brandon
 */

#ifndef HAL_CONFIG_KIRICAPSENSECONFIG_H_
#define HAL_CONFIG_KIRICAPSENSECONFIG_H_

#define KCS_ACMP_CAPSENSE                       ACMP0
#define KCS_ACMP_CAPSENSE_CLKEN                 CMU_HFPERCLKEN0_ACMP0
#define PRS_CH_CTRL_SOURCESEL_ACMP_CAPSENSE     PRS_CH_CTRL_SOURCESEL_ACMP0
#define PRS_CH_CTRL_SIGSEL_ACMPOUT_CAPSENSE     PRS_CH_CTRL_SIGSEL_ACMP0OUT

#define KCS_NUM_CHANNELS        2             /**< Number of channels used by Kirisaki Capsense */

#define BUTTON0_CHANNEL         0             /**< Button 0 channel */
#define BUTTON1_CHANNEL         1             /**< Button 1 channel */

#define KCS_BUF0_SZ             3
#define KCS_BUF1_SZ             3             /**< Also called NHD in a certain application note */

#define KCS_MHD                 20
#define KCS_RECAL_THR			10000u
#define KCS_SLR_U				0			  /**< Both slew rates operate at call speed / KCS_BUF0_SZ / KCS_BUF1_SZ */
#define KCS_SLR_D				20

#define  KCS_CHANNEL_IDX_2_HW {BUTTON0_CHANNEL, BUTTON1_CHANNEL}

#endif /* HAL_CONFIG_KIRICAPSENSECONFIG_H_ */
