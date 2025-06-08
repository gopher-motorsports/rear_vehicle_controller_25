/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
     PC9   ------> I2C3_SDA
     PA8   ------> I2C3_SCL
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, BRK_LT_Pin|BUZZER_Pin|MCU_AUX_1_Pin|MCU_AUX_2_Pin
                          |PULLUP_GATE_3_Pin|PULLUP_GATE_4_Pin|PULLUP_GATE_2_Pin|PULLUP_GATE_1_Pin
                          |PULLUP_GATE_6_Pin|PULLUP_GATE_5_Pin|GSENSE_LED_Pin|MCU_STATUS_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TSSI_RED_Pin|TSSI_GREEN_Pin|AUX_GPIO_2_Pin|AUX_GPIO_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PCB_BUZZER_GPIO_Port, PCB_BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HARDFAULT_LED_GPIO_Port, HARDFAULT_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CURR_FAULT_12V_Pin */
  GPIO_InitStruct.Pin = CURR_FAULT_12V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CURR_FAULT_12V_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BRK_LT_Pin BUZZER_Pin MCU_AUX_1_Pin MCU_AUX_2_Pin
                           GSENSE_LED_Pin MCU_STATUS_LED_Pin */
  GPIO_InitStruct.Pin = BRK_LT_Pin|BUZZER_Pin|MCU_AUX_1_Pin|MCU_AUX_2_Pin
                          |GSENSE_LED_Pin|MCU_STATUS_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PULLUP_GATE_3_Pin PULLUP_GATE_4_Pin PULLUP_GATE_2_Pin PULLUP_GATE_1_Pin
                           PULLUP_GATE_6_Pin PULLUP_GATE_5_Pin */
  GPIO_InitStruct.Pin = PULLUP_GATE_3_Pin|PULLUP_GATE_4_Pin|PULLUP_GATE_2_Pin|PULLUP_GATE_1_Pin
                          |PULLUP_GATE_6_Pin|PULLUP_GATE_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : RTD_BUTTON_Pin */
  GPIO_InitStruct.Pin = RTD_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RTD_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TH_SDA_Pin */
  GPIO_InitStruct.Pin = TH_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(TH_SDA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TH_SCL_Pin */
  GPIO_InitStruct.Pin = TH_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(TH_SCL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TSSI_RED_Pin TSSI_GREEN_Pin AUX_GPIO_2_Pin AUX_GPIO_1_Pin */
  GPIO_InitStruct.Pin = TSSI_RED_Pin|TSSI_GREEN_Pin|AUX_GPIO_2_Pin|AUX_GPIO_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SDC_OUT_SENSE_Pin SDC_IN_SENSE_Pin */
  GPIO_InitStruct.Pin = SDC_OUT_SENSE_Pin|SDC_IN_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PCB_BUZZER_Pin */
  GPIO_InitStruct.Pin = PCB_BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PCB_BUZZER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CURR_FAULT_5V_Pin CURR_FAULT_5V_2_Pin BSPD_TS_SNS_FAULT_Pin BSPD_TS_BRK_FAULT_Pin
                           BSPD_BRK_FAULT_Pin */
  GPIO_InitStruct.Pin = CURR_FAULT_5V_Pin|CURR_FAULT_5V_2_Pin|BSPD_TS_SNS_FAULT_Pin|BSPD_TS_BRK_FAULT_Pin
                          |BSPD_BRK_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : HARDFAULT_LED_Pin */
  GPIO_InitStruct.Pin = HARDFAULT_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HARDFAULT_LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
