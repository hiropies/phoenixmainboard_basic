/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SW1_Pin GPIO_PIN_14
#define SW1_GPIO_Port GPIOC
#define SW1_EXTI_IRQn EXTI15_10_IRQn
#define SW2_Pin GPIO_PIN_15
#define SW2_GPIO_Port GPIOC
#define SW2_EXTI_IRQn EXTI15_10_IRQn
#define buzzer_Pin GPIO_PIN_4
#define buzzer_GPIO_Port GPIOA
#define LINK_TX_Pin GPIO_PIN_4
#define LINK_TX_GPIO_Port GPIOC
#define SPI1_CS_ACCEL_Pin GPIO_PIN_1
#define SPI1_CS_ACCEL_GPIO_Port GPIOB
#define SPI1_CS_GYRO_Pin GPIO_PIN_2
#define SPI1_CS_GYRO_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_14
#define LED3_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_15
#define LED4_GPIO_Port GPIOB
#define CAN_RX_CH3_Pin GPIO_PIN_8
#define CAN_RX_CH3_GPIO_Port GPIOA
#define LINK_RX_Pin GPIO_PIN_10
#define LINK_RX_GPIO_Port GPIOA
#define CAN_TX_CH3_Pin GPIO_PIN_15
#define CAN_TX_CH3_GPIO_Port GPIOA
#define CAN_RX_CH2_Pin GPIO_PIN_5
#define CAN_RX_CH2_GPIO_Port GPIOB
#define CAN_TX_CH2_Pin GPIO_PIN_6
#define CAN_TX_CH2_GPIO_Port GPIOB
#define CAN_RX_CH1_Pin GPIO_PIN_8
#define CAN_RX_CH1_GPIO_Port GPIOB
#define CAN_TX_CH1_Pin GPIO_PIN_9
#define CAN_TX_CH1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
