/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f3xx_hal.h"

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
#define SWITCH1_Pin GPIO_PIN_13
#define SWITCH1_GPIO_Port GPIOC
#define SWITCH2_Pin GPIO_PIN_14
#define SWITCH2_GPIO_Port GPIOC
#define CAN_STB_Pin GPIO_PIN_15
#define CAN_STB_GPIO_Port GPIOC
#define MOTOR3_AN1_Pin GPIO_PIN_0
#define MOTOR3_AN1_GPIO_Port GPIOC
#define MOTOR4_AN1_Pin GPIO_PIN_1
#define MOTOR4_AN1_GPIO_Port GPIOC
#define MOTOR1_AN1_Pin GPIO_PIN_2
#define MOTOR1_AN1_GPIO_Port GPIOC
#define MOTOR2_AN1_Pin GPIO_PIN_3
#define MOTOR2_AN1_GPIO_Port GPIOC
#define MOTOR3_Pin GPIO_PIN_0
#define MOTOR3_GPIO_Port GPIOA
#define MOTOR4_Pin GPIO_PIN_1
#define MOTOR4_GPIO_Port GPIOA
#define MOTOR1_Pin GPIO_PIN_2
#define MOTOR1_GPIO_Port GPIOA
#define MOTOR2_Pin GPIO_PIN_3
#define MOTOR2_GPIO_Port GPIOA
#define LEVEL_Pin GPIO_PIN_4
#define LEVEL_GPIO_Port GPIOF
#define PS2_DI_Pin GPIO_PIN_4
#define PS2_DI_GPIO_Port GPIOA
#define PS2_DO_Pin GPIO_PIN_5
#define PS2_DO_GPIO_Port GPIOA
#define PS2_CS_Pin GPIO_PIN_6
#define PS2_CS_GPIO_Port GPIOA
#define PS2_CLK_Pin GPIO_PIN_7
#define PS2_CLK_GPIO_Port GPIOA
#define CURRENT_Pin GPIO_PIN_0
#define CURRENT_GPIO_Port GPIOB
#define VOLTAGE_Pin GPIO_PIN_1
#define VOLTAGE_GPIO_Port GPIOB
#define KEY_1_Pin GPIO_PIN_2
#define KEY_1_GPIO_Port GPIOB
#define SONAR_ECHO_Pin GPIO_PIN_10
#define SONAR_ECHO_GPIO_Port GPIOB
#define SONAR_ECHO_EXTI_IRQn EXTI15_10_IRQn
#define SR04_TRIG_1_Pin GPIO_PIN_11
#define SR04_TRIG_1_GPIO_Port GPIOB
#define SERVO_SIG1_Pin GPIO_PIN_12
#define SERVO_SIG1_GPIO_Port GPIOB
#define SERVO_SIG2_Pin GPIO_PIN_13
#define SERVO_SIG2_GPIO_Port GPIOB
#define SERVO_SIG3_Pin GPIO_PIN_14
#define SERVO_SIG3_GPIO_Port GPIOB
#define SERVO_SIG4_Pin GPIO_PIN_15
#define SERVO_SIG4_GPIO_Port GPIOB
#define BEEP_Pin GPIO_PIN_8
#define BEEP_GPIO_Port GPIOC
#define FAN_SIG_Pin GPIO_PIN_9
#define FAN_SIG_GPIO_Port GPIOC
#define WS2812_Pin GPIO_PIN_10
#define WS2812_GPIO_Port GPIOA
#define SONAR_TRIG2_Pin GPIO_PIN_10
#define SONAR_TRIG2_GPIO_Port GPIOC
#define SONAR_TRIG1_Pin GPIO_PIN_11
#define SONAR_TRIG1_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
