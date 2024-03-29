/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "stm32f7xx_hal.h"
#include <stdint.h>
void __io_putchar(uint8_t ch);
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define MOT_L_STB_Pin GPIO_PIN_13
#define MOT_L_STB_GPIO_Port GPIOC
#define CS_ENC_L_Pin GPIO_PIN_1
#define CS_ENC_L_GPIO_Port GPIOH
#define MOT_L_PH_1_Pin GPIO_PIN_0
#define MOT_L_PH_1_GPIO_Port GPIOC
#define WIDESEN_R_Pin GPIO_PIN_1
#define WIDESEN_R_GPIO_Port GPIOC
#define MOT_L_PH_2_Pin GPIO_PIN_2
#define MOT_L_PH_2_GPIO_Port GPIOC
#define WIDESEN_L_Pin GPIO_PIN_1
#define WIDESEN_L_GPIO_Port GPIOA
#define MOT_L_PWM_Pin GPIO_PIN_0
#define MOT_L_PWM_GPIO_Port GPIOB
#define BATT_ADC_Pin GPIO_PIN_1
#define BATT_ADC_GPIO_Port GPIOB
#define FAN_MOT_Pin GPIO_PIN_10
#define FAN_MOT_GPIO_Port GPIOB
#define SPK_Pin GPIO_PIN_11
#define SPK_GPIO_Port GPIOB
#define IRLED_RFLS_Pin GPIO_PIN_12
#define IRLED_RFLS_GPIO_Port GPIOB
#define IRLED_RSLF_Pin GPIO_PIN_14
#define IRLED_RSLF_GPIO_Port GPIOB
#define LED_L1_Pin GPIO_PIN_15
#define LED_L1_GPIO_Port GPIOB
#define CS_ENC_R_Pin GPIO_PIN_6
#define CS_ENC_R_GPIO_Port GPIOC
#define MOT_R_PWM_Pin GPIO_PIN_7
#define MOT_R_PWM_GPIO_Port GPIOC
#define LED_L2_Pin GPIO_PIN_8
#define LED_L2_GPIO_Port GPIOC
#define LED_L3_Pin GPIO_PIN_9
#define LED_L3_GPIO_Port GPIOC
#define LED_L4_Pin GPIO_PIN_8
#define LED_L4_GPIO_Port GPIOA
#define GYRO_INT1_Pin GPIO_PIN_11
#define GYRO_INT1_GPIO_Port GPIOA
#define GYRO_INT2_Pin GPIO_PIN_12
#define GYRO_INT2_GPIO_Port GPIOA
#define CS_GYRO_Pin GPIO_PIN_13
#define CS_GYRO_GPIO_Port GPIOA
#define LED_R4_Pin GPIO_PIN_14
#define LED_R4_GPIO_Port GPIOA
#define LED_R3_Pin GPIO_PIN_15
#define LED_R3_GPIO_Port GPIOA
#define LED_R2_Pin GPIO_PIN_10
#define LED_R2_GPIO_Port GPIOC
#define LED_R1_Pin GPIO_PIN_11
#define LED_R1_GPIO_Port GPIOC
#define MOT_R_PH_1_Pin GPIO_PIN_6
#define MOT_R_PH_1_GPIO_Port GPIOB
#define MOT_R_PH_2_Pin GPIO_PIN_7
#define MOT_R_PH_2_GPIO_Port GPIOB
#define MOT_R_STB_Pin GPIO_PIN_8
#define MOT_R_STB_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
