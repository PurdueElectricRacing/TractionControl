/* USER CODE BEGIN Header */
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
  * COPYRIGHT(c) 2020 STMicroelectronics
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

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
#define MCUCLK_Pin GPIO_PIN_0
#define MCUCLK_GPIO_Port GPIOA
#define ADCshock_Pin GPIO_PIN_2
#define ADCshock_GPIO_Port GPIOA
#define countbit1_Pin GPIO_PIN_3
#define countbit1_GPIO_Port GPIOA
#define countbit2_Pin GPIO_PIN_4
#define countbit2_GPIO_Port GPIOA
#define countbit3_Pin GPIO_PIN_5
#define countbit3_GPIO_Port GPIOA
#define countbit4_Pin GPIO_PIN_6
#define countbit4_GPIO_Port GPIOA
#define countbit5_Pin GPIO_PIN_7
#define countbit5_GPIO_Port GPIOA
#define countclr_Pin GPIO_PIN_0
#define countclr_GPIO_Port GPIOB
#define ADCstrain_Pin GPIO_PIN_1
#define ADCstrain_GPIO_Port GPIOB
#define countbit6_Pin GPIO_PIN_8
#define countbit6_GPIO_Port GPIOA
#define countbit7_Pin GPIO_PIN_9
#define countbit7_GPIO_Port GPIOA
#define countbit8_Pin GPIO_PIN_10
#define countbit8_GPIO_Port GPIOA
#define errorflag_Pin GPIO_PIN_3
#define errorflag_GPIO_Port GPIOB
#define OElowspeed_Pin GPIO_PIN_4
#define OElowspeed_GPIO_Port GPIOB
#define OEmedspeed_Pin GPIO_PIN_5
#define OEmedspeed_GPIO_Port GPIOB
#define OEhighspeed_Pin GPIO_PIN_6
#define OEhighspeed_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
