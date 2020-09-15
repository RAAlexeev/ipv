/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal_dma.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
//typedef enum {false, true} bool;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void _Error_Handler(const char *f,const uint16_t line);

/* USER CODE BEGIN EFP */
extern uint8_t mb_buf_in[256];
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SENS2_FAIL_Pin GPIO_PIN_1
#define SENS2_FAIL_GPIO_Port GPIOA
#define SENS1_FAIL_Pin GPIO_PIN_3
#define SENS1_FAIL_GPIO_Port GPIOA
#define RELAY_1_Pin GPIO_PIN_4
#define RELAY_1_GPIO_Port GPIOC
#define RELAY_2_Pin GPIO_PIN_5
#define RELAY_2_GPIO_Port GPIOC
#define SS_Pin GPIO_PIN_0
#define SS_GPIO_Port GPIOB
#define SS_2_Pin GPIO_PIN_1
#define SS_2_GPIO_Port GPIOB
#define SS_3_Pin GPIO_PIN_2
#define SS_3_GPIO_Port GPIOB
#define HG_1_Pin GPIO_PIN_8
#define HG_1_GPIO_Port GPIOD
#define HG_2_Pin GPIO_PIN_9
#define HG_2_GPIO_Port GPIOD
#define BUT2_Pin GPIO_PIN_15
#define BUT2_GPIO_Port GPIOA
#define BUT2_EXTI_IRQn EXTI15_10_IRQn
#define BUT1_Pin GPIO_PIN_10
#define BUT1_GPIO_Port GPIOC
#define BUT1_EXTI_IRQn EXTI15_10_IRQn
#define A_Pin GPIO_PIN_0
#define A_GPIO_Port GPIOD
#define B_Pin GPIO_PIN_1
#define B_GPIO_Port GPIOD
#define C_Pin GPIO_PIN_2
#define C_GPIO_Port GPIOD
#define D_Pin GPIO_PIN_3
#define D_GPIO_Port GPIOD
#define E_Pin GPIO_PIN_4
#define E_GPIO_Port GPIOD
#define F_Pin GPIO_PIN_5
#define F_GPIO_Port GPIOD
#define G_Pin GPIO_PIN_6
#define G_GPIO_Port GPIOD
#define DP_Pin GPIO_PIN_7
#define DP_GPIO_Port GPIOD
#define U1_DE_Pin GPIO_PIN_5
#define U1_DE_GPIO_Port GPIOB
#define WC_Pin GPIO_PIN_0
#define WC_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */
#define Error_Handler()_Error_Handler(__FILE__,__LINE__);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
