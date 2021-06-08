/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : DHT11.h
  * @brief          : Header for DHT11.c file.
  *                   Function for DHT11
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DHT11_H
#define __DHT11_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "math.h"

#define DHT11_PORT 	GPIOA
#define DHT11_PIN 	GPIO_PIN_3

extern TIM_HandleTypeDef htim16;

void delayuSecTim16(uint32_t uSec);
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

void startDHT11(void);
uint8_t checkResponse (void);
uint8_t readDHT11(void);
float soundSpeedCalibration(uint8_t tempByte1);


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
