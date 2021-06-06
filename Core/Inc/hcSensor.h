/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : hcSensor.h
  * @brief          : Header for hcSensor.c file.
  *                   Function for hcSensor
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HCSENSOR_H
#define __HCSENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim7;

uint64_t distanceCenter;
uint64_t distanceLeft;
uint64_t distanceRight;
uint64_t distance;
uint64_t valueRisingEdge;
uint64_t valueFallingEdge;
uint64_t measureTime;
uint8_t waitOnFallingEdge;

uint64_t cntTics;

volatile uint8_t ready;

#define PIN_CENTER			GPIO_PIN_3
#define PIN_CENTER_ECHO		GPIO_PIN_2
#define PORT_CENTER			GPIOF

#define PIN_LEFT			GPIO_PIN_5
#define PIN_LEFT_ECHO		GPIO_PIN_8
#define PORT_LEFT			GPIOF

#define PIN_RIGHT			GPIO_PIN_7
#define PIN_RIGHT_ECHO		GPIO_PIN_9
#define PORT_RIGHT			GPIOF


struct HC_SR04
{

};

void uDelayTim1(uint64_t uTenSec);
void uDelayTim7(uint64_t uSec);
uint32_t get_pulses_cnt(void);
void clear_pulses_cnt(void);
uint64_t measureDistance(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint64_t triggerMeasureCenter(void);
uint64_t triggerMeasureLeft(void);
uint64_t triggerMeasureRight(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
