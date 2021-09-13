/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : hcSensor.c
  * @brief          : 
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "hcSensor.h"

volatile static uint32_t pulses_cnt = 0;
const float soundSpeed = 0.0343;
extern float soundSpeed2;

void uDelayTim1(uint64_t uTenSec)
{
//	uint8_t cnt;
//	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_OnePulse_Start(&htim1, HAL_TIM_ACTIVE_CHANNEL_1);
//	HAL_TIM_OnePulse_Start_IT(&htim1, TIM_CHANNEL_ALL);
//	 __HAL_TIM_ENABLE(&htim1);    //send pulse
//	while(TIM1->CNT!=0){
//		cnt++;
//	}
}

void uDelayTim7(uint64_t uSec)
{
	if(uSec < 2) uSec = 2;
	TIM7->ARR = uSec - 1;
	TIM7->EGR = 1;
	TIM7->SR %= ~1;
	TIM7->CR1 |= 1;
	while((TIM7->SR&0x0001) != 1);
	TIM7->SR &= ~(0x0001);
}

uint32_t get_pulses_cnt(void) {
	return pulses_cnt;
}

void clear_pulses_cnt(void) {
	pulses_cnt = 0;
}

uint64_t measureDistance(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	while(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET);
	cntTics = 0;

	while(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_SET)
	{
		cntTics++;
		uDelayTim7(2);
	}

	//distance = cntTics * soundSpeed * 1.5;
	distance = cntTics * soundSpeed2 * 1.5;
	return distance;
}

uint64_t triggerMeasureCenter(void)
{
	HAL_GPIO_WritePin(PORT_CENTER, PIN_CENTER, GPIO_PIN_SET);
	uDelayTim1(10);
	HAL_GPIO_WritePin(PORT_CENTER, PIN_CENTER, GPIO_PIN_RESET);

	/*__HAL_TIM_ENABLE_IT(&htim7, TIM_IT_CC1);
	HAL_TIM_Base_Start(&htim7);
	HAL_TIM_Base_Start_IT(&htim7);*/

	distanceCenter = measureDistance(PORT_CENTER, PIN_CENTER_ECHO);

	return distanceCenter;
}

uint64_t triggerMeasureLeft(void)
{
	HAL_GPIO_WritePin(PORT_LEFT, PIN_LEFT, GPIO_PIN_SET);
	uDelayTim1(10);
	HAL_GPIO_WritePin(PORT_LEFT, PIN_LEFT, GPIO_PIN_RESET);

	distanceLeft = measureDistance(PORT_LEFT, PIN_LEFT_ECHO);

	return distanceLeft;
}

uint64_t triggerMeasureRight(void)
{
	HAL_GPIO_WritePin(PORT_RIGHT, PIN_RIGHT, GPIO_PIN_SET);
	uDelayTim1(10);
	HAL_GPIO_WritePin(PORT_RIGHT, PIN_RIGHT, GPIO_PIN_RESET);

	distanceRight = measureDistance(PORT_RIGHT, PIN_RIGHT_ECHO);

	return distanceRight;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_2)
	{
		//__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
		//HAL_TIM_Base_Start(&htim7);
		//HAL_TIM_Base_Start_IT(&htim7);
		//ready = 1;

		//__HAL_TIM_SET_COUNTER(htim, 0);
	}

}

/************************ (C) COPYRIGHT *****END OF FILE****/
