#include "DHT11.h"

uint8_t checkDHT11;

float soundSpeed2 = 0.0343;
float tempTemp = 15;


void delayuSecTim16(uint32_t uSec)
{
	if(uSec < 2) uSec = 2;
	TIM16->ARR = uSec - 1;
	TIM16->EGR = 1;
	TIM16->SR %= ~1;
	TIM16->CR1 |= 1;
	while((TIM16->SR&0x0001) != 1);
	TIM16->SR &= ~(0x0001);

}


void delay22(uint16_t time)
{
	HAL_TIM_Base_Start_IT(&htim16);
	__HAL_TIM_SET_COUNTER(&htim16, 0);
	while((__HAL_TIM_GET_COUNTER(&htim16))<time);

	HAL_TIM_Base_Stop_IT(&htim16);
}

/*
void odlczanie(uint32_t usekundy)
{
	uint32_t i = 0;
	for(i=0; i<usekundy; i++)
	{
		delay22(1);
	}
}*/


void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}



void startDHT11(void)
{
	//Set_Pin_Output(DHT11_PORT, DHT11_PIN);
	Set_Pin_Output(DHT11_2_PORT, DHT11_2_PIN);
	//HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 0);
	HAL_GPIO_WritePin(DHT11_2_PORT, DHT11_2_PIN, 0);
	delay22(18000);
	//HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 1);
	HAL_GPIO_WritePin(DHT11_2_PORT, DHT11_2_PIN, 1);
	delay22(20);
	//Set_Pin_Input(GPIOA, GPIO_PIN_3);
	Set_Pin_Input(DHT11_2_PORT, DHT11_2_PIN);
}

uint8_t checkResponse(void)
{
	uint8_t Response = 0;
	uint16_t i = 0;
	//delayuSecTim16(40);
	delay22(40);
	if (!(HAL_GPIO_ReadPin(DHT11_2_PORT, DHT11_2_PIN)))
	{
		//delayuSecTim16(80);
		delay22(80);
		if (HAL_GPIO_ReadPin(DHT11_2_PORT, DHT11_2_PIN))
			Response = 1;
		else Response = -1;
	}
	if(Response == 1)
	{
		while(HAL_GPIO_ReadPin(DHT11_2_PORT, DHT11_2_PIN));
	}
	else
	{

	}

	return Response;
}

uint8_t readDHT11(void)
{
	uint8_t i,j;
	for(j=0; j<8; j++)
	{
		while(!(HAL_GPIO_ReadPin(DHT11_2_PORT, DHT11_2_PIN)));
		delay22(40);
		if(!(HAL_GPIO_ReadPin(DHT11_2_PORT, DHT11_2_PIN)))
		{
			i&=~(1UL<<(7-j));
			
		}
		else i|=(1UL<<(7-j));
		while((HAL_GPIO_ReadPin(DHT11_2_PORT, DHT11_2_PIN)));
	}
	return i;
	
	
}

uint8_t measureDHT11(void)
{
	uint8_t responde;
	uint8_t humidityByte1;
	uint8_t humidityByte2;
	uint8_t tempByte1;
	uint8_t tempByte2;

	startDHT11();
	//responde = checkResponse();
	//HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 0);
	//HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 1);
	//delay22(20);
	//HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 0);
	//checkDHT11 = 0;
	if(checkDHT11 != 0)
	{
		//checkResponse();
		responde = checkResponse();
		humidityByte1 = readDHT11();
		humidityByte2 = readDHT11();
		tempByte1 = readDHT11();
		tempByte2 = readDHT11();
	}
	checkDHT11 = 1;


	return tempByte1;
}

uint16_t soundSpeedCalibration(uint8_t tempByte1)
{

	float newSoundSpeed;
	uint8_t tempTempByte1 = tempByte1;
	float a = tempByte1 * 0.6;

	newSoundSpeed = 331.5 + a;

	if((tempByte1 + 5 < tempTemp) || (tempByte1 - 5 > tempTemp))
	{

	}
	else
	{
		tempTemp = tempTempByte1;
		soundSpeed2 = newSoundSpeed / 1000;
	}



	return newSoundSpeed;
}

