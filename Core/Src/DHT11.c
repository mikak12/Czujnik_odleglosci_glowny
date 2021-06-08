#include "DHT11.h"

uint8_t checkDHT11;


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
	Set_Pin_Output(DHT11_PORT, DHT11_PIN);
	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 1);
	delayuSecTim16(4000000);
	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 0);
	delayuSecTim16(18000);
	Set_Pin_Input(DHT11_PORT, DHT11_PIN);
}

uint8_t checkResponse(void)
{
	uint8_t Response = 0;
	delayuSecTim16(40);
	if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)))
	{
		delayuSecTim16(80);
		if (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) 
			Response = 1;
		else Response = -1;
	}
	while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)==1);
	return Response;
}

uint8_t readDHT11(void)
{
	uint8_t i,j;
	for(j=0; j<8; j++)
	{
		while(!(HAL_GPIO_ReadPin(DHT11_PORT,DHT11_PIN)));
		delayuSecTim16(40);
		if(!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)))
		{
			i&=~(1UL<<(7-j));
			
		}
		else i|=(1UL<<(7-j));
		while((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)));
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

float soundSpeedCalibration(uint8_t tempByte1)
{

	float newSoundSpeed;
	float a = tempByte1/273.15;
	float b = 1+ a;
	float c = sqrt(b);

	newSoundSpeed = 331.5 * c * 0.0001;

	return newSoundSpeed;
}

