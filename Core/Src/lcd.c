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
#include "lcd.h"

uint8_t row;
uint8_t col;
uint8_t j;

uint8_t dotOrNot;


void lcdInit(struct lcdConf * lcd)
{
	uint64_t i;
	HAL_GPIO_WritePin(lcd->rst_port, lcd->rst_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(lcd->rst_port, lcd->rst_pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(lcd->bl_port, lcd->bl_pin, GPIO_PIN_SET);

	lcdCmdMode(lcd, 0x21);
	lcdCmdMode(lcd, 0x04);
	lcdCmdMode(lcd, 0x10 | 0x04);
	lcdCmdMode(lcd, 0x80 | 0x38);
	lcdCmdMode(lcd, 0x20);
	lcdCmdMode(lcd, 0x08 | 0x04);

	for(i=0; i<348; i++)
	{
		lcdCmdMode(lcd, 0x00);
	}
}

void lcdCmdMode(struct lcdConf * lcd, uint8_t cmd)
{
	HAL_GPIO_WritePin(lcd->dc_port, lcd->dc_pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(lcd->ce_port, lcd->ce_pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(lcd->spi, &cmd, 1, 100);

	HAL_GPIO_WritePin(lcd->ce_port, lcd->ce_pin, GPIO_PIN_SET);

	if(cmd != 0x00 )
	{
		dotOrNot++;
	}
}

void lcdDataMode(struct lcdConf * lcd, uint8_t data)
{
	HAL_GPIO_WritePin(lcd->dc_port, lcd->dc_pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(lcd->ce_port, lcd->ce_pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(lcd->spi, &data, 1, 100);

	HAL_GPIO_WritePin(lcd->ce_port, lcd->ce_pin, GPIO_PIN_SET);

}

void lcdSetPlace(struct lcdConf * lcd, enum displayDirection direction, uint8_t col, uint8_t row)
{
	if(direction == vertical)
	{
		lcdCmdMode(lcd, 0x20 | 0x02);
	}
	else
	{
		lcdCmdMode(lcd, 0x20 | 0x00);
	}
	lcdCmdMode(lcd, 0x80 | col);
	lcdCmdMode(lcd, 0x40 | row);
}

void lcdWelcome(struct lcdConf  lcd)
{
	lcdClearBuffer(&lcd);
	lcdSetPlace(&lcd, horizontal, 0, 0);

	char layer1[] = "WITAMY";

	memcpy(&(lcd.buffer[1][3]), layer1, strlen(layer1));
	lcdReadBuffer(&lcd);

	char layer2[] = "W";

	memcpy(&(lcd.buffer[2][6]), layer2, strlen(layer2));
	lcdReadBuffer(&lcd);

	char layer3[] = "IZER A";

	memcpy(&(lcd.buffer[3][3]), layer3, strlen(layer3));
	lcdReadBuffer(&lcd);

	char layer4[] = "({)";

	memcpy(&(lcd.buffer[4][6]), layer4, strlen(layer4));
	lcdReadBuffer(&lcd);
}


void lcdMarkPrint(struct lcdConf * lcd, char mark)
{
    for (uint8_t i = 0; i < 5; i++)
    {
    	lcdDataMode(lcd, display_font[mark - 0x20][i]);
    }
    if(dotOrNot != 0)
    {
    	lcdDataMode(lcd, 0x80);
    }
    else
    {
    	lcdDataMode(lcd, 0x00);
    }
    dotOrNot = 0;
}

void lcdReadBuffer(struct lcdConf * lcd)
{
	lcdSetPlace(lcd, horizontal, 0, 0);

    for(uint8_t i=0; i<5; i++)
        for(uint8_t j=0; j<14; j++)
        	lcdMarkPrint(lcd, lcd->buffer[i][j]);
}

void lcdClearBuffer(struct lcdConf * lcd)
{
    for(uint8_t i=0; i<6; i++)
        for(uint8_t j=0; j<14; j++)
        	lcd->buffer[i][j] = ' ';
}


// obsluga

void display_based(struct lcdConf * lcd)
{
	lcdClearBuffer(lcd);
	lcdSetPlace(lcd, horizontal, 0, 0);

	char layer1[] = "|       }";

	memcpy(&(lcd->buffer[1][2]), layer1, strlen(layer1));
	lcdReadBuffer(lcd);

	char layer2[] = "|_ ___ _}";

	memcpy(&(lcd->buffer[2][2]), layer2, strlen(layer2));
	lcdReadBuffer(lcd);
}

void display_lvl1_based(struct lcdConf * lcd)
{
	lcdClearBuffer(lcd);
	lcdSetPlace(lcd, horizontal, 0, 0);

	char layer1[] = "||       }}";

	memcpy(&(lcd->buffer[1][1]), layer1, strlen(layer1));
	lcdReadBuffer(lcd);

	char layer2[] = "||_ ___ _}}";

	memcpy(&(lcd->buffer[2][1]), layer2, strlen(layer2));
	lcdReadBuffer(lcd);

	char layer3[] = "|__ ___ __}";

	memcpy(&(lcd->buffer[3][1]), layer3, strlen(layer3));
	lcdReadBuffer(lcd);

}

void display_lvl2_based(struct lcdConf * lcd)
{
	lcdClearBuffer(lcd);
	lcdSetPlace(lcd, horizontal, 0, 0);

	char layer1[] = "|||       }}}";

	memcpy(&(lcd->buffer[1][0]), layer1, strlen(layer1));
	lcdReadBuffer(lcd);

	char layer2[] = "|||_ ___ _}}}";

	memcpy(&(lcd->buffer[2][0]), layer2, strlen(layer2));
	lcdReadBuffer(lcd);

	char layer3[] = "||__ ___ __}}";

	memcpy(&(lcd->buffer[3][0]), layer3, strlen(layer3));
	lcdReadBuffer(lcd);

	char layer4[] = "|___ ___ ___}";

	memcpy(&(lcd->buffer[4][0]), layer4, strlen(layer4));
	lcdReadBuffer(lcd);

}

void display_lvl1_srodek(struct lcdConf * lcd, uint32_t centerSensor)
{
	lcdClearBuffer(lcd);
	lcdSetPlace(lcd, horizontal, 0, 0);

	//char layer;

	//sprintf(layer, "%d", centerSensor);

	//union uintToChar pp;
	//pp.x = centerSensor;
	/*
	uint8_t a = centerSensor/100;
	centerSensor = centerSensor%100;
	uint8_t b = centerSensor/10;
	centerSensor = centerSensor%10;
	uint8_t c = centerSensor/1;
	char layer[2];
	layer[2]=a;
	layer[1]=b;
	layer[0]=c;

	memcpy(&(lcd->buffer[0][5]), layer, strlen(layer));
	lcdReadBuffer(lcd);*/

	char layer1[] = "  |       }  ";

	memcpy(&(lcd->buffer[1][0]), layer1, strlen(layer1));
	lcdReadBuffer(lcd);

	char layer2[] = "  |_ ___ _}  ";

	memcpy(&(lcd->buffer[2][0]), layer2, strlen(layer2));
	lcdReadBuffer(lcd);

	char layer3[] = "     ___     ";

	memcpy(&(lcd->buffer[3][0]), layer3, strlen(layer3));
	lcdReadBuffer(lcd);


}

void display_lvl1_left(struct lcdConf * lcd)
{
	lcdClearBuffer(lcd);
	lcdSetPlace(lcd, horizontal, 0, 0);

	char layer1[] = " ||       }  ";

	memcpy(&(lcd->buffer[1][0]), layer1, strlen(layer1));
	lcdReadBuffer(lcd);

	char layer2[] = " ||_ ___ _}  ";

	memcpy(&(lcd->buffer[2][0]), layer2, strlen(layer2));
	lcdReadBuffer(lcd);

	char layer3[] = " |__          ";

	memcpy(&(lcd->buffer[3][0]), layer3, strlen(layer3));
	lcdReadBuffer(lcd);
}

void display_lvl1_right(struct lcdConf * lcd)
{
	lcdClearBuffer(lcd);
	lcdSetPlace(lcd, horizontal, 0, 0);

	char layer1[] = " |       }}";

	memcpy(&(lcd->buffer[1][1]), layer1, strlen(layer1));
	lcdReadBuffer(lcd);

	char layer2[] = " |_ ___ _}}";

	memcpy(&(lcd->buffer[2][1]), layer2, strlen(layer2));
	lcdReadBuffer(lcd);

	char layer3[] = "        __}";

	memcpy(&(lcd->buffer[3][1]), layer3, strlen(layer3));
	lcdReadBuffer(lcd);
}

void display_lvl2_srodek(struct lcdConf * lcd)
{
	lcdClearBuffer(lcd);
	lcdSetPlace(lcd, horizontal, 0, 0);

	char layer1[] = "  |       }  ";

	memcpy(&(lcd->buffer[1][0]), layer1, strlen(layer1));
	lcdReadBuffer(lcd);

	char layer2[] = "  |_ ___ _}  ";

	memcpy(&(lcd->buffer[2][0]), layer2, strlen(layer2));
	lcdReadBuffer(lcd);

	char layer3[] = "     ___     ";

	memcpy(&(lcd->buffer[3][0]), layer3, strlen(layer3));
	lcdReadBuffer(lcd);

	char layer4[] = "     ___     ";

	memcpy(&(lcd->buffer[4][0]), layer4, strlen(layer4));
	lcdReadBuffer(lcd);


}

void display_lvl2_left(struct lcdConf * lcd)
{
	lcdClearBuffer(lcd);
	lcdSetPlace(lcd, horizontal, 0, 0);

	char layer1[] = "|||       }  ";

	memcpy(&(lcd->buffer[1][0]), layer1, strlen(layer1));
	lcdReadBuffer(lcd);

	char layer2[] = "|||_ ___ _}  ";

	memcpy(&(lcd->buffer[2][0]), layer2, strlen(layer2));
	lcdReadBuffer(lcd);

	char layer3[] = "||__         ";

	memcpy(&(lcd->buffer[3][0]), layer3, strlen(layer3));
	lcdReadBuffer(lcd);

	char layer4[] = "|___         ";

	memcpy(&(lcd->buffer[4][0]), layer4, strlen(layer4));
	lcdReadBuffer(lcd);


}

void display_lvl2_right(struct lcdConf * lcd)
{
	lcdClearBuffer(lcd);
	lcdSetPlace(lcd, horizontal, 0, 0);

	char layer1[] = "  |       }}}";

	memcpy(&(lcd->buffer[1][0]), layer1, strlen(layer1));
	lcdReadBuffer(lcd);

	char layer2[] = "  |_ ___ _}}}";

	memcpy(&(lcd->buffer[2][0]), layer2, strlen(layer2));
	lcdReadBuffer(lcd);

	char layer3[] = "         __}}";

	memcpy(&(lcd->buffer[3][0]), layer3, strlen(layer3));
	lcdReadBuffer(lcd);

	char layer4[] = "         ___}";

	memcpy(&(lcd->buffer[4][0]), layer4, strlen(layer4));
	lcdReadBuffer(lcd);
}





void display_driver(uint64_t leftSensor, uint64_t centerSensor, uint64_t rightSensor, struct lcdConf lcd)
{
	uint64_t mainSensor;
	uint64_t check;  // 1 -> center, 2-> left, 3-> right

	//rightSensor = 100;
	//centerSensor = 100;
	//leftSensor = 100;

	if(leftSensor < 50 && centerSensor < 50 && rightSensor < 50)
	{
		if(leftSensor < 20 && centerSensor < 20 && rightSensor < 20)
		{
			display_lvl2_based(&lcd);
		}
		else
		{
			display_lvl1_based(&lcd);
		}
	}
	else
	{
			mainSensor = centerSensor;
			check = 1;
			if(leftSensor < mainSensor)
			{
				mainSensor = leftSensor;
				check = 2;
			}

			if(rightSensor < mainSensor)
			{
				mainSensor = rightSensor;
				check = 3;
			}

			if(check == 1)
			{
				if(mainSensor > 100 || mainSensor == 0)
				{
					display_based(&lcd);
				}
				else if(mainSensor < 101 && mainSensor > 40)
				{
					display_lvl1_srodek(&lcd, centerSensor);
				}
				else if(mainSensor  < 41)
				{
					display_lvl2_srodek(&lcd);
				}
			}
			if(check == 2)
			{
				if(mainSensor > 100 || mainSensor == 0)
				{
					display_based(&lcd);
				}
				else if(mainSensor < 101 && mainSensor > 40)
				{
					display_lvl1_left(&lcd);
				}
				else if(mainSensor  < 41)
				{
					display_lvl2_left(&lcd);
				}
			}
			if(check == 3)
			{
				if(mainSensor > 100 || mainSensor == 0)
				{
					display_based(&lcd);
				}
				else if(mainSensor < 101 && mainSensor > 40)
				{
					display_lvl1_right(&lcd);
				}
				else if(mainSensor  < 41)
				{
					display_lvl2_right(&lcd);
				}
			}

	}

}

/************************ (C) COPYRIGHT *****END OF FILE****/
