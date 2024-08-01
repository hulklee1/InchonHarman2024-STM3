/*
 * led.c
 *
 *  Created on: Jul 26, 2024
 *      Author: user
 */
#include "main.h"

extern char * p[];
void LED_Control()
{
	int ln, st;
	sscanf(p[1],"%d",&ln);
	//if(strcmp(p[2], "ON") == 0) st = 1; else st = 0;
	st = (myStrncmp(p[2], "ON", 2) == 0) ? 1 : (myStrncmp(p[2], "OFF", 3) == 0) ? 0 : -1;
	switch(ln)
	{
	case 1:
		HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, st); break;
	case 2:
		HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin, st); break;
	case 3:
		HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin, st); break;
	}
	printf("%s %d had controled to %d\r\n", p[0], ln, st);
}


