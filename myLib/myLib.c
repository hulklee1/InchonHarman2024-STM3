/*
 * myLib.c
 *
 *  Created on: Jul 4, 2024
 *      Author: user
 */
#include "main.h"

extern UART_HandleTypeDef huart2;

int __io_putchar(int ch) // 1 char output to terminal
{
	HAL_UART_Transmit(&huart2, &ch, 1, 10);
	return ch;
}
int __io_getchar(void)
{
	char ch;
	while(HAL_UART_Receive(&huart2, &ch, 1, 10) != HAL_OK) ;
	HAL_UART_Transmit(&huart2, &ch, 1, 10);  // echo
	if(ch == '\r') HAL_UART_Transmit(&huart2, "\n", 1, 10);
	return ch;
}
void ProgramStart(char *name)
{
	printf("\033[2J\033[1;1H\n"); // [y;xH : move cur to (x,y)
	printf("Program(%s) ready. Press Blue button to start\r\n", name);
	while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) != 0); // B1 for start
}


