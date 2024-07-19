/*
 * myLib.c
 *
 *  Created on: Jul 4, 2024
 *      Author: user
 */
#include "main.h"
#include "myLib.h"

extern UART_HandleTypeDef huart2;

myBuffer Buf;

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
//__weak extern TIM_HandleTypeDef htim3;
//void microdelay(int us)
//{
//	int t1 = htim3.Instance->CNT;
//	while(htim3.Instance->CNT - t1 < us);
//}
void Dump(int n) // 1,2,4... (byte)
{
	int max_row = 20;
	int max_col = 16 / n;
	char str[10];
	sprintf(str,"%%0%dx ", n*2);  // printf format string %02x %04x %08x
	for(int i=0;i<max_row;i++) // ROW index
	{
	  for(int j=0;j<max_col;j++)	// COLUMN index
	  {
		  unsigned int v =
				(n == 1)? Buf.v0[(i*16)+j] :
				(n == 2)? Buf.v1[(i*8)+j] :
				(n == 4)? Buf.v2[(i*4)+j] : -1;
		  printf(str, v);
		  if(j == (8 / n) - 1) printf("  ");
	  }
	  printf("\r\n");
	}
}

void cls()
{
	printf("\033[0J\033[1J\033[2J\033[1;1H\n"); // screen clear & move cur to (1,1)
}
void CursorUp()
{
	printf("\033[A");
}
void Cursor(int OnOff)
{
	if(OnOff) printf("\033[?25h\n"); else printf("\033[?25l\n");
}
void Wait()
{
	while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) != 0); // B1 for start
}
void ProgramStart(char *name)
{
	//printf("\033[2J\033[1;1H\n"); // [y;xH : move cur to (x,y)
	cls();
	printf("Program(%s) ready. Press Blue button to start\r\n", name);
	Cursor(0);
	Wait();    //while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) != 0); // B1 for start
}


