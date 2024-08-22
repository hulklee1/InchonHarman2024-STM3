/*
 * myLib.c
 *
 *  Created on: Jul 4, 2024
 *      Author: user
 */
#include "main.h"
#include "myLib.h"
#include <stdio.h>
extern UART_HandleTypeDef huart2;

myBuffer Buf;
myCMDSET myCmd[] =
{
	{"LED", 2},
	{"MOTOR", 2},
	{"BUZZER", 2},
	{"GO",1},
	{NULL,0}
};

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
int head = 0, tail = 0;
int GetBuffer(char* b)	// b : char array pointer for destination
{
	int len = 0;
	char* s = &Buf;                   //.v0[0];
	tail = MAX_BUF - huart2.hdmarx->Instance->NDTR;
	if(tail > head)
	{
		memcpy(b, s + head, tail - head);		// from head to Tail
		len = tail - head;  // length
	}
	else if(tail < head)
	{
		memcpy(b, s + head, MAX_BUF - head);	// from head to End
		memcpy(b + MAX_BUF - head, s, tail);	// from Start to Tail
		len = MAX_BUF - head + tail;  // length
	}
	else // tail == head
	{
		len = 0;
	}
	*(b + len) = 0;  // NULL
	head = tail;
	return len;
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

/* -------------------------------------------------------
 *  문자열 처리 함수
 * 1. char* Trim(char *s)
 * 2. int myStrncmp(char *s1, char *s2, int n)
 * 3. int myStrlen(char *s)
 * 4. int CheckCMD(char *s)
 * 5. int Parsing(char *s, char *p[])
 *
 ---------------------------------------------------------*/
void BufClear() { setvbuf(stdin, NULL, _IONBF, 0); } // BUFFER CLEAR
char* Trim(char *s)  // Delete white_space ("SPC \t \r \n")  "  ABC   " ==> "ABC"
{
	int h=0,t=strlen(s)-1;
	if(t < 0) return NULL;
	while(*(s+h) == ' ' || *(s+h) == '\t' || *(s+h) == '\r' || *(s+h) == '\n') h++;  // erase left spaces
	while(*(s+t) == ' ' || *(s+t) == '\t' || *(s+t) == '\r' || *(s+t) == '\n') t--;  // erase left spaces

	//if(h==0 && t==strlen(s)-1) return(s);
	char *ret = (char *)malloc(t-h+1+1);
	memcpy(ret, s+h, t-h+1); ret[t-h+1] = 0;
	return ret;
}

int myStrncmp(char *s1, char *s2, int n)  // 0: ==   , 1: !=
{
	for(int i=0;i<n;i++)
	{
		//if(*(s1+i)|0x20 != *(s2+i)|0x20) return 1;  // 0b0010 0000
		if(((*(s1+i))|0x20) != ((*(s2+i))|0x20)) return 1;  // 0b0010 0000
	}
	return 0;
}

int myStrlen(char *s)
{
	for(int i=0;;i++)
	{
		if(*(s+i) == 0 || *(s+i) == ' ' || *(s+i) == '\t' || *(s+i) == '\r' || *(s+i) == '\n') return i;
	}
	return -1;
}
int CheckCMD(char *s)	//*s = "   Led     1    On     "==> *b = "Led     1    On"
{
	int ret = 0;
	char *b = Trim(s), *c = NULL;  //
	if(*b == NULL) return 0;
	for(int i=0;myCmd[i].key;i++)
	{
		if(myStrncmp(b, myCmd[i].key, myStrlen(myCmd[i].key)) == 0)
		{
			for(int j=0;;j++)
			{
				int idx = myStrlen(b);
				if(c) free(c);
				c = Trim(b + idx); //*b = "Led     1    On" ==> *c = "1    On"
				free(b); b = c;
				if(myStrlen(c) == 0) { ret = 1; break; }

//				int n = myCmd[i].op_no;
//				while(*c) { if(*c == ' ') n--; c++; }
//				if(n == 0) return 1; else return 0;
			}
		}
	}
	return ret;
}

int Parsing(char *s, char *p[])  // *s = "   Led     1    On     "   ret v = 3
{
	/*if(!CheckCMD(b)) return 0;
	int n = 0;  //  index of p[]
	p[n++] = b;
	for(int i=0;b[i] != 0;i++)  // b[i] != 0 ==> b[i]  ==> *(b+i)
	{
		if(b[i] == ' ')
		{
			b[i] = 0;	// '\0'
			p[n++] = b + i + 1;
		}
	}
	return n;  */
	////while(*b) {	if(*b == ' '){	*b = '\0';	p[n++] = ++b;	}	}

	int ret = 0;
	char *b = Trim(s), *c = NULL;  //

	if(*b == NULL) return 0;
	for(int i=0;myCmd[i].key;i++)
	{
		if(myStrncmp(b, myCmd[i].key, myStrlen(myCmd[i].key)) == 0)
		{
			for(int j=0;;j++)
			{
				p[j] = b;
				int idx = myStrlen(b);
				if(strlen(b) == idx) { ret = j; break; }
				*(b+idx) = 0; //
				//if(c) free(c);
				c = Trim(b + idx + 1); //*b = "Led     1    On" ==> *c = "1    On"
				//free(b);
				b = c;
				if(myStrlen(c) == 0) { ret = j; break; }

//				int n = myCmd[i].op_no;
//				while(*c) { if(*c == ' ') n--; c++; }
//				if(n == 0) return 1; else return 0;
			}
		}
	}
	return ret;
}


