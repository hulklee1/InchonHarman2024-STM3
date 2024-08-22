/*
 * lcd.c
 *
 *  Created on: Jul 31, 2024
 *      Author: user
 */
#include "main.h"
#include "myLib.h"
extern I2C_HandleTypeDef hi2c1;

int i2c_scan() // 현재 연결되어 있는 I2C device의 주소를 표시하고 반환
{
	int addr,ret;
	for(addr=0;addr<128;addr++)
	{
		if(HAL_I2C_IsDeviceReady(&hi2c1, addr<<1, 1, 10) == HAL_OK)
		{
			printf(" 0x%02x", addr); ret = addr;
		}
		else printf("  .  ");
		if((addr % 8) == 0)	printf("\r\n");
		HAL_Delay(10);
	}
	printf("\r\n");
	return ret;
}

void lcd_command(char cmd)  //  cmd : abcd efgh
{
	char d1,d2,data[4];
	d1 = cmd & 0xf0; 		//  d1 : abcd 0000
	d2 = (cmd & 0x0f)<<4;	//  d2 : 0000 efgh ==> efgh 0000
	data[0] = d1 | 0x0c;	//  0c : 0000 1100 => write|enable|NC|RS(1/0)
	data[1] = d1 | 0x08;	//  08 : 0000 1000 => write|disable|NC|RS
	data[2] = d2 | 0x0c;
	data[3] = d2 | 0x08;
	HAL_I2C_Master_Transmit(&hi2c1, LCD1602_ADDR, data, 4, 10);
}

void lcd_data(char ch)  //  ch : abcd efgh,  단일 문자 전송
{
	char d1,d2,data[4];
	d1 = ch & 0xf0; 		//  d1 : abcd 0000
	d2 = (ch & 0x0f)<<4;	//  d2 : 0000 efgh ==> efgh 0000
	data[0] = d1 | 0x0d;
	data[1] = d1 | 0x09;
	data[2] = d2 | 0x0d;
	data[3] = d2 | 0x09;
	HAL_I2C_Master_Transmit(&hi2c1, LCD1602_ADDR, data, 4, 10);
}

void lcd_print(char *s)
{
	while(*s) lcd_data(*s++);
}

void lcd_printEx(char *s, int ln)
{
	if(ln == 1) 	 lcd_command(0x80);
	else if(ln == 2) lcd_command(0xc0);
	while(*s) lcd_data(*s++);
}

void lcd_init()  // 1602 LCD 초기화 : Disp ON, Cursor Position, etc...
{
	lcd_command(0x01); // screen clear
	lcd_command(0x02); // cursor 초기 위치
	lcd_command(0x06); // 문자 입력 모드 : 좌 -> 우
	lcd_command(0x0f); // display mode : Disp On, Curs On, Flash On
}








