/*
 * myLib.h
 *
 *  Created on: Jul 19, 2024
 *      Author: user
 */

#ifndef INC_MYLIB_H_
#define INC_MYLIB_H_

#define MAX_BUF 480
typedef union
{
	char	v0[MAX_BUF];
	short	v1[MAX_BUF/2];
	int		v2[MAX_BUF/4];
	long	v3[MAX_BUF/4];
} myBuffer;

#endif /* INC_MYLIB_H_ */
