#include "main.h"

extern UART_HandleTypeDef huart3;
#if !defined(__MICROLIB)
#pragma import(__use_no_semihosting_swi) 

struct __FILE
{
    int handle;
};

FILE __stdout;

void  _sys_exit( int x)
{
x=x;
}

void __ttywrch(int ch)
	{
	ch=ch;
}
	
#endif 

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE  int fputc(int ch, FILE *f)
#endif 

PUTCHAR_PROTOTYPE
{
		HAL_UART_Transmit(&huart3 , (uint8_t *)&ch, 1, 0xffff);
		return ch;
}


int fgetc(FILE * f)
{ uint8_t ch;
  ch =HAL_UART_Receive(&huart3,&ch, 1, 0xffff);
  return ch;
}


