#include "ch32fun.h"
#include <stdio.h>

#define PIN_BUTTON PB22

int main()
{
	SystemInit();

	funGpioInitAll(); // no-op on ch5xx

	funPinMode( PIN_BUTTON, GPIO_CFGLR_IN_PU );

	SetupUART(FUNCONF_UART_PRINTF_BAUD); // UART1 on rx,tx:PA8,PA9 at 115200 baud (PA2,PA3 on ch570/2)
	printf("|.~'~.- ch5xx TX demo -.~'~.|\r\n"); // test long string

	u8 i = 0;
	char msg[] = "ch32fun is awesome!\r\n";
	while(1)
	{
		if(!funDigitalRead( PIN_BUTTON )) { // remove "if(!funDigitalRead( PIN_BUTTON ))" if you don't have a button
			putchar(msg[i++]);
		}
		i %= sizeof(msg);
		Delay_Ms( 100 );
	}
}
