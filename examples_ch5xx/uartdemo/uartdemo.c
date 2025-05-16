#include "ch32fun.h"
#include <stdio.h>

int main()
{
	SystemInit();

	funGpioInitAll(); // Enable GPIOs

	funPinMode( PB22, GPIO_CFGLR_IN_PU );

	SetupUART(FUNCONF_UART_PRINTF_BAUD); // UART1 on rx,tx:PA8,PA9 at 115200 baud (PA2,PA3 on ch570/2)
	printf("TX in on PA3 or PA9, RX is on PA2 or PA8 but not implemented in this demo.\r\n");

	u8 i = 0;
	char msg[] = "ch32fun is awesome!\r\n";
	while(1)
	{
		if(!funDigitalRead( PB22 )) { // remove "if(!funDigitalRead( PB22 ))" if you don't have a button on PB22
			putchar(msg[i++]);
		}
		i %= sizeof(msg);
		Delay_Ms( 100 );
	}
}
