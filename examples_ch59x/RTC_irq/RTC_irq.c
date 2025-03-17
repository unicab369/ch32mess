#include "ch32fun.h"
#include <stdio.h>

#define LED PA8

void RTC_IRQHandler(void) __attribute__((interrupt));
void RTC_IRQHandler(void)
{
	// clear timer and trigger flags
	R8_RTC_FLAG_CTRL = (RB_RTC_TMR_CLR | RB_RTC_TRIG_CLR);

	// Flip the gpio (GPIO_InverseBits is a ch5xx specific macro)
	GPIO_InverseBits(LED);

	// Set a trigger again.
	// The TMR function of the RTC can also be used for this,
	// but like this the demo basically shows both
	RTCTrigger( MS_TO_RTC(333) );
}


int main(void)
{
	SystemInit();
	RTCInit();
	NVIC_EnableIRQ(RTC_IRQn);

	funPinMode(LED, GPIO_CFGLR_OUT_2Mhz_PP);
	RTCTrigger( MS_TO_RTC(333) );

	while(1);
}
