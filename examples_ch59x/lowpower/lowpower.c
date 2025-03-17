#include "ch32fun.h"
#include <stdio.h>

#define LED PA8
#define LED_ON FUN_LOW
#define LED_OFF FUN_HIGH
#define SLEEPTIME_MS 300

__HIGH_CODE
__INTERRUPT
void RTC_IRQHandler(void)
{
	// clear timer and trigger flags
	R8_RTC_FLAG_CTRL = (RB_RTC_TMR_CLR | RB_RTC_TRIG_CLR);
}

__HIGH_CODE
void blink_led(int n) {
	for(int i = 0; i < n; i++) {
		funDigitalWrite( LED, LED_ON );
		Delay_Ms( 33 );
		funDigitalWrite( LED, LED_OFF );
		Delay_Ms( 33 );
	}
}

__HIGH_CODE
int main()
{
	SystemInit();
	RTCInit();
    NVIC_EnableIRQ(RTC_IRQn);
    DCDCEnable();
	LowPowerInit();
	funPinMode( LED, GPIO_CFGLR_OUT_2Mhz_PP );
	blink_led(1);

	while(1)
	{
		LowPowerIdle( MS_TO_RTC(SLEEPTIME_MS) );
		blink_led(2);
		LowPowerSleep( MS_TO_RTC(SLEEPTIME_MS), (RB_PWR_RAM2K | RB_PWR_RAM24K | RB_PWR_CORE | RB_PWR_EXTEND | RB_XT_PRE_EN) );
		blink_led(3);
	}
}
