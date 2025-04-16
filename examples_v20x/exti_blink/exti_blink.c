#include "ch32fun.h"

// specify interrupt line
void EXTI1_IRQHandler( void ) __attribute__((interrupt));

// set state of the LED on PA10
int state = 1;

int main()
{
	SystemInit();

	funGpioInitAll();

	// configure PA10 for output and set it high
	funPinMode( PA10, GPIO_CFGLR_OUT_10Mhz_PP );
	funDigitalWrite( PA10, FUN_HIGH );

    // configure PA 1 for floating input
    funPinMode( PA1, GPIO_CFGLR_IN_FLOAT );


    // Seting the Registers for interrupt
    // attached to PA1
    AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI1_PA;
    EXTI->INTENR = EXTI_INTENR_MR1; // Enable EXTI1 interrupt
    EXTI->RTENR = EXTI_RTENR_TR1;   // enable rising edge trigger
    EXTI->FTENR = 0; // disable falling edge trigger

    // enable toggle interuppt
	NVIC_EnableIRQ( EXTI1_IRQn );

    EXTI->INTFR = EXTI_Line1;

	while(1)
	{

	}
}

/*
Interrupt code
	Checks if it was set low or high last
	and toggles it based on that info.
*/
void EXTI1_IRQHandler( void ) {
	// verify the interrupt flag occured.
    if (EXTI->INTFR & EXTI_Line1) {
        if (state == 0)
        {
            funDigitalWrite( PA10, FUN_HIGH );
            state = 1;
        } else
        {
            funDigitalWrite( PA10, FUN_LOW );
            state = 0;
        }
        EXTI->INTFR = EXTI_Line1; // clear interrupt flag
    }
}