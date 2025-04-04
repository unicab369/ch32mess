#include "ch32fun.h"
#include <stdio.h>


uint16_t initial_count = 0;
uint16_t last_count = 0;


//# Timer 2 pin mappings by AFIO->PCFR1
/*	00	AFIO_PCFR1_TIM2_REMAP_NOREMAP
		D4		T2CH1ETR
		D3		T2CH2
		C0		T2CH3
		D7		T2CH4  		//? requires disabling nRST in opt
	01	AFIO_PCFR1_TIM2_REMAP_PARTIALREMAP1
		C5		T2CH1ETR_	//! SPI-SCK
		C2		T2CH2_		//! I2C-SDA
		D2		T2CH3_
		C1		T2CH4_		//! I2C-SCL
	10	AFIO_PCFR1_TIM2_REMAP_PARTIALREMAP2
		C1		T2CH1ETR_	//! I2C-SCL
		D3		T2CH2		
		C0		T2CH3
		D7		T2CH4  		//? requires disabling nRST in opt
	11	AFIO_PCFR1_TIM2_REMAP_FULLREMAP
		C1		T2CH1ETR_	//! I2C-SCL
		C7		T2CH2_		//! SPI-MISO
		D6		T2CH3_		//! UART_TX
		D5		T2CH4_		//! UART_RX
*/

//# Timer 1 pin mappings by AFIO->PCFR1
/*  00	AFIO_PCFR1_TIM1_REMAP_NOREMAP
        (ETR/PC5, BKIN/PC2)
        CH1/CH1N PD2/PD0
        CH2/CH2N PA1/PA2
        CH3/CH3N PC3/PD1	//! PD1 SWIO
        CH4 PC4
    01	AFIO_PCFR1_TIM1_REMAP_PARTIALREMAP1
        (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PA6, CH1N/PA7, CH2N/PB0, CH3N/PB1)
        CH1/CH1N PC6/PC3	//! PC6 SPI-MOSI
        CH2/CH2N PC7/PC4	//! PC7 SPI-MISO
        CH3/CH3N PC0/PD1	//! PD1 SWIO
        CH4 PD3
    10	AFIO_PCFR1_TIM1_REMAP_PARTIALREMAP2
        (ETR/PD4, CH1/PD2, CH2/PA1, CH3/PC3, CH4/PC4, BKIN/PC2, CH1N/PD0, CN2N/PA2, CH3N/PD1)
        CH1/CH1N PD2/PD0
        CH2/CH2N PA1/PA2
        CH3/CH3N PC3/PD1	//! PD1 SWIO
        CH4 PC4
    11	AFIO_PCFR1_TIM1_REMAP_FULLREMAP
        (ETR/PE7, CH1/PE9, CH2/PE11, CH3/PE13, CH4/PE14, BKIN/PE15, CH1N/PE8, CH2N/PE10, CH3N/PE12)
        CH1/CH1N PC4/PC3	
        CH2/CH2N PC7/PD2	//! PC7 SPI-MISO
        CH3/CH3N PC5/PC6	//! PC5 SPI-SCK, PC6 SPI-MOSI
        CH4 PD2
*/


void modEncoder_setup() {
	//! Enable GPIOC, TIM2, and AFIO *very important!*
	RCC->APB2PCENR |= RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOD;
	RCC->APB1PCENR |= RCC_APB1Periph_TIM2;

	//! TIM2 remap mode
	AFIO->PCFR1 |= AFIO_PCFR1_TIM2_REMAP_NOREMAP;

	// PC2 is T2CH1_, Input w/ Pullup/down
	GPIOD->CFGLR &= ~(0xf<<(4*3)); 					//clear old values
	GPIOD->CFGLR |= (GPIO_CNF_IN_PUPD)<<(4*3); 		//set new ones
	GPIOD->OUTDR |= 1<<2;							//1 = pull-up, 0 = pull-down

	// PC5 is T2CH2_, Input w/ Pullup/down
	GPIOD->CFGLR &= ~(0xf<<(4*4)); 					//clear values
	GPIOD->CFGLR |= (GPIO_CNF_IN_PUPD)<<(4*4); 		//set new ones
	GPIOD->OUTDR |= 1<<4;							//1 = pull-up, 0 = pull-down
	
	//! Reset TIM2 to init all regs
	RCC->APB1PRSTR |= RCC_APB1Periph_TIM2;
	RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM2;
	
	// set TIM2 clock prescaler If you want to reduce the resolution of the encoder
	//TIM2->PSC = 0x0000;

	// set a automatic reload if you want the counter to wrap earlier than 0xffff
	//TIM2->ATRLR = 0xffff;

	// SMCFGR: set encoder mode SMS=011b
	TIM2->SMCFGR |= TIM_EncoderMode_TI12;

	// initialize timer
	TIM2->SWEVGR |= TIM_UG;

	// set count to about mid-scale to avoid wrap-around
	TIM2->CNT = 0x8fff;

	// Enable TIM2
	TIM2->CTLR1 |= TIM_CEN;

	initial_count = TIM2->CNT;
	last_count = TIM2->CNT;
};

static uint32_t encoder_debounceTime = 0;

void modEncoder_task(uint32_t current_time) {
	// if (current_time - encoder_debounceTime < 50) return;
	// encoder_debounceTime = current_time;

	uint16_t count = TIM2->CNT;
	if( count != last_count) {
		printf("Position relative=%ld absolute=%u delta=%ld\n",
				(int32_t)count - initial_count, count, (int32_t)count-last_count);
		last_count = count;
	}
}