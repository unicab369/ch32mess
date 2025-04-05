/*
 * Example of a useless code. It doesn't do anything good except being great at what it does.
 * It demonstrates the power of repetition: if you repeat the samething many times over, it becomes you.
 * 
 * fun_gpio.h is based on ch32v_hal.h by Larry Bank
 * fun_timer.h is based on Example SysTick with IRQs work of E. Brombaugh and C. Lohr
 * by unicab369
 */

// .\minichlink.exe -u
// make all monitor

#include "ch32fun.h"
#include <stdio.h>

#include "1_Foundation/fun_button.h"
// #include "3_Nrf/nrf24l01_main.h"

// #define PIN_LOWPOWER 0xD3
#define PIN_LOWPOWER 0xD4
#define PIN_BUTTON 0xD2
// #define PIN_LED 0xA2
#define PIN_LED 0xD0
#define PIN_MODE 0xA1
#define PIN_DONE 0xC3


void nrf_onReceive(uint8_t* data) {
	digitalWrite(PIN_LED, !digitalRead(PIN_LED));
	struct SensorData* receivedData = (struct SensorData*) data;

	// printf("\n\rRecieved");
	// printf("\n\rtemp=%lu, hum=%lu, lux=%lu, v=%lu, mA=%lu", 
	// 			receivedData->tempF, receivedData->hum, receivedData->lux, receivedData->voltage, receivedData->mA);
}

void make_inputPullups() {
	// Set all GPIOs to input pull up
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD;

	// GPIOA: Set to output
	GPIOA->CFGLR = (GPIO_CNF_IN_PUPD<<(4*2)) | (GPIO_CNF_IN_PUPD<<(4*1));
	GPIOA->BSHR = GPIO_BSHR_BS2 | GPIO_BSHR_BR1;

	GPIOC->CFGLR = (GPIO_CNF_IN_PUPD<<(4*7)) | (GPIO_CNF_IN_PUPD<<(4*6)) |
					(GPIO_CNF_IN_PUPD<<(4*5)) | (GPIO_CNF_IN_PUPD<<(4*4)) |
					(GPIO_CNF_IN_PUPD<<(4*3)) | (GPIO_CNF_IN_PUPD<<(4*2)) |
					(GPIO_CNF_IN_PUPD<<(4*1)) | (GPIO_CNF_IN_PUPD<<(4*0));
	GPIOC->BSHR = GPIO_BSHR_BS7 | GPIO_BSHR_BS6 | GPIO_BSHR_BS5 | GPIO_BSHR_BS4 |
					GPIO_BSHR_BS3 | GPIO_BSHR_BS2 | GPIO_BSHR_BS1 | GPIO_BSHR_BS0;

	GPIOD->CFGLR = (GPIO_CNF_IN_PUPD<<(4*7)) | (GPIO_CNF_IN_PUPD<<(4*6)) |
					(GPIO_CNF_IN_PUPD<<(4*5)) | (GPIO_CNF_IN_PUPD<<(4*4)) |
					(GPIO_CNF_IN_PUPD<<(4*3)) | (GPIO_CNF_IN_PUPD<<(4*2)) |
					(GPIO_CNF_IN_PUPD<<(4*0));
	GPIOD->BSHR = GPIO_BSHR_BS7 | GPIO_BSHR_BS6 | GPIO_BSHR_BS5 | GPIO_BSHR_BS4 |
					GPIO_BSHR_BS3 | GPIO_BSHR_BS2 | GPIO_BSHR_BS0;
	
}

void sleepMode_setup(uint8_t useButton) {
	// enable power interface module clock
	RCC->APB1PCENR |= RCC_APB1Periph_PWR;

	// enable low speed oscillator (LSI)
	RCC->RSTSCKR |= RCC_LSION;
	while ((RCC->RSTSCKR & RCC_LSIRDY) == 0) {}

	// enable AutoWakeUp event
	EXTI->EVENR |= EXTI_Line9;
	EXTI->FTENR |= EXTI_Line9;

	//# t = AWUWR*AWUPSC/fLSI
	// fLSI = 128000
	// AWUWR = 1 to 63
	// AWUPSC = 2048, 4096, 10240 or 61440, though lower values are possible.    

	PWR->AWUPSC |= PWR_AWU_Prescaler_4096;		// configure AWU prescaler
	PWR->AWUWR &= ~0x3f; PWR->AWUWR |= 63;		// configure AWU window comparison value
	PWR->AWUCSR |= (1 << 1);						// enable AWU

	//# standby_btn
	if (useButton) {
		RCC->APB2PCENR |= RCC_AFIOEN;							// AFIO is needed for EXTI
		AFIO->EXTICR |= (uint32_t)(0b11 << (2*2));		// assign pin 2 interrupt from portD (0b11) to EXTI channel 2

		// enable line2 interrupt event
		EXTI->EVENR |= EXTI_Line2;
		EXTI->FTENR |= EXTI_Line2;
	}

	//# sleep
	PWR->CTLR |= PWR_CTLR_PDDS;					// select standby on power-down	
	PFIC->SCTLR |= (1 << 2);						// peripheral interrupt controller send to deep sleep
}


uint32_t ref_counter = 0;
uint8_t delay_arr[4] = { 1, 2, 50, 100 };
uint8_t delay_index = 0;

uint8_t getDelay() {
	return delay_arr[delay_index];
}

uint32_t getRefIncrements() {
	uint32_t delayTime = getDelay();
	return delayTime*18;
}

void setLeds(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4, uint8_t pin5) {
	uint8_t delayTime = getDelay();

	pinMode(pin3, INPUT);
	pinMode(pin4, INPUT);
	pinMode(pin5, INPUT);
	pinMode(pin1, OUTPUT);
	pinMode(pin2, OUTPUT);

	digitalWrite(pin1, 1);
	digitalWrite(pin2, 0);
	Delay_Ms(delayTime);

	digitalWrite(pin1, 0);
	digitalWrite(pin2, 1);
	Delay_Ms(delayTime);
}

void turnOffGPIOs(uint8_t* arr, uint8_t len) {
	for (uint8_t i=0; i<len; i++) {
		pinMode(arr[i], OUTPUT);
		digitalWrite(arr[i], 0);
	}
}



//######### Button
#include "1_Foundation/fun_uart.h"
#include "1_Foundation/i2c_slave.h"
#include "1_Foundation/modPWM.h"
#include "1_Foundation/modEncoder.h"
#include "1_Foundation/modJoystick.h"
#include "Display/modST77xx.h"

#include "2_Device/modI2C.h"
#include "2_Device/fun_ws2812.h"

void button_onSingleClick() {
	printf("I'M USELESS.\n\r");
	digitalWrite(0xC0, !digitalRead(0xD0));
}

void button_onDoubleClick() {
	printf("I'M USELESS TWICE.\n\r");
}

volatile uint8_t i2c_registers[32] = {0xaa};

void onWrite(uint8_t reg, uint8_t length) {
	printf("IM HERE.\n\r");
    funDigitalWrite(PA2, i2c_registers[0] & 1);
}

void onRead(uint8_t reg) {
	printf("IM HERE.\n\r");
	digitalWrite(0xC0, !digitalRead(0xD0));
}


int main() {
	SystemInit();

	static const char message[] = "Hello World!\r\n";
	uint32_t ledc_time = 0;
	uint32_t sec_time = 0;
	uint32_t time_ref = 0;


	modI2C_setup();

	pinMode(0xD0, OUTPUT);
	button_setup(0xC3);	
	modJoystick_setup();		// ADC Ch0, Ch1 | DMA1 Ch1
	ws2812_setup();				// DMA1 Ch3

	modEncoder_setup();			// TIM2 Ch1, Ch2
	// modPWM_setup();				// TIM2 Ch3

	// led_setup();
	uart_setup();				// PD5
	dma_uart_setup();			// DMA1 Ch4
	
	for(;;) {			
		uint32_t now = millis();

		button_run();
		// modEncoder_task(now);

		if (now - sec_time > 400) {
			sec_time = now;

			// modJoystick_task();
			dma_uart_tx(message, sizeof(message) - 1);

			print_runtime("I2C", modI2C_task);
		}
		if (now - ledc_time > 12) {
			ledc_time = now;
			modPWM_task();
			ws2812_task();
		}
	}
}

//######### MAIN
uint8_t turnOff = 0;

int main2() {  
	SystemInit();
	pinMode(0xD2, OUTPUT);
	digitalWrite(0xD2, 1);
	pinMode(0xC0, INPUT);

	for(;;) {
		uint8_t read = digitalRead(0xC0);

		if (read) {
			uint32_t refIncrements = getRefIncrements();
			ref_counter+=refIncrements;

			if (ref_counter > 4000) {
				turnOff = 1;
			} else if (ref_counter > 1000) {
				delay_index++;
				if (delay_index>=sizeof(delay_arr)) delay_index = 0;
				ref_counter = 0;
			}
		} else {
			ref_counter = 0;
		}

		if (turnOff == 1) {
			uint8_t gpios[5] = { 0xD6, 0xC3, 0xD0, 0xD3, 0xC2 };
			turnOffGPIOs(gpios, sizeof(gpios));
			digitalWrite(0xD2, 0);
		} else {
			setLeds(0xD6, 0xC3, 0xD0, 0xD3, 0xC2);
			setLeds(0xD6, 0xD0, 0xC3, 0xD3, 0xC2);
			setLeds(0xD6, 0xD3, 0xC3, 0xD0, 0xC2);
			setLeds(0xD6, 0xC2, 0xC3, 0xD0, 0xD3);

			setLeds(0xC3, 0xD0, 0xD6, 0xD3, 0xC2);
			setLeds(0xC3, 0xD3, 0xD6, 0xD0, 0xC2);
			setLeds(0xC3, 0xC2, 0xD0, 0xD6, 0xD3);

			setLeds(0xD0, 0xD3, 0xC3, 0xD6, 0xC2);
			setLeds(0xD0, 0xC2, 0xC3, 0xD6, 0xD3);
		}
	}

	// // pinMode(PIN_LED, OUTPUT);
	// // pinMode(PIN_LOWPOWER, INPUT_PULLUP);
	// // pinMode(PIN_MODE, INPUT_PULLUP);
	// // pinMode(0xC3, OUTPUT);
	// // pinMode(PIN_DONE, OUTPUT);

	// // I2CInit(0xC1, 0xC2, 100000);
	// // BH17_Setup();

	// // button_setup(PIN_BUTTON);

	// // lowPower flag INPUT_PULLUP, Low for OFF
	// uint8_t readLowpower = digitalRead(PIN_LOWPOWER);
	// struct SensorData readings = { 0, 0, 0, 0, 0 };

	// // readLowpower == 1
	// if (readLowpower == 0) {
	// 	printf("\n\rIM HERE 1111");
	// 	nrf_setup(0);

	// 	for(;;) {
	// 		i2c_getReadings(&readings);
	// 		printf("\n\rtemp=%lu, hum=%lu, lux=%lu, v=%lu, mA=%lu", 
	// 					readings.tempF, readings.hum, readings.lux, readings.voltage, readings.mA);
	// 		sendData(&readings, sizeof(readings));
	// 		digitalWrite(PIN_LED, !digitalRead(PIN_LED));
	// 		digitalWrite(PIN_DONE, 1);
	// 		Delay_Ms(2000);
	// 	}
	// } else {
	// 	printf("\n\rIM HERE 2222");
	// 	nrf_setup(1);

	// 	// GPIO D0 Push-Pull for RX notification
	// 	RCC->APB2PCENR |= RCC_APB2Periph_GPIOD;
	// 	GPIOD->CFGLR &= ~(0xf<<(4*4));
	// 	GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*4);

	// 	for(;;) {
	// 		nrf_run(1);
	// 	}
	// }

	// if (readLowpower == 1) {
	// 	Delay_Ms(3500);
	// 	make_inputPullups();
	// 	sleepMode_setup(1);

	// 	for(;;) {
	// 		__WFE();
	// 		SystemInit();

	// 		digitalWrite(0xC3, 0);
	// 		nrf_setup(0);
	// 		send();
	// 		send();
	// 		digitalWrite(0xC3, 1);
	// 		digitalWrite(PIN_DONE, 1);
	// 		// Delay_Ms(1000);
	// 		// // make_inputPullups();
	// 		// // pinMode(0xC3, INPUT_PULLDOWN);
	// 	}
	// } 
	// else {
	// 	uint8_t readMode = digitalRead(PIN_MODE);
	// 	printf("\nMOde=%u", readMode);
	// 	nrf_setup(readMode);

	// 	// GPIO D0 Push-Pull for RX notification
	// 	RCC->APB2PCENR |= RCC_APB2Periph_GPIOD;
	// 	GPIOD->CFGLR &= ~(0xf<<(4*4));
	// 	GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*4);

	// 	for(;;) {
	// 		nrf_run(readMode);
	// 		button_run();
	// 	}
	// }
}

