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
	// struct SensorData* receivedData = (struct SensorData*) data;

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
#include "2_Device/fun_ws2812.h"
#include "2_Device/ssd1306_draw.h"


volatile uint8_t i2c_registers[32] = {0xaa};

void onWrite(uint8_t reg, uint8_t length) {
	printf("IM HERE.\n\r");
    funDigitalWrite(PA2, i2c_registers[0] & 1);
}

void onRead(uint8_t reg) {
	printf("IM HERE.\n\r");
	digitalWrite(0xC0, !digitalRead(0xD0));
}


// #include "ST7735/st7735_demo.h"
#include "ST7735/modTFT.h"
#include "Storage/modStorage.h"

char str_output[20];

void encoder_onChanged(M_Encoder *model) {
	sprintf(str_output, "Pos %ld delta %ld",
			(int32_t)model->count - model->initial_count, 
			(int32_t)model->count - model->last_count);
	ssd1306_print_str_at(str_output, 0, 0);
	printf(str_output); printf("\n\r");
}

void button_onChanged(int btn, uint32_t duration) {
	if (btn == BTN_SINGLECLICK) {
		sprintf(str_output, "BTN 1x CLICK");
	} else if (btn == BTN_DOUBLECLICK) {
		sprintf(str_output, "BTN 2x CLICK");
	} else if (btn == BTN_LONGPRESS) {
		sprintf(str_output, "BTN LONGPRESS %ld", duration);
	}

	ssd1306_print_str_at(str_output, 0, 0);
	printf(str_output); printf("\n\r");
}

void i2c_scan_callback(const uint8_t addr) {
	if (addr == 0x00 || addr == 0x7F) return; // Skip reserved addresses
	
	static int line = 1;
	sprintf(str_output, "I2C: 0x%02X", addr);
	ssd1306_print_str_at(str_output, line++, 0);
	printf(str_output); printf("\n\r");
}

uint8_t decimal_to_bcd(uint8_t val) { return ((val/10)*16) + (val%10); }

i2c_err_t ret;
uint8_t buff[8];

void test_htu21() {
	uint8_t userRegisterData = 0;

	ret = i2c_read_reg(&dev_htu21, 0xE7, userRegisterData, 1);		// Read User register
	printf("Error0: %d\n", ret);
	printf("HTU21 User Register: %02X\n", userRegisterData);

	userRegisterData &= 0x7E;		// clear resolution bits with 0
	userRegisterData |= 0x80;		// add new resolution bits

	ret = i2c_write_reg(&dev_htu21, 0xE6, userRegisterData, 1);		// Write User register
	printf("Error1: %d\n", ret);

	ret = i2c_read_reg(&dev_htu21, 0xE7, buff, 1);		// Read User register
	printf("Error2: %d\n", ret);
	printf("HTU21 New User Register: %02X\n", userRegisterData);

	buff[0] = 0xE5;									
	ret = i2c_write_raw(&dev_htu21, buff, 1);		// Request Read Humidity
	printf("Error3: %d\n", ret);

	Delay_Ms(30);									// Wait for measurement to complete
	ret = i2c_read_raw(&dev_htu21, buff, 3);		// Read Humidity
	printf("Error4: %d\n", ret);
	printf("HTU21 Read: %02X %02X %02X\n", buff[0], buff[1], buff[2]);
}

void test_aht21() {
	ret = i2c_read_reg(&dev_aht21, 0x71, buff, 1);
	buff[0] = buff[0] & 0x18;
	printf("Error0: %d\n", ret);
	printf("AHT21 Read reg 0x71: %02X\n", buff[0]);		// expect 0x18

	buff[0] = 0xAC;
	buff[1] = 0X33;
	buff[2] = 0x00;
	ret = i2c_write_raw(&dev_aht21, buff, 3);
	printf("Error1: %d\n", ret);

	Delay_Ms(100);									// Wait for measurement to complete
	ret = i2c_read_raw(&dev_aht21, buff, 6);		// Read sensor
	printf("Error2: %d\n", ret);

	// calculate humidity
	uint32_t hum = (buff[1] << 12) | (buff[2] << 4) | (buff[3] >> 4);
	hum = (hum * 100) / 0x100000;
	printf("Humidity: %lu\n", hum);

	// calculate temperature
	uint32_t temp = ((buff[3] & 0xF) << 16) | (buff[4] << 8) | (buff[5]);
	temp = (temp * 200) / 0x100000;
	printf("Temperature: %lu\n", temp);
}

void test_bmp280() {
	struct {
		int16_t dig_T1;
		int16_t dig_T2;
		int16_t dig_T3;
		// Pressure calibration would be added similarly
	} bmp280_calib;

	ret = i2c_read_reg(&dev_bmp280, 0xD0, buff, 1);
	printf("Error0: %d\n", ret);
	printf("BMP280 Read reg 0xD0: %02X\n", buff[0]);		// expect 0x58

	ret = i2c_read_reg(&dev_bmp280, 0x88, buff, 2);
	bmp280_calib.dig_T1 =  (int16_t)(buff[1] << 8) | buff[0];

	ret = i2c_read_reg(&dev_bmp280, 0x8A, buff, 2);
	bmp280_calib.dig_T2 =  (int16_t)(buff[1] << 8) | buff[0];

	ret = i2c_read_reg(&dev_bmp280, 0x8C, buff, 2);
	bmp280_calib.dig_T3 =  (int16_t)(buff[1] << 8) | buff[0];

	printf("dig_T1: %lu\n", bmp280_calib.dig_T1);
	printf("dig_T2: %lu\n", bmp280_calib.dig_T2);
	printf("dig_T3: %ld\n", bmp280_calib.dig_T3);

	ret = i2c_write_reg(&dev_bmp280, 0xF4, (uint8_t[]){0x5D}, 1);
	printf("Error1: %d\n", ret);
	Delay_Ms(100);

	ret = i2c_read_reg(&dev_bmp280, 0xF7, buff, 6);			// read sensors

	uint32_t raw_pressure = (uint32_t)(buff[0]*4096 + buff[1]*16 + (buff[2]/16));
	uint32_t raw_temp = (uint32_t)(buff[3]*4096 + buff[4]*16 + (buff[5]/16));

	uint32_t var1 = (raw_temp/16384 - bmp280_calib.dig_T1/1024) * bmp280_calib.dig_T2;
	uint32_t var2 = ((raw_temp/131072 - bmp280_calib.dig_T1/8192) * (raw_temp/131072 - bmp280_calib.dig_T1/8192)) * bmp280_calib.dig_T3;
	uint32_t t_fine = var1 + var2;		// t_fine in 0.01 degrees Celsius

	uint32_t temp = t_fine/5120;
	printf("Temperature: %lu\n", temp);

}

int main() {
	static const char message[] = "Hello World!\r\n";
	uint32_t counter = 0;
	uint32_t ledc_time = 0;
	uint32_t sec_time = 0;
	uint32_t time_ref = 0;

	M_Encoder encoder_a = {0, 0, 0};
	M_Button button_a = {0xC0, BUTTON_IDLE, 0, 0, 0, 0, 0, 0};

	SystemInit();
	systick_init();			//! required for millis()
	Delay_Ms(100);
	
	// used PC0
	button_setup(&button_a);

	// TIM2 Ch1, Ch2 : uses PD3, PD4.
	modEncoder_setup(&encoder_a);

	// I2CInit(0xC1, 0xC2, 100000);

	// return 0;

	// I2C1: uses PC1 & PC2
	if(i2c_init(&dev_aht21) != I2C_OK)
		printf("Failed to init I2C\n");
	else
		ssd1306_setup();
		// modI2C_task();

	// sprintf(str_output, "Hello Bee!");
	// ssd1306_print_str_at(str_output, 0, 0);

	// modST7735_setup();
	// pinMode(0xD0, OUTPUT);
	// modJoystick_setup();		// ADC Ch0, Ch1 | DMA1 Ch1
	// ws2812_setup();				// DMA1 Ch3

	// modPWM_setup();				// TIM2 Ch3

	// led_setup();
	// uart_setup();				// PD5
	// dma_uart_setup();			// DMA1 Ch4

    // SPI_init2();

	// Scan the I2C Bus, prints any devices that respond
	printf("----Scanning I2C Bus for Devices---\n");
	i2c_scan(i2c_scan_callback);
	printf("----Done Scanning----\n\n");

	// test_htu21();
	// test_aht21();
	test_bmp280();

	return 1;


	
    uint8_t config[] = {
        0x80,           // ENABLE
        0x0F,           // Power ON, Proximity enable, ALS enable, Wait enable
        0x90,           // CONFIG2
        0x01,           // Proximity gain control (4x)
        0x8F, 0x20,     // Proximity pulse count (8 pulses)
        0x8E, 0x87      // Proximity pulse length (16us)
    };
	// i2c_write_raw(&dev_apds9960, config, sizeof(config));

	for(;;) {			
		uint32_t now = millis();

		button_run(&button_a, button_onChanged);
		modEncoder_task(now, &encoder_a, encoder_onChanged);

		if (now - sec_time > 2000) {
			sec_time = now;

			sprintf(str_output, "counter %lu", counter++);
			ssd1306_print_str_at(str_output, 7, 0);

			// check_sensors();


			if (i2c_ping(dev_htu21.addr) == I2C_OK) {
				printf("HTU21\n");
				buff[0] = 0xE3;
				ret = i2c_read_reg(&dev_htu21, 0xE3, buff, 3);
				// err = i2c_write_raw(&dev_htu21, buff, 1);
				printf(("Error0: %d\n"), ret);

				// Delay_Ms(50);
				// err = i2c_read_raw(&dev_htu21, buff, 3);
				// printf(("Error1: %d\n"), err);

				printf("HTU21 Read: %02X %02X %02X\n", buff[0], buff[1], buff[2]);
			}

			// AHT21 Addr: 0x38
			if (i2c_ping(dev_aht21.addr) == I2C_OK) {
				buff[0] = 0xAC;
				buff[1] = 0X33;
				buff[2] = 0x00;
				// I2CWrite(dev_aht21.addr, buff, 3);
				ret = i2c_write_raw(&dev_aht21, buff, 3);
				printf(("Error1: %d\n"), ret);

				Delay_Ms(100);
				I2CRead(dev_aht21.addr, buff, 6);
				ret = i2c_read_raw(&dev_aht21, buff, 6);
				printf(("Error2: %d\n"), ret);

				//CALCULATING HUMIDITY
				uint32_t humidity;
				humidity = (buff[1] << 12) | (buff[2] << 4) | (buff[3] >> 4);
				humidity = (humidity * 100);
				humidity = humidity / 0x100000;
				printf("Humidity: %lu\n\n", humidity);
			}

			// SHT3x Addr: 0x44
			if (i2c_ping(dev_sht3x.addr) == I2C_OK) {
				uint16_t tempF, hum;
				
				Delay_Ms(1);
				SHT3x_getReading(&tempF, &hum);
				sprintf(str_output, "temp %lu, hum %lu", tempF, hum);
				// ssd1306_print_str_at(str_output, 2, 0);
				// printf(str_output); printf("\n\r");

				Delay_Ms(100);
			}

			// BH1750 Addr: 0x23
			if (i2c_ping(dev_bh17.addr) == I2C_OK) {
				uint8_t readData[2];

				// I2CWrite(BH17_ADDR, BH17_CONT_HI1, 1);
				// Delay_Ms(200); // Wait for measurement to complete
				// I2CRead(BH17_ADDR, readData, 2);
				// printf("BH17 Read: %02X %02X\n", readData[0], readData[1]);
				
				// uint16_t lux = BH17_Read(); 
				// sprintf(str_output, "lux %lu", lux);
				// ssd1306_print_str_at(str_output, 1, 0);
				// printf(str_output); printf("\n\r");
			}

			// DS3231 Addr: 0x68
			if (i2c_ping(dev_ds3231.addr) == I2C_OK) {
				uint8_t time[3] = {0};  // Time in Sec, Min, Hrs (Hex not Decimal)

				// Read time from DS3231
				i2c_read_reg(&dev_ds3231, 0x00, time, sizeof(time));
				sprintf(str_output, "Time %02X:%02X:%02X", time[2], time[1], time[0]);
				ssd1306_print_str_at(str_output, 3, 0);
				printf(str_output); printf("\n\r");
			}

			// APDS9960 Addr: 0x39
			if (i2c_ping(dev_apds9960.addr) == I2C_OK) {
				uint8_t proximity = 0;
				i2c_read_reg(&dev_apds9960, 0x9C, &proximity, 1);
				sprintf(str_output, "Prox %02X", proximity);
				ssd1306_print_str_at(str_output, 5, 0);
				printf(str_output); printf("\n\r");

				uint8_t raw[8] = { 0 };
				i2c_read_reg(&dev_apds9960, 0x94, raw, sizeof(raw));
				uint16_t red = (raw[1] << 8) | raw[0];
				uint16_t green = (raw[3] << 8) | raw[2	];
				uint16_t blue = (raw[5] << 8) | raw[4];
				uint16_t clear = (raw[7] << 8) | raw[6];				
				sprintf(str_output, "Red %lu, Green %lu, Blue %lu, Clear %lu", red, green, blue, clear);
				ssd1306_print_str_at(str_output, 6, 0);
				printf(str_output); printf("\n\r");
			}

			// modJoystick_task();
			// dma_uart_tx(message, sizeof(message) - 1);

			// uint32_t runtime_i2c = get_runTime(modI2C_task);
			// sprintf(str_output, "I2C runtime: %lu us", runtime_i2c);
			// ssd1306_print_str_at(str_output, 0, 0);

			// print_runtime("ST7735", st7735_task);
			// print_runtime("ST7735b", tft_test);

			// storage_test();
		}

	// 	if (now - ledc_time > 12) {
	// 		ledc_time = now;
	// 		modPWM_task();
	// 		ws2812_task();
	// 	}
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

