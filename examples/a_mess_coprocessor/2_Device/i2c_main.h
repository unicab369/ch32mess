#include "ch32fun.h"
#include "i2c_hal.h"


#define SSD1306_128X64
#include "ssd1306_i2c.h"
#include "ssd1306.h"
#include "bomb.h"

#define INA219_ADDR 0x40

//! BH1750
#define BH17_CONT_HI1 0x10      // 1 lux resolution 120ms
#define BH17_CONT_HI2 0x11      // .5 lux resolution 120ms
#define BH17_CONT_LOW 0x13      // 4 lux resolution 16ms
#define BH17_ONCE_HI1 0x20      // 1 lux resolution 120ms
#define BH17_ONCE_HI2 0x21      // .5 lux resolution 120ms
#define BH17_ONCE_LOW 0x23      // 4 lux resolution 16ms
#define BH17_RESET 0x07
#define BH17_POWER_ON 0x01
#define BH17_ADDR 0x23

uint16_t hexToDecimal(const uint8_t* hexArray, size_t length) {
	uint16_t result = 0;
	
	for (size_t i = 0; i < length; i++) {
		result = (result << 8) | hexArray[i];
	}
	
	return result;
}

void BH17_Setup() {
	uint8_t command[1] = { BH17_CONT_LOW };
	I2CWrite(BH17_ADDR, command, sizeof(command));
}

uint16_t BH17_Read() {
	uint8_t readData[2];
	I2CRead(BH17_ADDR, readData, 2);
	return hexToDecimal(readData, sizeof(readData));
}

uint16_t INA219_Read() {
	uint8_t readData[2];
	uint16_t value16;

	uint8_t data1[] = {0x02};
	I2CWrite(INA219_ADDR, data1, 2);
	I2CRead(INA219_ADDR, readData, 2);
	
	memcpy(&value16, readData, sizeof(value16));
	uint16_t swapped = (value16>>8) | (value16<<8);
	return (swapped>>3)*4;
}

//! SHT30
uint8_t SHT_RESET_CMD[2]        = { 0x30, 0xA2 };
uint8_t SHT_HIGHREP_HOLD_CMD[2] = { 0x2C, 0x06 };
uint8_t SHT_MEDREP_HOLD_CMD[2]  = { 0x2C, 0x0D };
uint8_t SHT_LOWREP_HOLD_CMD[2]  = { 0x2C, 0x10 };
uint8_t SHT_HIGHREP_FREE_CMD[2] = { 0x24, 0x00 };
uint8_t SHT_MEDREP_FREE_CMD[2]  = { 0x24, 0x0B };
uint8_t SHT_LOWREP_FREE_CMD[2]  = { 0x24, 0x16 };
uint8_t SHT_HEATER_DISABLE[2]   = { 0x30, 0x66 };
uint8_t SHT_HEATER_ENABLE[2]    = { 0x30, 0x6D };
#define SHT3_ADDR 0x44

struct SensorData {
   uint16_t tempF, hum, lux, voltage, mA;
};

void i2c_getReadings(struct SensorData* data) {
   // uint16_t lux = 0;
	// uint16_t tempC = 0, tempF = 0, hum = 0;
	// uint16_t busVoltage = 0, mA = 0;

   //! BH1750 Reading
   int check = I2CTest(BH17_ADDR);

   if (check) { 
      data->lux = BH17_Read(); 
   }

   // char strOut[32];
   // mini_snprintf(strOut, sizeof(strOut), "\nlux=%lu", lux);
   // printf(strOut);
   // ssd1306_drawstr_sz(0, 16, strOut, 1, fontsize_8x8);

   // //! SHT31 Reading
   int check2 = I2CTest(SHT3_ADDR);

   if (check2) {
      I2CWrite(SHT3_ADDR, SHT_LOWREP_FREE_CMD, sizeof(SHT_LOWREP_FREE_CMD));
      Delay_Ms(5);
      uint8_t buff[6];
      I2CRead(SHT3_ADDR, buff, 6);

      uint16_t rawTemp = 0, rawHum = 0;
      rawTemp = (buff[0]<<8) + buff[1];
      rawHum = (buff[3]<<8) + buff[4];

      // tempC = rawTemp*175/65535 - 45;
      data->tempF = rawTemp*63/13107 - 49;
      data->hum = rawHum*100/65535;
   }

   // mini_snprintf(strOut, sizeof(strOut), "\nt=%lu, h=%lu", tempF, hum);
   // printf(strOut);

   // //! INA219 Reading
   int check3 = I2CTest(INA219_ADDR);

   if (check3) {
      data->voltage = INA219_Read();
   }

   // printf("\n\rtemp=%lu, hum=%lu, lux=%lu, v=%lu, mA=%lu", tempC, hum, lux, busVoltage, mA);
   // mini_snprintf(strOut, sizeof(strOut), "\nV=%lu, mA=%lu", busVoltage, mA);
   // printf(strOut);		

   // uint8_t allValues[] = { 0xB1, 0xCA, 0xFF, 0xFF, 0xFF, 0xFF, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x00, 0x0D, 0x0A };
   // uint16_t allValues[] = { 0xCAB1, 0xFFFF, 0xFFFF, tempF, hum, lux, busVoltage, 0x0A0D };
}

int i2c_test() {
   for(uint8_t mode=0; mode<(SSD1306_H>32?9:8); mode++) {
      // clear buffer for next mode
      ssd1306_setbuf(0);

      // switch(mode) {
      //    case 0:
      //       printf("buffer fill with binary\n\r");
      //       for(int i=0;i<sizeof(ssd1306_buffer);i++)
      //          ssd1306_buffer[i] = i;
      //       break;
         
      //    case 1:
      //       printf("pixel plots\n\r");
      //       for(int i=0;i<SSD1306_W;i++)
      //       {
      //          ssd1306_drawPixel(i, i/(SSD1306_W/SSD1306_H), 1);
      //          ssd1306_drawPixel(i, SSD1306_H-1-(i/(SSD1306_W/SSD1306_H)), 1);
      //       }
      //       break;
         
      //    case 2:
      //       {
      //          printf("Line plots\n\r");
      //          uint8_t y= 0;
      //          for(uint8_t x=0;x<SSD1306_W;x+=16)
      //          {
      //             ssd1306_drawLine(x, 0, SSD1306_W, y, 1);
      //             ssd1306_drawLine(SSD1306_W-x, SSD1306_H, 0, SSD1306_H-y, 1);
      //             y+= SSD1306_H/8;
      //          }
      //       }
      //       break;
            
      //    case 3:
      //       printf("Circles empty and filled\n\r");
      //       for(uint8_t x=0;x<SSD1306_W;x+=16)
      //          if(x<64)
      //             ssd1306_drawCircle(x, SSD1306_H/2, 15, 1);
      //          else
      //             ssd1306_fillCircle(x, SSD1306_H/2, 15, 1);
      //       break;
      //    case 4:
      //       printf("Image\n\r");
      //       ssd1306_drawImage(0, 0, bomb_i_stripped, 32, 32, 0);
      //       break;
      //    case 5:
      //       printf("Unscaled Text\n\r");
            ssd1306_drawstr(0,0, "This is a test", 1);
            ssd1306_drawstr(0,8, "of the emergency", 1);
            ssd1306_drawstr(0,16,"broadcasting", 1);
            ssd1306_drawstr(0,24,"system.",1);
            if(SSD1306_H>32)
            {
               ssd1306_drawstr(0,32, "Lorem ipsum", 1);
               ssd1306_drawstr(0,40, "dolor sit amet,", 1);
               ssd1306_drawstr(0,48,"consectetur", 1);
               ssd1306_drawstr(0,56,"adipiscing",1);
            }
            ssd1306_xorrect(SSD1306_W/2, 0, SSD1306_W/2, SSD1306_W);
      //       break;
            
      //    case 6:
      //       printf("Scaled Text 1, 2\n\r");
      //       ssd1306_drawstr_sz(0,0, "sz 8x8", 1, fontsize_8x8);
      //       ssd1306_drawstr_sz(0,16, "16x16", 1, fontsize_16x16);
      //       break;
         
      //    case 7:
      //       printf("Scaled Text 4\n\r");
      //       ssd1306_drawstr_sz(0,0, "32x32", 1, fontsize_32x32);
      //       break;
         
      //    case 8:
      //       // printf("Scaled Text 8\n\r");
      //       ssd1306_drawstr_sz(0,0, "64", 1, fontsize_64x64);
      //       break;

      //    default:
      //       break;
      // }

      ssd1306_refresh();
   
      Delay_Ms(2000);
   }
}
