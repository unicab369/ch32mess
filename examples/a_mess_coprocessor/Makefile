all : flash

LDFLAGS+=-Wl,--print-memory-usage
TARGET:=a_useless_button
ADDITIONAL_C_FILES+= 1_Foundation/fun_button.h 1_Foundation/fun_uart.h 1_Foundation/i2c_slave.h 1_Foundation/modPWM.h 1_Foundation/modEncoder.h
ADDITIONAL_C_FILES+= 2_Device/modI2C.h 2_Device/ssd1306_draw.h 2_Device/fun_ws2812.h 
ADDITIONAL_C_FILES+= 3_Nrf/nrf24l01_low_level.c 3_Nrf/nrf24l01.c
ADDITIONAL_C_FILES+= ST7735/modTFT.h Storage/modStorage.h
ADDITIONAL_C_FILES+= 2_Device/lib_i2c.c

TARGET_MCU?=CH32V003
include ../../ch32fun/ch32fun.mk

flash : cv_flash
clean : cv_clean


