all : flash

LDFLAGS+=-Wl,--print-memory-usage
TARGET:=a_useless_button
ADDITIONAL_C_FILES+=1_Foundation/fun_button.h 1_Foundation/fun_uart.h 1_Foundation/i2c_slave.h 1_Foundation/modPWM.h 1_Foundation/modEncoder.h 2_Device/i2c_main.h  2_Device/fun_ws2812.h 3_Nrf/nrf24l01_low_level.c 3_Nrf/nrf24l01.c

TARGET_MCU?=CH32V003
include ../../ch32fun/ch32fun.mk

flash : cv_flash
clean : cv_clean


