#include "ch32fun.h"
#include "i2c_slave.h"
#include <stdio.h>

// The I2C slave library uses a one byte address so you can extend the size of this array up to 256 registers
// note that the register set is modified by interrupts, to prevent the compiler from accidently optimizing stuff
// away make sure to declare the register array volatile

volatile uint8_t i2c_registers[32] = {0x00};

void onWrite(uint8_t reg, uint8_t length) {
    printf("IM WRITEEN TO\n\r");
    int read = funDigitalRead(PA1);
    funDigitalWrite(PA1, !read);
}

void onRead(uint8_t reg) {
    printf("IM READEN FROM\n\r");
    int read = funDigitalRead(PA2);
    funDigitalWrite(PA2, !read);
}

int main() {
    SystemInit();
    funGpioInitAll();

    i2c_registers[0] = 0xAA;
    i2c_registers[1] = 0xBB;
    i2c_registers[2] = 0xCC;

    // Initialize I2C slave
    funPinMode(PC1, GPIO_CFGLR_OUT_10Mhz_AF_OD); // SDA
    funPinMode(PC2, GPIO_CFGLR_OUT_10Mhz_AF_OD); // SCL
    SetupI2CSlave(0x77, i2c_registers, sizeof(i2c_registers), onWrite, onRead, false);

    // Initialize LED
    funPinMode(PA2, GPIO_CFGLR_OUT_10Mhz_PP); // LED
    funPinMode(PA1, GPIO_CFGLR_OUT_10Mhz_PP); // LED

    while (1) {} // Do not let main exit, you can do other things here
}
