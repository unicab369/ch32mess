#include "ch32fun.h"
#include "modTimer.h"
#include "fun_gpio.h"
#include <stdio.h>

#define TICK_DEBOUNCE_DUR     20
#define TICK_CLICK_DUR        160
#define TICK_LONG_PRESS_DUR   4000

enum {
   BTN_DOWN = 0,
   BTN_UP,
   BTN_DOWN2,
   BUTTON_IDLE
};

static uint32_t debounce_time = 0;
static uint32_t release_time = 0;
static uint32_t press_time = 0;
static uint8_t button_pin = 0xFF;
static uint8_t btn_state = BUTTON_IDLE;

void button_onSingleClick();
void button_onDoubleClick();

// void button_onSingleClick() {
// 	printf("\nI'M USELESS.");
// }

// void button_onDoubleClick() {
// 	printf("\nI'M USELESS TWICE.");
// }

void button_onLongPress(uint32_t duration) {
   printf("pressing duration: %ld\n\r", duration);
}

void _reset_timers(uint8_t newState) {
   btn_state = newState;
   debounce_time = millis();
   release_time = millis();
}

void button_setup(uint8_t pin) {
   if (pin == 0xFF) return; 
   button_pin = pin;

	pinMode(pin, INPUT_PULLUP);
   systick_init();
   _reset_timers(BUTTON_IDLE);
}

void button_run() {
   if (button_pin == 0xFF) return;
   
   uint32_t now = millis();
   uint8_t read = digitalRead(button_pin);

   // Debounce check
   if (now - debounce_time < TICK_DEBOUNCE_DUR) return;
   debounce_time = now;

   switch (btn_state) {
      case BUTTON_IDLE:
         if (read == 0) {
            press_time = now;
            _reset_timers(BTN_DOWN);      // First Press  
         }
         break;

      case BTN_DOWN:
         if (read == 1) {
            _reset_timers(BTN_UP);        // First Release

         } else {
            // Long press detection
            uint32_t press_duration = now - press_time;
            if (press_duration > TICK_LONG_PRESS_DUR) {
               button_onLongPress(press_duration);
            }
         }
         break;

      case BTN_UP: {
         uint32_t release_duration = now - release_time;

         if (read == 0 && release_duration < TICK_CLICK_DUR) {
            // Second Press in less than TICK_CLICK_DUR
            _reset_timers(BTN_DOWN2);

         } else if (release_duration > TICK_CLICK_DUR) {
            button_onSingleClick();
            _reset_timers(BUTTON_IDLE);
         }
         break;
      }

      case BTN_DOWN2:
         // Second release
         if (read == 1) {
            button_onDoubleClick();
            _reset_timers(BUTTON_IDLE);
         }
         break;
   }
}