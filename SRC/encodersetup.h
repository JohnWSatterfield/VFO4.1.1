/*
 * code by JSatterfield - KI5IDZ
 * inspired by: Arduino Rotary Encoder Tutorial by Dejan Nedelkovski, 
 * https://www.HowToMechatronics.com
 *
*/

#include "config.h"           // Hardware configuration (pin assignments & display type)
//#define __no_inline_not_in_flash_func(func) __attribute__((noinline,section(".ram_code"))) func

volatile long  quad = 0;
volatile long previous_data;
volatile uint8_t f_echange;
volatile bool locked = HIGH; 


void enc_read (void) {
 if (locked) {
  long current_data = digitalRead(DT) << 1 | digitalRead(CLK);
  if( current_data == previous_data ) {f_echange=0;return;}
  if( bitRead(current_data, 0) == bitRead(previous_data, 1) )
    quad -= 1;
  else
    quad += 1;
  previous_data = current_data;
 }
}



//void __no_inline_not_in_flash_func( enc_read) (void){code}

#if MC_TYPE == R2040
  void __no_inline_not_in_flash_func ( enc_read) (void) ;
  //void enc_read();
#else
  void IRAM_ATTR enc_read();
#endif



void setupencoder() {
  pinMode(DT,  INPUT_PULLUP);              // assign pin to DT
  pinMode(CLK, INPUT_PULLUP);              // assign pin to CLK
  attachInterrupt(digitalPinToInterrupt(DT),  enc_read, CHANGE);  // enable interrupts for rotary encoder pins
  attachInterrupt(digitalPinToInterrupt(CLK), enc_read, CHANGE);  // enable interrupts for rotary encoder pins
  previous_data = digitalRead(DT) << 1 | digitalRead(CLK);
}



