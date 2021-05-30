/*
 * onewire.cpp
 *
 *  Created on: 24 мая 2021 г.
 *      Author: corvin
 */
#include "onewire.h"

int8_t checkPresence(uint8_t onewirePin){
  uint8_t saveStack = SREG; // Save stack value
  cli(); // Disable interrupt
  int8_t onewireDevice;
  OWPORTDIR |= (1<<onewirePin);
  _delay_us(480);
  OWPORTDIR &= ~(1<<onewirePin);
  _delay_us(70);
  if ((OWPIN & (1<<onewirePin)) == 0) {
    onewireDevice = 1;
  } else {
    onewireDevice = 0;
  }
  SREG = saveStack;
  _delay_us(410);
  return onewireDevice;
}

uint8_t readBit(uint8_t onewirePin){
  uint8_t saveStack = SREG; // Save stack value
  cli(); // Disable interrupt
  int8_t receivedBit;
  OWPORTDIR |= (1<<onewirePin);
  _delay_us(2);
  OWPORTDIR &= ~(1<<onewirePin);
  _delay_us(13);
  receivedBit = (OWPIN & (1<<onewirePin))>>onewirePin;
  _delay_us(45);
  SREG = saveStack;
  return receivedBit;
}
