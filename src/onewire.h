/*
 * onewire.h
 *
 *  Created on: 24 мая 2021 г.
 *      Author: corvin
 */

#ifndef ONEWIRE_H_
#define ONEWIRE_H_
#define OWPORT PORTC
#define OWPORTDIR DDRC
#define OWPIN PINC
#include <Arduino.h>

struct owSettings {
	uint8_t owPin;
	uint8_t owPort;
};

int8_t checkPresence(uint8_t onewirePin);

uint8_t readBit(uint8_t onewirePin);
void writeBit(uint8_t data);

uint8_t readByte(uint8_t onewirePin);
void writeByte(uint8_t data);

#endif /* ONEWIRE_H_ */
