#ifndef THERMOSENSORS_H
#define THERMOSENSORS_H

#include <Arduino.h>

// Признак использования датчиков DS18B20
#ifndef DS18B20
  #define DS18B20
  // #undef DS18B20
#endif
// Признак использования датчиков LM35
#ifndef LM35_ON
  #define LM35_ON
  // #undef LM35_ON
#endif
// Определяем к каким входам платы подключены датчики DS18B20
#ifndef ONEWIRE_BUS_GATE1
  #define ONEWIRE_BUS_GATE1
  // #undef ONEWIRE_BUS_GATE1
#endif
#ifndef ONEWIRE_BUS_GATE2
  #define ONEWIRE_BUS_GATE2
  // #undef ONEWIRE_BUS_GATE2
#endif
#ifndef ONEWIRE_BUS_GATE3
  #define ONEWIRE_BUS_GATE3
  // #undef ONEWIRE_BUS_GATE3
#endif
#ifndef ONEWIRE_BUS_GATE4
  #define ONEWIRE_BUS_GATE4
  // #undef ONEWIRE_BUS_GATE4
#endif
#ifndef ONEWIRE_BUS_GATE5
  // #define ONEWIRE_BUS_GATE5
  #undef ONEWIRE_BUS_GATE5
#endif

#ifdef DS18B20
  #include "onewire.h"
  #include <OneWire.h>
  // OneWire ROM commands
  #define CMD_CONVERTTEMP    0x44
  #define CMD_RSCRATCHPAD    0xBE
  #define CMD_WSCRATCHPAD    0x4E
  #define CMD_CPYSCRATCHPAD  0x48
  #define CMD_RECEEPROM      0xB8
  #define CMD_RPWRSUPPLY     0xB4
  #define CMD_SEARCHROM      0xF0
  #define CMD_READROM        0x33
  #define CMD_MATCHROM       0x55
  #define CMD_SKIPROM        0xCC
  #define CMD_ALARMSEARCH    0xEC
#endif
// Если используются датчики DS18B20 задаем номера пинов на которых они работают
#ifdef ONEWIRE_BUS_GATE1
  #define DS18B20_GATE1_PIN PC0				// Номер пина Arduino с подключенным датчиком A0
  #define DS18B20_GATE1_POWER 0				// 0 – питание от внешнего источника, 1 – "паразитное" питание.
#endif
#ifdef ONEWIRE_BUS_GATE2
  #define DS18B20_GATE2_PIN A1				// Номер пина Arduino с подключенным датчиком
  #define DS18B20_GATE2_POWER 0				// 0 – питание от внешнего источника, 1 – "паразитное" питание.
#endif
#ifdef ONEWIRE_BUS_GATE3
  #define DS18B20_GATE3_PIN A2				// Номер пина Arduino с подключенным датчиком
  #define DS18B20_GATE3_POWER 0				// 0 – питание от внешнего источника, 1 – "паразитное" питание.
#endif
#ifdef ONEWIRE_BUS_GATE4
  #define DS18B20_GATE4_PIN A3				// Номер пина Arduino с подключенным датчиком
  #define DS18B20_GATE4_POWER 0				// 0 – питание от внешнего источника, 1 – "паразитное" питание.
#endif
#ifdef ONEWIRE_BUS_GATE5
  #define DS18B20_GATE5_PIN A4				// Номер пина Arduino с подключенным датчиком
  #define DS18B20_GATE5_POWER 0				// 0 – питание от внешнего источника, 1 – "паразитное" питание.
#endif

// Определяем к каким входам платы подключены датчики LM35
#ifndef LM35_GATE1
  // #define LM35_GATE1
  #undef LM35_GATE1
#endif
#ifndef LM35_GATE2
  // #define LM35_GATE2
  #undef LM35_GATE2
#endif
#ifndef LM35_GATE3
  // #define LM35_GATE3
  #undef LM35_GATE3
#endif
#ifndef LM35_GATE4
  // #define LM35_GATE4
  #undef LM35_GATE4
#endif
#ifndef LM35_GATE5
  #define LM35_GATE5
  // #undef LM35_GATE5
#endif

// Если используются датчики LM35 задаем номера пинов на которых они работают
#ifdef LM35_GATE1
  #define LM35_GATE1_PIN A0				// Номер пина Arduino с подключенным датчиком LM35
#endif
#ifdef LM35_GATE2
  #define LM35_GATE2_PIN A1				// Номер пина Arduino с подключенным датчиком LM35
#endif
#ifdef LM35_GATE3
  #define LM35_GATE3_PIN A2				// Номер пина Arduino с подключенным датчиком LM35
#endif
#ifdef LM35_GATE4
  #define LM35_GATE4_PIN A3				// Номер пина Arduino с подключенным датчиком LM35
#endif
#ifdef LM35_GATE5
  #define LM35_GATE5_PIN A4				// Номер пина Arduino с подключенным датчиком LM35
#endif

#endif /* THERMOSENSORS_H */
