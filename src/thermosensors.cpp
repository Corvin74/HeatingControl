#include "thermosensors.h"

// Если используются датчики DS18B20 создаем необходимые объекты
#ifdef ONEWIRE_BUS_GATE1
  OneWire  ds1(DS18B20_GATE1_PIN);
#endif
#ifdef ONEWIRE_BUS_GATE2
  OneWire  ds2(DS18B20_GATE2_PIN);
#endif
#ifdef ONEWIRE_BUS_GATE3
  OneWire  ds3(DS18B20_GATE3_PIN);
#endif
#ifdef ONEWIRE_BUS_GATE4
  OneWire  ds4(DS18B20_GATE4_PIN);
#endif
#ifdef ONEWIRE_BUS_GATE5
  OneWire  ds5(DS18B20_GATE5_PIN);
#endif

void ds18b20StartMeasurement(OneWire *ds, uint8_t gate_power){
  ds->reset();
  ds->write(CMD_SKIPROM, gate_power);
  ds->write(CMD_CONVERTTEMP, gate_power);
}
// http://mypractic.ru/urok-26-podklyuchenie-termodatchikov-ds18b20-k-arduino-biblioteka-onewire-tochnyj-arduino-termometr-registrator.html
int8_t ds18b20GetResultMeasurement(OneWire *ds, uint8_t gate_power){
  uint8_t dsTempBuf[9];
  float measurementResult = -200.0;
  ds->reset();
  ds->write(CMD_SKIPROM, gate_power);
  ds->write(CMD_RSCRATCHPAD, gate_power);
  ds->read_bytes(dsTempBuf, 9);
  if ( OneWire::crc8(dsTempBuf, 8) == dsTempBuf[8] ) {
    measurementResult = (float)((int)dsTempBuf[0] | (((int)dsTempBuf[1]) << 8)) * 0.0625 + 0.03125;
    return ((int)measurementResult*100);
  }
}
