#ifndef DS18B20_H
#define DS18B20_H
#include <Arduino.h>
#include <OneWire.h>
#define ONEWIRE_BUS 2				// Номер пина Arduino с подключенным датчиком

OneWire  ds(ONEWIRE_BUS);
// описание класса
class Ds18b20 {
public:
  Ds18b20(uint8_t *sensorAddress, byte length, long updateInterval = 10000);
  void setSensorAddress(uint8_t *address, byte length);
  void setUpdateInterval(long updateInterval);
  uint8_t getSensorAddress();
  long getUpdateInterval();
  void startConversion();
  void readScratchpad();
  float currentTemperature;
private:
  byte _ds18b20Data[12]; // данные последнего измерения
  byte _updateInterval;  // интервал обновления данных
  uint8_t _address[8];
};
// реализация методов
Ds18b20::Ds18b20(uint8_t *sensorAddress, byte length, long updateInterval = 10000) {
  setSensorAddress(sensorAddress, length);
  setUpdateInterval(updateInterval);
}

void Ds18b20::setSensorAddress(uint8_t *address, byte length) {
  length = length / sizeof(uint8_t);
  for (byte i = 0; i < length; i++) {
    _address[i] = address[i];
  }
}
void Ds18b20::setUpdateInterval(long updateInterval) {
  _updateInterval = updateInterval;
}
void Ds18b20::startConversion() {
  ds.reset();
  ds.select(_address);
  ds.write(0x44, 1);
}

void Ds18b20::readScratchpad(){
  ds.reset();   // Не понял, зачем present
  ds.select(_address);
  ds.write(0xBE);         // Read Scratchpad

  for ( byte i = 0; i < 9; i++) {           // we need 9 bytes
    _ds18b20Data[i] = ds.read();
    Serial.print(_ds18b20Data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" - ");
  int16_t raw = (_ds18b20Data[1] << 8) | _ds18b20Data[0];

  byte cfg = (_ds18b20Data[4] & 0x60);
  // at lower res, the low bits are undefined, so let's zero them
  if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
  else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
  else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
  //// default is 12 bit resolution, 750 ms conversion time

  currentTemperature = (float)raw / 16.0;
}
#endif /* DS18B20_H */
