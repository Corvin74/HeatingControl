#ifndef DS18B20_H
#define DS18B20_H
#include <Arduino.h>
#include <OneWire.h>
#define ONEWIRE_BUS 2				// Номер пина Arduino с подключенным датчиком

// описание класса
class Ds18b20 {
public:
  Ds18b20(uint8_t *sensorAddress, byte length, long updateInterval);
  void setSensorAddress(uint8_t *address, byte length);
  void setUpdateInterval(long updateInterval);
  uint8_t getSensorAddress();
  long getUpdateInterval();
  void startConversion();
  void readScratchpad();
  float currentTemperature;
private:
  byte _ds18b20Data[8]; // данные последнего измерения
  byte _updateInterval;  // интервал обновления данных
  uint8_t _address[8];
};

#endif /* DS18B20_H */
