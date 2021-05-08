#ifndef DS18B20_H
#define DS18B20_H
#include <Arduino.h>
#include <OneWire.h>
#include <PubSubClient.h>
#define ONEWIRE_BUS 2				// Номер пина Arduino с подключенным датчиком
// #define ONEWIRE_BUS_THERMO_SENSOR1 A0				// Номер пина Arduino с подключенным датчиком
// #define ONEWIRE_BUS_THERMO_SENSOR2 A1				// Номер пина Arduino с подключенным датчиком
// #define ONEWIRE_BUS_THERMO_SENSOR3 A2				// Номер пина Arduino с подключенным датчиком
#define ONEWIRE_BUS_THERMO_SENSOR4 A3				// Номер пина Arduino с подключенным датчиком
#define ONEWIRE_BUS_THERMO_SENSOR5 A4				// Номер пина Arduino с подключенным датчиком

// описание класса
class Ds18b20 {
public:
  Ds18b20(uint8_t *sensorAddress, byte length, long updateInterval);
  void setSensorAddress(uint8_t *address, byte length);
  void setUpdateInterval(long updateInterval);
  uint8_t getSensorAddress(void);
  long getUpdateInterval(void);
  void startConversion(OneWire *ds);
  void readScratchpad(OneWire *ds);
  float publishSensor(OneWire *ds);
  float currentTemperature;
private:
  byte _ds18b20Data[8]; // данные последнего измерения
  byte _updateInterval;  // интервал обновления данных
  uint8_t _address[8];
  // #define ONEWIRE_BUS 2				// Номер пина Arduino с подключенным датчиком

};

#endif /* DS18B20_H */
