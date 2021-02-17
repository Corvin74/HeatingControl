#include "ds18b20.h"

OneWire  ds(ONEWIRE_BUS);

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
  ds.reset();
  ds.select(_address);
  ds.write(0xBE);         // Read Scratchpad

  for ( byte i = 0; i < 9; i++) {           // we need 9 bytes
    _ds18b20Data[i] = ds.read();
    #ifdef DEBUG
      Serial.print(_ds18b20Data[i], HEX);
      Serial.print(" ");
    #endif
  }
  #ifdef DEBUG
    Serial.print(" - ");
  #endif
  if (63488 == ((_ds18b20Data[1] << 8)^0b11111000)) {
    /* При отрицательном значении ( S=1 ) сначала необходимо перевести
     * дополнительный код в прямой. Для этого надо инвертировать каждый
     * разряд двоичного кода и прибавить 1. А затем перевести в десятичный
     * и умножить на 0,0625 °C.
     */
    int16_t raw = (_ds18b20Data[1] << 8) | _ds18b20Data[0];
    raw = (raw ^ 0xffff) + 1;
  } else {
    int16_t raw = (_ds18b20Data[1] << 8) | _ds18b20Data[0];

    byte cfg = (_ds18b20Data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time

    Ds18b20::currentTemperature = (float)raw / 16.0;
  }
}

float Ds18b20::publishSensor(void) {
  Ds18b20::readScratchpad();
  Ds18b20::startConversion();
  #ifdef DEBUG
    Serial.print(F("Sensor1 = "));
    Serial.print( Ds18b20::currentTemperature );
    Serial.println(" °C");
  #endif
  return Ds18b20::currentTemperature;
  // if (!client->publish("/countryhouse/ds18b20_1", dataTempChar)) {
  //   Serial.println(F("Publish sensor1 temperature failed"));
  // }
  // client.publish("/countryhouse/ds18b20_1", dataTempChar);
}
