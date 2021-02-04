#ifndef DS18B20_H
#define DS18B20_H
#include <Arduino.h>
// описание класса
class Ds18b20 {
public:
 Ds18b20(uint8_t sensorAddress, long updateInterval = 10000);
 void setSensorAddress(uint8_t address);
 void setUpdateInterval(long updateInterval);
 uint8_t getSensorAddress();
 long getUpdateInterval();
private:
 byte ds18b20Data[12]; // данные последнего измерения
 byte _bright; // переменная яркости
};
// реализация методов
Color::Color(byte color = 5, byte bright = 30) { // конструктор
 _color = color;   // запоминаем
 _bright = bright;
}
void Color::setColor(byte color) {_color = color;}
void Color::setBright(byte bright) {_bright = bright;}
byte Color::getColor() {return _color;}
byte Color::getBright() {return _bright;}
#endif /* DS18B20_H */
