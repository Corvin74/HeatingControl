/*
 * main.h
 *
 *  Created on: 3 окт. 2018 г.
 *      Author: corvin
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <Arduino.h>
#include <SPI.h>
#include <Ethernet2.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include "thermosensors.h"

/**************************************************************/
/* Настройка дополнительных параметров: отладка, сеть, etc... */
/**************************************************************/

// При отладке компилировать с диагностическими сообщениями
#ifndef DEBUG
  #define DEBUG
  // #undef DEBUG
#endif

// Для инициализации IP адреса по DHCP
#ifndef DHCP
  // #define GET_DHCP
  #undef GET_DHCP
#endif
// Для инициализации статического IP адреса в зависимости от положения
#ifndef STATIC
  // #define GET_STATIC_HOME
  #undef GET_STATIC_HOME

  #define GET_STATIC_WORK
  // #undef GET_STATIC_WORK

  // #define GET_STATIC_COUNTRYHOUSE
  #undef GET_STATIC_COUNTRYHOUSE
#endif

// Название устройства при подключении к MQTT
#define DEVICE_NAME "TermostatOnArduino"

// Для DS18B20
#define TEMP1_UPDATE_TIME 10000		// Определяем периодичность проверок
#define TEMP2_UPDATE_TIME 12000		// Определяем периодичность проверок
#define TEMP3_UPDATE_TIME 14000		// Определяем периодичность проверок
/******************************************************************************/

#define LED_PIN 9                 // Пин 9 с подключенным контрольным светодиодом
#define RELAY1_PIN 7              // Пин 7 для реле 1
#define RELAY2_PIN 8              // Пин 8 для реле 2
// -------------------------------------- END - Пины Arduino ------------------------------------------------

/***************************************************************/
/*          Описание функций используемых в программе          */
/***************************************************************/
/*
 * Инициализируем работу портов
 */
void initializeVariables(void);
/*
 * Инициируем всю необходимую переферию
 */
void initializeThePeriphery(void);
/*
 * Инициируем передачу данных по шине UART на скорости 9600 бит/сек.
 */
void initSerial(void);
/*
 * Инициализируем работу с сетью через EthernetShield W5500 с помощью библиотеки
 * Ethernet2, если использовать EthernetShield W5100 надо применять библиотеку
 * Ethernet
 */
void initNetwork(void);
/*
 * Колбэк функция для работы с MQTT-брокером
 */
void callback(char* topic, byte* payload, unsigned int length);
/*
 * Функция для подключения к MQTT-брокеру
 */
boolean reconnect(void);
/*
 * Среднее значение температуры в доме
 */
float calcAvarage(float sensor1, float sensor2);
/*
 * Мигаем светодиодом
 * ledPin - пин к которому подключен светодиод
 * time - время в мс
 * quantity - количество вспышек
 */
void flashLed(uint8_t ledPin, uint16_t time, uint8_t quantity);
/*
 * Проверяем необходимость включения котла по средней температуре
 */
void checkHeatingAVG(void);
/*
 * Проверяем необходимость включения котла по температуре первого датчика
 */
void checkHeatingSensor1(void);
/*
 * Включаем котл если он выключен
 */
void heatingON(void);
/*
 * Выключаем котл если он включен
 */
void heatingOFF(void);

/****************************************************************/
/*         Описание переменных используемых в программе         */
/****************************************************************/
// Признак первого запуска
uint8_t coldStart;
// Адреса датчиков используемых в проекте
// uint8_t ds18b20Sensor1[8] = { 0x28, 0xFF, 0x64, 0xAA, 0xB2, 0x16, 0x05, 0x7E };
// uint8_t ds18b20Sensor2[8] = { 0x28, 0xFF, 0x2F, 0x99, 0x50, 0x17, 0x04, 0x35 };
// uint8_t ds18b20Sensor3[8] = { 0x28, 0xFF, 0x2F, 0x99, 0x50, 0x17, 0x04, 0x35 };
// uint8_t ds18b20SensorX[8] = { 0x28, 0x, 0x, 0x, 0x, 0x, 0x, 0x };
// uint8_t ds18b20Sensor1[8] = { 0x28, 0xFF, 0x98, 0x29, 0x03, 0x15, 0x03, 0x37 }; // Датчик в корридоре (герметичный) 28 FF BC B4 B2 16 5 33
uint8_t ds18b20Sensor1[8] = { 0x28, 0xFF, 0xBC, 0xB4, 0xB2, 0x16, 0x05, 0x33 }; // Датчик в корридоре (герметичный) 28 FF BC B4 B2 16 5 33
uint8_t ds18b20Sensor2[8] = { 0x28, 0x4B, 0x18, 0x43, 0x98, 0x01, 0x02, 0xCC }; // Датчик в погребе
uint8_t ds18b20Sensor3[8] = { 0x28, 0xFF, 0xAF, 0xA9, 0xB2, 0x16, 0x05, 0xA4 }; // Датчик в котельной на плате

uint8_t ds18b20Sensor4[8] = { 0x28, 0x88, 0x21, 0x43, 0x98, 0x01, 0x02, 0x7D };
uint8_t ds18b20Sensor5[8] = { 0x28, 0x15, 0x00, 0x43, 0x98, 0x0F, 0x00, 0xD3 };
uint8_t ds18b20Sensor6[8] = { 0x28, 0x33, 0x25, 0x43, 0x98, 0x24, 0x00, 0x1F }; // Датчик в котельной у пола
uint8_t ds18b20Sensor7[8] = { 0x28, 0xB3, 0x0B, 0x43, 0x98, 0x01, 0x02, 0x2D };
uint8_t ds18b20Sensor8[8] = { 0x28, 0xFF, 0x2F, 0x99, 0x50, 0x17, 0x04, 0x35 };
uint8_t ds18b20Sensor9[8] = { 0x28, 0x86, 0x01, 0x43, 0x98, 0x0E, 0x00, 0x32 };
// Объекты датчиков температуры
// Ds18b20 mySensor1(ds18b20Sensor1, sizeof(ds18b20Sensor1), TEMP1_UPDATE_TIME);
// Ds18b20 mySensor2(ds18b20Sensor2, sizeof(ds18b20Sensor2), TEMP2_UPDATE_TIME);
// Ds18b20 mySensor3(ds18b20Sensor3, sizeof(ds18b20Sensor3), TEMP3_UPDATE_TIME);
// Текущие данные с датчиков температуры
float dsSensor1;
float dsSensor2;
float dsSensor3;
float TSensor1;
float TSensor2;
float TSensor3;
float TSensor4;
float TSensor5;
// Переменные для хранения времени последнего обновления данных
long previousUpdateTime1;
long previousUpdateTime2;
long previousUpdateTime3;

struct HeatingControl {
	float curentAverageTemperature;		// Текущая средняя температура в доме
	uint8_t currentState;	// Состояние котла
	float targetTemperature;	// Текущая целевая температура
	float hysteresis;	// Допустимый гистерезис целевой температуры
	uint8_t heatingAuto;		// Авторежим
  uint8_t heatingChanged;
};

HeatingControl heatingControl;
uint8_t saveSettingsToEEPROM(HeatingControl* heatingStruct);
uint8_t restoreSettingsFromEEPROM(HeatingControl* heatingStruct);

struct MessageMQTT {
	String topic;		// Текущая средняя температура в доме
	String payload;	// Состояние котла
};

MessageMQTT messageMQTT;

// MAC-адрес для модуля W5500
byte mac[] = { 0x00, 0xAA, 0xBB, 0xCC, 0xDA, 0x01 };

#ifdef GET_STATIC_HOME
  // Статический IP-адрес для модуля W5500
  IPAddress ip(172, 20, 10, 163);
  // Адрес DNS сервера для модуля W5500
  IPAddress ip_dns(8, 8, 8, 8);
  // Адрес шлюза для модуля W5500
  IPAddress ip_gateway(172, 20, 10, 1);
  // Адрес подсети для модуля W5500
  IPAddress ip_subnet(255, 255, 255, 0);
#endif

#ifdef GET_STATIC_WORK
  // Статический IP-адрес для модуля W5500
  IPAddress ip(172, 16, 6, 201);
  // Адрес DNS сервера для модуля W5500
  IPAddress ip_dns(172, 16, 6, 1);
  // Адрес шлюза для модуля W5500
  IPAddress ip_gateway(172, 16, 6, 1);
  // Адрес подсети для модуля W5500
  IPAddress ip_subnet(255, 255, 255, 0);
#endif

#ifdef GET_STATIC_COUNTRYHOUSE
  // Статический IP-адрес для модуля W5500
  IPAddress ip(172, 20, 20, 202);
  // Адрес DNS сервера для модуля W5500
  IPAddress ip_dns(8, 8, 8, 8);
  // Адрес шлюза для модуля W5500
  IPAddress ip_gateway(172, 20, 20, 1);
  // Адрес подсети для модуля W5500
  IPAddress ip_subnet(255, 255, 254, 0);
#endif

// IP-адресс MQTT брокера
// IPAddress server(172, 20, 10, 102);
IPAddress server(172, 20, 20, 125);
// IPAddress server(172, 16, 6, 40);

// Уставновить Логин и Пароль для подключения к MQTT брокеру
const char* mqtt_username = "corvin";
const char* mqtt_password = "eTx1243";

EthernetClient ethClient;
PubSubClient client(server, 1883, callback, ethClient);
#endif /* MAIN_H_ */
