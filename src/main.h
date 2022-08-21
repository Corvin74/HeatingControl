/*
 * main.h
 *
 *  Created on: 3 окт. 2018 г.
 *      Author: corvin
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <Arduino.h>

/**************************************************************/
/*    Настройка основных параметров: сеть, датчики, etc...    */
/**************************************************************/

/*##############################*/
/*        Настройки сети        */
/*##############################*/
// Для использования DHCP
#ifndef DHCP
  #define GET_DHCP
  // #undef GET_DHCP
#endif
// Для использования статических настроек IP адреса
#ifndef STATIC
  // #define GET_STATIC_HOME
  #undef GET_STATIC_HOME

  // #define GET_STATIC_WORK
  #undef GET_STATIC_WORK

  // #define GET_STATIC_COUNTRYHOUSE
  #undef GET_STATIC_COUNTRYHOUSE
#endif
// Название устройства при подключении к MQTT брокеру
#define DEVICE_NAME "TermostatOnAtmega328"

/*###########################################*/
/* Настройки радиомодуля на 433MHz WL102-341 */
/*###########################################*/
// Для использования модуля включить дефайн
#ifndef WL102_ON
  // #define WL102_ON
  #undef WL102_ON
#endif
#ifdef WL102_ON
  #include <VirtualWire.h>
  #define WL102_TX_PIN PB1
#endif

/*###################################*/
/* Настройки периода опроса датчиков */
/*###################################*/
// Температура подачи
#define SENS1_UPTIME 5000
#define UPDATE_TIME1 5000
// Температура обратки
#define SENS2_UPTIME 8080
#define UPDATE_TIME2 8080
// Температура в погребе
#define SENS3_UPTIME 61000
#define UPDATE_TIME3 90000
// Температура в холле
#define SENS4_UPTIME 62100
#define UPDATE_TIME4 90000
// Температура на кухне
#define SENS5_UPTIME 63200
#define UPDATE_TIME5 90000

/*#############################*/
/*   Настройки термодатчиков   */
/*#############################*/

#define SENSOR1_DIGITAL   A0
// #define SENSOR1_ANALOG   A0
#define SENSOR2_DIGITAL   A1
// #define SENSOR2_ANALOG   A1
#define SENSOR3_DIGITAL    A2
// #define SENSOR3_ANALOG    A2
// #define SENSOR4_DIGITAL    A3
// #define SENSOR4_ANALOG    A3
// #define SENSOR5_DIGITAL    A4
// #define SENSOR5_ANALOG   A4

#ifdef SENSOR1_DIGITAL
  #include <OneWire.h>
  OneWire sensor1(SENSOR1_DIGITAL);
#endif
#ifdef SENSOR2_DIGITAL
  #include <OneWire.h>
  OneWire sensor2(SENSOR2_DIGITAL);
#endif
#ifdef SENSOR3_DIGITAL
  #include <OneWire.h>
  OneWire sensor3(SENSOR3_DIGITAL);
#endif
#ifdef SENSOR4_DIGITAL
  #include <OneWire.h>
  OneWire sensor4(SENSOR4_DIGITAL);
#endif
#ifdef SENSOR4_DIGITAL
  #include <OneWire.h>
  OneWire sensor5(SENSOR5_DIGITAL);
#endif

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

// uint8_t ds18b20Sensor[8] = { 0x28, 0xFF, 0xBC, 0xB4, 0xB2, 0x16, 0x05, 0x33 };

/*##############################*/
/*        Настройки реле        */
/*##############################*/
#define RELAY1_PIN 3              // Пин 3 для реле 1
#define RELAY2_PIN 4              // Пин 4 для реле 2
#define RELAY3_PIN 5              // Пин 5 для реле 3
#define RELAY4_PIN 6              // Пин 6 для реле 4
#define RELAY5_PIN 7              // Пин 7 для реле 5


/***************************************************************/
/* Настройка дополнительных параметров: отладка, экран, etc... */
/***************************************************************/

// При отладке компилировать с диагностическими сообщениями
#ifndef DEBUG
  #define DEBUG
  // #undef DEBUG
#endif
#ifdef WL102_ON
  #ifndef DEBUGRF
    #define DEBUGRF
    // #undef DEBUGRF
  #endif
#endif
#define LED_PIN 13                // Пин 13(PB5) с подключенным контрольным светодиодом
// Признак подключения LCD экрана
#ifndef LCD_ON
  // #define LCD_ON
  #undef LCD_ON
#endif

/***************************************************************/

/*###############################*/
/*  Подключение общих библиотек  */
/*###############################*/
#include <SPI.h>
#include <Ethernet2.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include "thermosensors.h"

#ifdef LCD_ON
  #include <LiquidCrystal_I2C.h>
  LiquidCrystal_I2C lcd(0x27,20,4);
#endif

/****************************************************************/
/*        Объявление переменных используемых в программе        */
/****************************************************************/

/*###############################*/
/*      Объявление структур      */
/*###############################*/
struct HeatingControl {
	float curentAverageTemperature;		// Текущая средняя температура в доме
	uint8_t currentState;	// Состояние котла
	float targetTemperature;	// Текущая целевая температура
	float hysteresis;	// Допустимый гистерезис целевой температуры
	uint8_t heatingAuto;		// Авторежим
  uint8_t heatingChanged;
};

struct MessageMQTT {
	String topic;		// Текущая средняя температура в доме
	String payload;	// Состояние котла
};

struct TSensorData {
  int16_t tSensor1; // Данные с первого датчика температуры
  int16_t tSensor2; // Данные с второго датчика температуры
  int16_t tSensor3; // Данные с третьего датчика температуры
  int16_t tSensor4; // Данные с четвертого датчика температуры
  int16_t tSensor5; // Данные с пятого датчика температуры
};

/*##############################*/
/*        Настройки сети        */
/*##############################*/

// MAC-адрес для модуля W5500
byte mac[] = { 0x00, 0xAA, 0xBB, 0xCC, 0xDA, 0x01 };

// Настройки статического IP-адрес для домашней сети
#ifdef GET_STATIC_HOME
  IPAddress ip(172, 20, 10, 201);
  IPAddress ip_dns(172, 20, 10, 1);
  IPAddress ip_gateway(172, 20, 10, 1);
  IPAddress ip_subnet(255, 255, 255, 0);
#endif

// Настройки статического IP-адрес для рабочей сети
#ifdef GET_STATIC_WORK
  IPAddress ip(172, 16, 6, 201);
  IPAddress ip_dns(172, 16, 6, 1);
  IPAddress ip_gateway(172, 16, 6, 1);
  IPAddress ip_subnet(255, 255, 255, 0);
#endif

// Настройки статического IP-адрес для сети на даче
#ifdef GET_STATIC_COUNTRYHOUSE
  IPAddress ip(172, 25, 24, 201);
  IPAddress ip_dns(8, 8, 8, 8);
  IPAddress ip_gateway(172, 20, 20, 1);
  IPAddress ip_subnet(255, 255, 254, 0);
#endif
// Структура для работы с MQTT сервером
MessageMQTT messageMQTT;
// IP-адресс MQTT брокера в домашней сети
// IPAddress server(172, 20, 10, 131);
// IP-адресс MQTT брокера в рабочей сети
// IPAddress server(172, 16, 6, 40);
// IP-адресс MQTT брокера в дачной сети
IPAddress server(172, 25, 24, 119);

// Уставновить Логин и Пароль для подключения к MQTT брокеру
// Home
const char* mqtt_username = "corvin";
const char* mqtt_password = "eTx1243";
// Work
// const char* mqtt_username = "admin";
// const char* mqtt_password = "eTx1243";

EthernetClient ethClient;

/*################################*/
/*   Вспомагательные переменные   */
/*################################*/
// Признак первого запуска
uint8_t coldStart;

TSensorData sensorData;
/***************************************************************/
/*          Описание функций используемых в программе          */
/***************************************************************/

/*
 * Инициализируем работу портов
 */
void initializeVariables(void);
/*
 * Инициализируем LCD экран
 */
void initLCD(void);
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
 * Инициализируем датчики температуры
 */
void initSensor(void);
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
 * Включаем котел если он выключен
 */
void heatingON(void);

/*
 * Выключаем котл если он включен
 */
void heatingOFF(void);

/*
 * Отправляем данные термосенсора на сервер
 * dataToSend - температура для отправки
 */
uint8_t sendDataToServer(int8_t dataToSend);

/****************************************************************/
/*         Описание переменных используемых в программе         */
/****************************************************************/
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
unsigned long previousUpdateTime1;
unsigned long previousUpdateTime2;
unsigned long previousUpdateTime3;
unsigned long previousUpdateTime4;
unsigned long previousUpdateTime5;

HeatingControl heatingControl;
uint8_t saveSettingsToEEPROM(HeatingControl* heatingStruct);
uint8_t restoreSettingsFromEEPROM(HeatingControl* heatingStruct);


PubSubClient client(server, 1883, callback, ethClient);

int16_t getDigitalSensorData(OneWire &sensor, uint8_t portNumber);
int16_t getLM35SensorData(int analogSensorPin, uint8_t portNumber);

#endif /* MAIN_H_ */