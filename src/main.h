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
  #define WL102_ON
  // #undef WL102_ON
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
#define SENS2_UPTIME 6000
#define UPDATE_TIME2 6000
// Температура в погребе
#define SENS3_UPTIME 60000
#define UPDATE_TIME3 60000
// Температура в холле
#define SENS4_UPTIME 90000
#define UPDATE_TIME4 90000
// Температура на кухне
#define SENS5_UPTIME 120000
#define UPDATE_TIME5 120000

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
  // #define DEBUG
  #undef DEBUG
#endif
#ifndef DEBUGRF
  #define DEBUGRF
  // #undef DEBUGRF
#endif
#define LED_PIN 13                // Пин 13(PB5) с подключенным контрольным светодиодом
// Признак подключения LCD экрана
#ifndef LCD_ON
  #define LCD_ON
  // #undef LCD_ON
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
IPAddress server(172, 20, 10, 135);
// IP-адресс MQTT брокера в рабочей сети
// IPAddress server(172, 16, 6, 40);
// IP-адресс MQTT брокера в дачной сети
// IPAddress server(172, 25, 24, 119);

// Уставновить Логин и Пароль для подключения к MQTT брокеру
const char* mqtt_username = "corvin";
const char* mqtt_password = "eTx1243";

EthernetClient ethClient;

/*#################################*/
/*  Адреса температурных датчиков  */
/*  в теории их надо убрать т.к.   */
/*  на каждом порту только один    */
/*  датчик                         */
/*#################################*/

uint8_t ds18b20Sensor1[8] = { 0x28, 0xFF, 0xBC, 0xB4, 0xB2, 0x16, 0x05, 0x33 };
uint8_t ds18b20Sensor2[8] = { 0x28, 0x4B, 0x18, 0x43, 0x98, 0x01, 0x02, 0xCC };
uint8_t ds18b20Sensor3[8] = { 0x28, 0xFF, 0xAF, 0xA9, 0xB2, 0x16, 0x05, 0xA4 };
uint8_t ds18b20Sensor4[8] = { 0x28, 0x88, 0x21, 0x43, 0x98, 0x01, 0x02, 0x7D };
uint8_t ds18b20Sensor5[8] = { 0x28, 0x15, 0x00, 0x43, 0x98, 0x0F, 0x00, 0xD3 };

/*################################*/
/*   Вспомагательные переменные   */
/*################################*/
// Признак первого запуска
uint8_t coldStart;

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

HeatingControl heatingControl;
uint8_t saveSettingsToEEPROM(HeatingControl* heatingStruct);
uint8_t restoreSettingsFromEEPROM(HeatingControl* heatingStruct);


PubSubClient client(server, 1883, callback, ethClient);
#endif /* MAIN_H_ */
