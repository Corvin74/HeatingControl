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
#include "ds18b20.h"
/**************************************************************/
/* Настройка дополнительных параметров: отладка, сеть, etc... */
/**************************************************************/

// При отладке компилировать с диагностическими сообщениями
#ifndef DEBUG
  #define DEBUG
#endif

// Для инициализации IP адреса по DHCP
#ifndef DHCP
  #define GET_DHCP
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
void calcAvarage(float sensor1, float sensor2);

/****************************************************************/
/*         Описание переменных используемых в программе         */
/****************************************************************/
// Признак первого запуска
uint8_t coldStart = 1;
// Адреса датчиков используемых в проекте
uint8_t ds18b20Sensor1[8] = { 0x28, 0xFF, 0x64, 0xAA, 0xB2, 0x16, 0x05, 0x7E };
uint8_t ds18b20Sensor2[8] = { 0x28, 0xFF, 0x2F, 0x99, 0x50, 0x17, 0x04, 0x35 };
uint8_t ds18b20Sensor3[8] = { 0x28, 0xFF, 0x2F, 0x99, 0x50, 0x17, 0x04, 0x35 };
// Текущие данные с датчиков температуры
float dsSensor1 = -200.0;
float dsSensor2 = -200.0;
float dsSensor3 = -200.0;
// Переменные для хранения времени последнего обновления данных
long previousUpdateTime1;
long previousUpdateTime2;
long previousUpdateTime3;

byte ledState = 0;

struct HeatingControl {
	float curentAverageTemperature;		// Текущая средняя температура в доме
	byte heatingState;	// Состояние котла
	float targetTemperature;	// Идентификатор пакета.
	float targetTemperatureHiden;	// ID исполнительного устройства, может принимать значения 1-254
	byte heatingCommand;		// Название параметра (feedTemperature, returnTemperature, relayState...)
};

HeatingControl heatingControl;

// -------------------------------------- BEGIN - Глобальные переменные -------------------------------------
byte Led = 0;                             //Переменная для хранения состояния светодиода
boolean Relay1 = HIGH;                    //Переменная для хранения состояния Реле 1
boolean Relay2 = HIGH;                    //Переменная для хранения состояния Реле 2
boolean SensorKey = LOW;                 //Переменная для хранения состояния сенсорной кнопки
// -------------------------------------- END - Глобальные переменные ---------------------------------------

// Утановить IP адресс для этой Arduino (должен быть уникальным в вашей сети)
IPAddress ip(172, 20, 20, 195);

byte mac[] = { 0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02 };

// Уставновить IP адресс MQTT брокера
// byte server[] = { 172, 16, 6, 40 };
IPAddress server(172, 16, 6, 40);

// Уставновить Логин и Пароль для подключения к MQTT брокеру
const char* mqtt_username = "corvin";
const char* mqtt_password = "eTx1243";

EthernetClient ethClient;
PubSubClient client(server, 1883, callback, ethClient);
#endif /* MAIN_H_ */
