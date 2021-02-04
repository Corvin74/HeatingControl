/*
 * main.h
 *
 *  Created on: 3 окт. 2018 г.
 *      Author: corvin
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <Arduino.h>
// #include <LiquidCrystal_I2C.h>
// #include <DallasTemperature.h>
// #include <EEPROM.h>
#include <SPI.h>
#include <Ethernet2.h>
#include <PubSubClient.h>
#include <OneWire.h>
// #include <Wire.h>
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
/******************************************************************************/
// Название устройства при подключении к MQTT
#define DEVICE_NAME "TermostatOnArduino"

// Для DS18B20
#define ONEWIRE_BUS 2				// Номер пина Arduino с подключенным датчиком
#define TEMP1_UPDATE_TIME 12000		// Определяем периодичность проверок
#define TEMP2_UPDATE_TIME 10000		// Определяем периодичность проверок
#define SEND_UPDATE_TIME 120000		// Определяем периодичность отправки через RS485

// Для RS485
#define SerialTxControl 11			// RS485 указываем номер вывода arduino, к которому подключены выводы RE и DE конвертирующего модуля
#define RS485Transmit    HIGH
#define RS485Receive     LOW
char buffer[100];					// Буфер приема информации через RS485

#define LED_PIN 9
#define BUTTON_PIN 4

// -------------------------------------- BEGIN - Пины Arduino ----------------------------------------------
#define LED_STRIP 6                         //Пин 6 для светодиодов
#define RELAY1_PIN 7                      //Пин 6 для реле 1
#define RELAY2_PIN 8                      //Пин 7 для реле 2
// -------------------------------------- END - Пины Arduino ------------------------------------------------
void callback(char* topic, byte* payload, unsigned int length);
boolean reconnect(void);

/**
  * Инициируем всю необходимую переферию
  */
void initializeThePeriphery(void);
/**
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
 * Инициируем работу с датчиком DS18B20
 */
void initDS18B20(void);
/*
 * Инициируем опрос датчиков DS18B20 с целью запуска преобразования температуры
 */
void ds18b20StartConversion(byte ds18b20Addr[8]);
/*
 * Считываем с датчиков DS18B20 текущую информацию
 */
float ds18b20ReadScratchpad(byte ds18b20Addr[8]);

boolean ColdStart = 1;

OneWire  ds(ONEWIRE_BUS);  // on pin 10 (a 4.7K resistor is necessary)
byte ds18b20Data[12];
//byte ds18b20Addr[8];
uint8_t ds18b20Sensor1[8] = { 0x28, 0xFF, 0x64, 0xAA, 0xB2, 0x16, 0x05, 0x7E };
uint8_t ds18b20Sensor2[8] = { 0x28, 0xFF, 0x2F, 0x99, 0x50, 0x17, 0x04, 0x35 };

float ds18b20TemperatureSensor1;
float ds18b20TemperatureSensor2;

// Переменная для хранения времени последней отправки данных
long previousUpdateTime1 = 0;
long previousUpdateTime2 = 0;
//long previousUpdateTime = 0;

byte ledState = 0;
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

//EthernetClient client;


// void printAddress(DeviceAddress deviceAddress);
// void printTemperature(DeviceAddress deviceAddress);
// void printResolution(DeviceAddress deviceAddress);
// void printData(DeviceAddress deviceAddress);
// float getTemperature(DeviceAddress deviceAddress);

// struct Message {
// 	byte DeviceID;		// ID устройства, может принимать значения 1-254
// 	byte DestinationID;	// Номер устройства-получателя, может принимать значения 0-254
// 						// (0 - броадкаст)
// 	uint8_t PacketID;	// Идентификатор пакета.
// 	byte ActuatorID;	// ID исполнительного устройства, может принимать значения 1-254
// 	byte CommandID;		// Название параметра (feedTemperature, returnTemperature, relayState...)
// 	int DataValue;		// Значение параметра типа int
// };
//
// uint8_t stateWork = 0b00000000; // Битовое поле статусов задач
/*
 *	0 - отправлен запрос на измерение температуры (0-нет, 1-да)
 *	1 - считана температура с датчика после последнего запроса (0-нет, 1-да)
 *	2 - статус первого тена (0-выключен, 1-включен)
 *	3 - статус второго тена (0-выключен, 1-включен)
 *	4 - статус третьего тена (0-выключен, 1-включен)
 *	5 - режим работы
 *	6 - режим работы
 *	7 - изменение температуры более чем на 1 градус
 */

#endif /* MAIN_H_ */
