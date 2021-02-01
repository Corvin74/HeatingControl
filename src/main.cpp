#include <Arduino.h>
#include <SPI.h>
#include <Ethernet2.h>
#include <PubSubClient.h>
#include <OneWire.h>

#define LED_PIN 9
#define BUTTON_PIN 4
// -------------------------------------- BEGIN - Пины Arduino ----------------------------------------------
#define LED_pin 6                         //Пин 5 для светодиодов
#define Relay1_pin 7                      //Пин 6 для реле 1
#define Relay2_pin 8                      //Пин 7 для реле 2
// -------------------------------------- END - Пины Arduino ------------------------------------------------

OneWire  ds(2);  // on pin 10 (a 4.7K resistor is necessary)
byte data[12];
byte addr[8];
float celsius;
byte present = 0;
byte i;

byte ledState = 0;
// -------------------------------------- BEGIN - Глобальные переменные -------------------------------------
byte Led = 0;                             //Переменная для хранения состояния светодиода
boolean Relay1 = HIGH;                    //Переменная для хранения состояния Реле 1
boolean Relay2 = HIGH;                    //Переменная для хранения состояния Реле 2
boolean SensorKey = LOW;                 //Переменная для хранения состояния сенсорной кнопки
// -------------------------------------- END - Глобальные переменные ---------------------------------------


void callback(char* topic, byte* payload, unsigned int length);

// Утановить IP адресс для этой Arduino (должен быть уникальным в вашей сети)
IPAddress ip(172, 20, 20, 195);

byte mac[] = {
  0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02
};

// Уставновить IP адресс MQTT брокера
byte server[] = { 172, 20, 20, 125 };

// Уставновить Логин и Пароль для подключения к MQTT брокеру
const char* mqtt_username = "corvin";
const char* mqtt_password = "eTx1243";

EthernetClient ethClient;
PubSubClient client(server, 1883, callback, ethClient);

// --------------------------------------- BEGIN - Подключение и подписка на MQTT broker ----------------------------------
boolean reconnect() {
  //Serial.println("reconnect...");
  if (client.connect("Arduino_test", mqtt_username, mqtt_password)) {
    client.publish("/countryhouse/led_in", "0");
    client.publish("/countryhouse/relay1in", "1");
    client.publish("/countryhouse/relay2in", "1");
    client.publish("/TestMQTT/in", "OFF");
    client.subscribe("/countryhouse/led"); Serial.println("Connected to: /countryhouse/led");
    client.subscribe("/countryhouse/relay1"); Serial.println("Connected to: /countryhouse/relay1");
    client.subscribe("/countryhouse/relay2"); Serial.println("Connected to: /countryhouse/relay2");
    client.subscribe("/TestMQTT/out"); Serial.println("Connected to: /TestMQTT/out");
//    Serial.println("MQTT connected");
  }
  return client.connected();
}
// --------------------------------------- END - Подключение и подписка на MQTT broker ----------------------------------


//EthernetClient client;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  // digitalWrite(LED_PIN, HIGH); // Решение проблемы с LOW статусом пинов при загрузке ардуино

  digitalWrite(Relay1_pin, HIGH); // Решение проблемы с LOW статусом пинов при загрузке ардуино
  digitalWrite(Relay2_pin, HIGH); // Решение проблемы с LOW статусом пинов при загрузке ардуино
  pinMode(LED_pin, OUTPUT);
  pinMode(Relay1_pin, OUTPUT);
  pinMode(Relay2_pin, OUTPUT);

  Serial.begin(9600); // Open serial communications

  // Start with a hard-coded address:
//  Ethernet.begin(mac, ip);

//  Serial.print("My ip address: ");
//  Serial.println(Ethernet.localIP());

  if (Ethernet.begin(mac) == 0) {
    for (;;){
      digitalWrite(LED_PIN, HIGH);
      delay(500);
      digitalWrite(LED_PIN, LOW);
      delay(500);
    }
  }
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
  reconnect(); // Подключение к брокеру, подписка на прописанные выше темы
  if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
    return;
  }
}

void loop() {
  client.loop();
  // if ( !ds.search(addr)) {
  //   Serial.println("No more addresses.");
  //   Serial.println();
  //   ds.reset_search();
  //   delay(250);
  //   return;
  // }
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);
  delay(750);
  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad
  // Serial.print("  Data = ");
  // Serial.print(present, HEX);
  // Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
     data[i] = ds.read();
     // Serial.print(data[i], HEX);
  //   Serial.print(" ");
  }
  // Serial.print(" CRC=");
  // Serial.print(OneWire::crc8(data, 8), HEX);
  // Serial.println();

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];

  byte cfg = (data[4] & 0x60);
  // at lower res, the low bits are undefined, so let's zero them
  if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
  else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
  else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
  //// default is 12 bit resolution, 750 ms conversion time

  celsius = (float)raw / 16.0;
  Serial.print("  Temperature = ");
  Serial.print(celsius);
  Serial.println(" Celsius, ");

  if ( ( digitalRead( BUTTON_PIN ) == HIGH ) && ( ledState == 0 ) ) { // Если кнопка нажата
    digitalWrite(LED_PIN, HIGH);// зажигаем светодиод
    client.publish("/TestMQTT/in", "ON");
    ledState = 1;
    delay(500);
  }
  if ( ( digitalRead( BUTTON_PIN ) == HIGH ) && ( ledState == 1 ) ) { // Если кнопка нажата
    digitalWrite(LED_PIN, LOW);// зажигаем светодиод
    client.publish("/TestMQTT/in", "OFF");
    ledState = 0;
    delay(500);
  }
}

// --------------------------------------- BEGIN - void callback ------------------------------------------
// Чтение данных из MQTT брокера
void callback(char* topic, byte* payload, unsigned int length) {
  // проверка новых сообщений в подписках у брокера
    payload[length] = '\0';
    Serial.print("Topic: ");
    Serial.print(String(topic));
    Serial.println(" - ");
    String strPayload = String((char*)payload);

  if (String(topic) == "/countryhouse/led") {
    String value = String((char*)payload);
    Led = value.substring(0, value.indexOf(';')).toInt();
    Led = map(Led, 0, 100, 0, 255);
    analogWrite(LED_pin, Led);
    Serial.print("Znachenie prisvoenoe peremennoy Led: ");
    Serial.println(Led);
  }

  if (String(topic) == "/countryhouse/relay1") {
    String value = String((char*)payload);
    Relay1 = value.substring(0, value.indexOf(';')).toInt();
    Serial.print("Znachenie prisvoenoe peremennoy Relay1: ");
    Serial.println(Relay1);
    digitalWrite(Relay1_pin, Relay1);
  }

  if (String(topic) == "/countryhouse/relay2") {
    String value = String((char*)payload);
    Relay2 = value.substring(0, value.indexOf(';')).toInt();
    Serial.print("Znachenie prisvoenoe peremennoy Relay2: ");
    Serial.println(Relay2);
    digitalWrite(Relay2_pin, Relay2);
  }

  if (String(topic) == "/TestMQTT/out") {
    if (strPayload == "OFF"){
      digitalWrite(LED_PIN, LOW);
      ledState = 0;
    } else if (strPayload == "ON"){
      digitalWrite(LED_PIN, HIGH);
      ledState = 1;
    }
//    String value = String((char*)payload);
//    SensorKey = value.substring(0, value.indexOf(';')).toInt();
//    Serial.print("Znachenie prisvoenoe peremennoy SensorKey: ");
//    Serial.println(SensorKey);
//    ledState = SensorKey;
//    digitalWrite(LED_PIN, SensorKey);
  }

}
// ---------------------------------------- END - void callback -------------------------------------------
