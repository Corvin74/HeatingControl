#include "main.h"
#include "ds18b20.h"
Ds18b20 mySensor1(ds18b20Sensor1, sizeof(ds18b20Sensor1), 5000);
Ds18b20 mySensor2(ds18b20Sensor2, sizeof(ds18b20Sensor2), 10000);

void setup() {
  initializeThePeriphery();
  initSerial();
  initNetwork();
  reconnect(); // Подключение к брокеру, подписка на прописанные выше темы
  mySensor1.startConversion();
  mySensor2.startConversion();
}

void loop() {
  client.loop();
  unsigned long currentMillis = millis();
  if (currentMillis - previousUpdateTime1 > TEMP1_UPDATE_TIME) {
    previousUpdateTime1 = currentMillis;
    mySensor1.readScratchpad();
    mySensor1.startConversion();
    ds18b20TemperatureSensor1 = mySensor1.currentTemperature;
    #ifdef DEBUG
      Serial.print(F("Sensor1 = "));
      Serial.print(ds18b20TemperatureSensor1);
      Serial.println(" °C");
    #endif
    char dataTempChar[5];
    dtostrf(ds18b20TemperatureSensor1, 5, 2, dataTempChar);
    client.publish("/countryhouse/ds18b20_1", dataTempChar);
  }
  if (currentMillis - previousUpdateTime2 > TEMP2_UPDATE_TIME) {
    previousUpdateTime2 = currentMillis;
    mySensor2.readScratchpad();
    mySensor2.startConversion();
    // ds18b20TemperatureSensor2 = mySensor2.currentTemperature;
    #ifdef DEBUG
      Serial.print(F("Sensor2 = "));
      Serial.print(mySensor2.currentTemperature);
      Serial.println(" °C");
    #endif
    char dataTempChar[5];
    dtostrf(mySensor2.currentTemperature, 5, 2, dataTempChar);
    client.publish("/countryhouse/ds18b20_2", dataTempChar);
  }

  if ( ( digitalRead( BUTTON_PIN ) == HIGH ) && ( ledState == 0 ) ) { // Если кнопка нажата
    digitalWrite(LED_PIN, HIGH);// зажигаем светодиод
    client.publish("/countryhouse/sensor_key", "ON");
    ledState = 1;
    delay(500);
  }
  if ( ( digitalRead( BUTTON_PIN ) == HIGH ) && ( ledState == 1 ) ) { // Если кнопка нажата
    digitalWrite(LED_PIN, LOW);// зажигаем светодиод
    client.publish("/countryhouse/sensor_key", "OFF");
    ledState = 0;
    delay(500);
  }
}

/*
 * Инициализируем работу портов
 */
void initializeThePeriphery(){
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_STRIP, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(RELAY1_PIN, HIGH);
  digitalWrite(RELAY2_PIN, HIGH);
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
}

/*
 * Инициализируем работу по шине UART на скорости 9600 бит/сек.
 */
 void initSerial(void)
 {
   /*
    * Инициируем передачу данных по шине UART на скорости 9600 бит/сек.
    */
   Serial.begin(9600);
   while (!Serial) {
     delay(100); // hang out until serial port opens
   }
 }
/*############################## End initSerial ##############################*/

/*
 * Инициализируем работу с сетью через EthernetShield W5500 с помощью библиотеки
 * Ethernet2, если использовать EthernetShield W5100 надо применять библиотеку
 * Ethernet
 */
void initNetwork(void)
{
  #ifdef DEBUG
    Serial.println(F("Start ethernet..."));
  #endif

  #ifdef GET_DHCP
    if (Ethernet.begin(mac) == 0) {
      Serial.println(F("!!!Failed to get IP address from DHCP server!!!"));
      for (;;){
        digitalWrite(LED_PIN, HIGH);
        delay(250);
        digitalWrite(LED_PIN, LOW);
        delay(250);
      }
    }
    digitalWrite(LED_PIN, HIGH);
    delay(150);
    digitalWrite(LED_PIN, LOW);
    delay(150);
    // Выводим в консоль адрес присвоеный интерфейсу
    Serial.print(F("My DHCP IP address: "));
  #else
    if (Ethernet.begin(mac,ip) == 0) {
      Serial.println(F("!!!Failed to get IP address from DHCP server!!!"));
      for (;;){
        digitalWrite(LED_PIN, HIGH);
        delay(250);
        digitalWrite(LED_PIN, LOW);
        delay(250);
      }
    }
    // Выводим в консоль адрес присвоеный интерфейсу
    Serial.print(F("My static IP address: "));
    digitalWrite(LED_PIN, HIGH);
    delay(150);
    digitalWrite(LED_PIN, LOW);
    delay(150);
  #endif

  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    // print the value of each byte of the IP address:
    if (thisByte != 3) {
      Serial.print(Ethernet.localIP()[thisByte], DEC);
      Serial.print(".");
    } else {
      Serial.println(Ethernet.localIP()[thisByte], DEC);
    }

  }
}
/*############################# End initNetwork ##############################*/

/*
 * Инициируем работу с датчиком DS18B20
 */
void initDS18B20(void){
  Serial.print("ROM sensor1 =");
  for( byte i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(ds18b20Sensor1[i], HEX);
  }
  Serial.println();
  Serial.print("ROM sensor2 =");
  for( byte i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(ds18b20Sensor2[i], HEX);
  }
  Serial.println();
}
/*############################# End initDS18B20 ##############################*/

/*
 * Инициируем опрос датчиков DS18B20 с целью запуска преобразования температуры
 */
void ds18b20StartConversion(byte ds18b20Addr[8]){
  ds.reset();
  ds.select(ds18b20Addr);
  ds.write(0x44, 1);
}
/*######################## End ds18b20StartConversion ########################*/

/*
 * Считываем с датчиков DS18B20 текущую информацию
 */
float ds18b20ReadScratchpad(byte ds18b20Addr[8]){
  ds.reset();
  ds.select(ds18b20Addr);
  ds.write(0xBE);         // Read Scratchpad

  for ( byte i = 0; i < 9; i++) {           // we need 9 bytes
    ds18b20Data[i] = ds.read();
    Serial.print(ds18b20Data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" - ");
  int16_t raw = (ds18b20Data[1] << 8) | ds18b20Data[0];

  byte cfg = (ds18b20Data[4] & 0x60);
  // at lower res, the low bits are undefined, so let's zero them
  if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
  else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
  else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
  //// default is 12 bit resolution, 750 ms conversion time

  float ds18b20Temperature = (float)raw / 16.0;
  ds18b20StartConversion(ds18b20Addr);
  return ds18b20Temperature;
}
/*######################## End ds18b20ReadScratchpad #########################*/

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
    byte ledToPWM = map(Led, 0, 100, 0, 255);
    analogWrite(LED_STRIP, ledToPWM);
    Serial.print("Znachenie prisvoenoe peremennoy Led: ");
    Serial.println(Led);
    char dataTempChar[5];
    if (Led < 10) {
      dtostrf(Led, 1, 0, dataTempChar);
    } else if ((Led > 9) && (Led < 100)) {
      dtostrf(Led, 2, 0, dataTempChar);
    } else {
      dtostrf(Led, 3, 0, dataTempChar);
    }
    client.publish("/countryhouse/led_in", dataTempChar);
  }

  if (String(topic) == "/countryhouse/relay1") {
    if (strPayload == "ON"){
      digitalWrite(RELAY1_PIN, LOW);
    } else if (strPayload == "OFF"){
      digitalWrite(RELAY1_PIN, HIGH);
    }
  }

  if (String(topic) == "/countryhouse/relay2") {
    if (strPayload == "ON"){
      digitalWrite(RELAY2_PIN, LOW);
    } else if (strPayload == "OFF"){
      digitalWrite(RELAY2_PIN, HIGH);
    }
  }

  if (String(topic) == "/countryhouse/sensor_key") {
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
// --------------------------------------- BEGIN - Подключение и подписка на MQTT broker ----------------------------------

/*
 * Подключение к брокеру MQTT, подписка на топики и начальная инициализация состояний
 * реле, выключателей и т.д.
 */
boolean reconnect(void) {
  #ifdef DEBUG
    Serial.println(F("reconnect..."));
  #endif
  if (client.connect(DEVICE_NAME, mqtt_username, mqtt_password)) {
    //TODO Настроить загрузку начальных значений из EEPROM
    if (ColdStart) {
      client.publish("/countryhouse/led_in", "0");
      client.publish("/countryhouse/relay1in", "OFF");
      client.publish("/countryhouse/relay2in", "OFF");
      client.publish("/countryhouse/sensor_key", "OFF");
      ColdStart = 0;
      // digitalWrite(LED_PIN, HIGH);
    }
    client.subscribe("/countryhouse/led");
    #ifdef DEBUG
      Serial.println(F("Connected to: /countryhouse/led"));
    #endif
    client.subscribe("/countryhouse/relay1");
    #ifdef DEBUG
      Serial.println(F("Connected to: /countryhouse/relay1"));
    #endif
    client.subscribe("/countryhouse/relay2");
    #ifdef DEBUG
      Serial.println(F("Connected to: /countryhouse/relay2"));
    #endif
    client.subscribe("/countryhouse/sensor_key");
    #ifdef DEBUG
      Serial.println(F("Connected to: /countryhouse/sensor_key"));
    #endif
  }
  return client.connected();
}
// --------------------------------------- END - Подключение и подписка на MQTT broker ----------------------------------
