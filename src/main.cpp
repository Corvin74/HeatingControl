#include "main.h"

void setup() {
  initializeVariables();
  initializeThePeriphery();
  initSerial();
  initNetwork();
  reconnect(); // Подключение к брокеру, подписка на прописанные выше темы
  mySensor1.startConversion();
  mySensor2.startConversion();
  mySensor3.startConversion();
}

void loop() {
  client.loop();
  if (heatingControl.heatingChanged) {
    heatingControl.heatingChanged = 0;
    messageMQTT.topic = "";
    messageMQTT.payload = "";
  }
  char dataTempChar[5];
  unsigned long currentMillis = millis();
  if (currentMillis - previousUpdateTime1 > TEMP1_UPDATE_TIME) {
    previousUpdateTime1 = currentMillis;
    mySensor1.readScratchpad();
    mySensor1.startConversion();
    #ifdef DEBUG
      Serial.print(F("Sensor1 = "));
      Serial.print(mySensor1.currentTemperature);
      Serial.println(" °C");
    #endif
    dtostrf(mySensor1.currentTemperature, 5, 2, dataTempChar);
    client.publish("/countryhouse/ds18b20_1", dataTempChar);
  }
  if (currentMillis - previousUpdateTime2 > TEMP2_UPDATE_TIME) {
    previousUpdateTime2 = currentMillis;
    mySensor2.readScratchpad();
    mySensor2.startConversion();
    #ifdef DEBUG
      Serial.print(F("Sensor2 = "));
      Serial.print(mySensor2.currentTemperature);
      Serial.println(" °C");
    #endif
    dtostrf(mySensor2.currentTemperature, 5, 2, dataTempChar);
    client.publish("/countryhouse/ds18b20_2", dataTempChar);
    calcAvarage(mySensor1.currentTemperature, mySensor2.currentTemperature);
    if (heatingControl.curentAverageTemperature > heatingControl.targetTemperature) {
      client.publish("/countryhouse/heating/Command", "ON");
      digitalWrite(RELAY1_PIN, LOW);
      heatingControl.heatingChanged = 1;
    } else {
      client.publish("/countryhouse/heating/Command", "OFF");
      digitalWrite(RELAY1_PIN, HIGH);
      heatingControl.heatingChanged = 1;
    }
  }
  if (currentMillis - previousUpdateTime3 > TEMP3_UPDATE_TIME) {
    previousUpdateTime3 = currentMillis;
    mySensor3.readScratchpad();
    mySensor3.startConversion();
    #ifdef DEBUG
      Serial.print(F("Sensor3 = "));
      Serial.print(mySensor3.currentTemperature);
      Serial.println(" °C");
    #endif
    dtostrf(mySensor3.currentTemperature, 5, 2, dataTempChar);
    client.publish("/countryhouse/ds18b20_3", dataTempChar);
    // calcAvarage(mySensor1.currentTemperature, mySensor2.currentTemperature);
  }
  if ((messageMQTT.topic == "/countryhouse/heating/Command") && !(heatingControl.heatingChanged)) {
    #ifdef DEBUG
      Serial.print("Topic: ");
      Serial.print(messageMQTT.topic);
      Serial.print(" - ");
      Serial.println(messageMQTT.payload);
    #endif
    if (messageMQTT.payload == "OFF") {
      // client.publish("/countryhouse/heating/State", "OFF");
      #ifdef DEBUG
        Serial.print("Incoming command: ");
        Serial.println(messageMQTT.payload);
      #endif
      digitalWrite(RELAY1_PIN, HIGH);
      heatingControl.heatingChanged = 1;
    } else {
      // client.publish("/countryhouse/heating/State", "ON");
      #ifdef DEBUG
        Serial.print("Incoming command: ");
        Serial.println(messageMQTT.payload);
      #endif
      digitalWrite(RELAY1_PIN, LOW);
      heatingControl.heatingChanged = 1;
    }
  }
  if (messageMQTT.topic == "/countryhouse/heating/targetTemperature") {
    heatingControl.targetTemperature = messageMQTT.payload.toFloat();
  }
}

/*
 * Инициализируем переменные используемые в программе
 */
void initializeVariables(void){
  coldStart = 1;

  dsSensor1 = -200.0;
  dsSensor2 = -200.0;
  dsSensor3 = -200.0;

  heatingControl.curentAverageTemperature = 0.0;
  heatingControl.heatingState = 0;
  heatingControl.targetTemperature = 0.0;
  heatingControl.targetTemperatureHiden = 0.0;
  heatingControl.heatingCommand = 0;
  heatingControl.heatingChanged = 0;

  previousUpdateTime1 = 0;
  previousUpdateTime2 = 0;
  previousUpdateTime3 = 0;
}
/*
 * Инициализируем работу портов
 */
void initializeThePeriphery(void){
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(RELAY1_PIN, HIGH);
  digitalWrite(RELAY2_PIN, HIGH);
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
}
/*
 * Инициализируем работу по шине UART на скорости 9600 бит/сек.
 */
 void initSerial(void){
   Serial.begin(9600);
   while (!Serial) {
     delay(100); // hang out until serial port opens
   }
 }
/*
 * Инициализируем работу с сетью через EthernetShield W5500 с помощью библиотеки
 * Ethernet2, если использовать EthernetShield W5100 надо применять библиотеку
 * Ethernet
 */
void initNetwork(void){
  #ifdef DEBUG
    Serial.println(F("Start ethernet..."));
  #endif
  #ifdef GET_DHCP
    if (Ethernet.begin(mac) == 0) {
      Serial.println(F("!!!Failed to get IP address from DHCP server!!!"));
      flashLed(LED_PIN, 250, 20);
    }
    flashLed(LED_PIN, 100, 2);
    // Выводим в консоль адрес присвоеный интерфейсу
    Serial.print(F("My DHCP IP address: "));
  #else
    if (Ethernet.begin(mac,ip) == 0) {
      Serial.println(F("!!!Failed to get IP address from DHCP server!!!"));
      flashLed(LED_PIN, 250, 20);
    }
    // Выводим в консоль адрес присвоеный интерфейсу
    Serial.print(F("My static IP address: "));
    flashLed(LED_PIN, 100, 2);
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
 * Возвращает среднее значение темаературы
 */
float calcAvarage(float sensor1, float sensor2){
  #ifdef DEBUG
    Serial.print(F("sensor1 = "));
    Serial.print(sensor1);
    Serial.println(F(" °C"));
    Serial.print(F("sensor2 = "));
    Serial.print(sensor2);
    Serial.println(F(" °C"));
  #endif
  char tempChar[5];
  if ((sensor1 > -200) && (sensor2 > -200)){
    heatingControl.curentAverageTemperature = (sensor1 + sensor2) / 2;
    dtostrf(heatingControl.curentAverageTemperature, 5, 2, tempChar);
    #ifdef DEBUG
      Serial.print(F("Average temperature = "));
      Serial.print(tempChar);
      Serial.println(F(" °C"));
    #endif
    client.publish("/countryhouse/heating/curentTemperature", tempChar);
    return heatingControl.curentAverageTemperature;
  }
}
/*
 * Мигаем светодиодом
 * ledPin - пин к которому подключен светодиод
 * time - время в мс
 * quantity - количество вспышек
 */
void flashLed(uint8_t ledPin, uint16_t time, uint8_t quantity){
  for (uint8_t i = 0; i <= quantity; i++) {
    digitalWrite(ledPin, HIGH);
    delay(time);
    digitalWrite(ledPin, LOW);
    delay(time);
  }
}

/*############################## Work with MQTT ##############################*/

/*
 * Колбэк функция для работы с MQTT-брокером
 */
void callback(char* topic, byte* payload, uint16_t length){
  messageMQTT.topic = "";
  messageMQTT.payload = "";
  messageMQTT.topic = String(topic);
  for (uint16_t i = 0; i < length; i++) {                // отправляем данные из топика
    messageMQTT.payload += String((char)payload[i]);
  }
}
/*
 * Подключение к брокеру MQTT, подписка на топики и начальная инициализация состояний
 * реле, выключателей и т.д.
 */
boolean reconnect(void) {
  if (client.connect(DEVICE_NAME, mqtt_username, mqtt_password)) {
    //TODO Настроить загрузку начальных значений из EEPROM
    if (coldStart) {
      client.publish("/countryhouse/heating/Command", "OFF");
      client.publish("/countryhouse/heating/curentTemperature", "0.00");
      client.publish("/countryhouse/heating/targetTemperature", "21.50");
      client.publish("/countryhouse/heating/targetTemperatureHiden", "10.00");
      coldStart = 0;
    }
    client.subscribe("/countryhouse/heating/State");
    client.subscribe("/countryhouse/heating/Command");
    #ifdef DEBUG
      Serial.println(F("Connected to: /countryhouse/heating/State"));
    #endif
    client.subscribe("/countryhouse/heating/curentTemperature");
    #ifdef DEBUG
      Serial.println(F("Connected to: /countryhouse/heating/curentTemperature"));
    #endif
    client.subscribe("/countryhouse/heating/targetTemperature");
    #ifdef DEBUG
      Serial.println(F("Connected to: /countryhouse/heating/targetTemperature"));
    #endif
    client.subscribe("/countryhouse/heating/targetTemperatureHiden");
    #ifdef DEBUG
      Serial.println(F("Connected to: /countryhouse/heating/targetTemperatureHiden"));
    #endif
  }
  return client.connected();
}

/*########################### End work with MQTT #############################*/
