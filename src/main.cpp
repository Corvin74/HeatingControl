#include "main.h"

void setup() {
  #ifdef LCD_ON
    initLCD();
  #endif
  initSerial();
  initializeVariables();
  initializeThePeriphery();
  initNetwork();
  reconnect(); // Подключение к брокеру, подписка на прописанные выше темы
}

void loop() {
  messageMQTT.topic = "";
  messageMQTT.payload = "";

  if (heatingControl.heatingChanged) {
  // if ((heatingControl.heatingChanged) && (messageMQTT.topic == "/countryhouse/heating/Command")) {
    heatingControl.heatingChanged = 0;
  }
  char dataTempChar[5];
  // unsigned long currentMillis = millis();

  if (millis() - previousUpdateTime1 > SENS1_UPTIME) {
    #ifdef SENSOR1_DIGITAL
      sensorData.tSensor1 = getDigitalSensorData(sensor1, 1);
      if (sensorData.tSensor1 == 8503) {
        while (sensorData.tSensor1 == 8503) {
          delay(250);
          sensorData.tSensor1 = getDigitalSensorData(sensor1, 1);
        }
      }
    #endif
    #ifdef SENSOR1_ANALOG
      sensorData.tSensor1 = getLM35SensorData(SENSOR1_ANALOG, 1);
    #endif
    if (sensorData.tSensor1 != -85) {
      dtostrf(sensorData.tSensor1/100.0, 5, 2, dataTempChar);
      if (!client.publish("/countryhouse/temperature/sensor1", dataTempChar)) {
        #ifdef DEBUG
          Serial.println(F("Publish sensor1 temperature failed"));
        #endif
      }
      #ifdef WL102_ON
        sendDataToServer(dataToSend);
      #endif
    }
    previousUpdateTime1 = millis();
  }

  if (millis() - previousUpdateTime2 > SENS2_UPTIME) {
    #ifdef SENSOR2_DIGITAL
      sensorData.tSensor2 = getDigitalSensorData(sensor2, 2);
      if (sensorData.tSensor2 == 8503) {
        while (sensorData.tSensor2 == 8503) {
          delay(250);
          sensorData.tSensor2 = getDigitalSensorData(sensor2, 2);
        }
      }
    #endif
    #ifdef SENSOR2_ANALOG
      sensorData.tSensor2 = getLM35SensorData(SENSOR2_ANALOG, 2);
    #endif
    if (sensorData.tSensor2 != -85) {
      dtostrf(sensorData.tSensor2/100.0, 5, 2, dataTempChar);
      if (!client.publish("/countryhouse/temperature/sensor2", dataTempChar)) {
        #ifdef DEBUG
          Serial.println(F("Publish sensor2 temperature failed"));
        #endif
      }
      #ifdef WL102_ON
        sendDataToServer(dataToSend);
      #endif
    }
    previousUpdateTime2 = millis();
  }

  if (millis() - previousUpdateTime3 > SENS3_UPTIME) {
    previousUpdateTime3 = millis();
    #ifdef SENSOR3_DIGITAL
      sensorData.tSensor3 = getDigitalSensorData(sensor3, 3);
    #endif
    #ifdef SENSOR3_ANALOG
      sensorData.tSensor3 = getLM35SensorData(SENSOR3_ANALOG, 3);
    #endif
    if (sensorData.tSensor3 != -85) {
      dtostrf(sensorData.tSensor3/100.0, 5, 2, dataTempChar);
      if (!client.publish("/countryhouse/temperature/sensor3", dataTempChar)) {
        #ifdef DEBUG
          Serial.println(F("Publish sensor3 temperature failed"));
        #endif
      }
      #ifdef WL102_ON
        sendDataToServer(dataToSend);
      #endif
    }
  }

  if (millis() - previousUpdateTime4 > SENS4_UPTIME) {
    previousUpdateTime4 = millis();
    #ifdef SENSOR4_DIGITAL
      sensorData.tSensor4 = getDigitalSensorData(sensor4, 4);
    #endif
    #ifdef SENSOR4_ANALOG
      sensorData.tSensor4 = getLM35SensorData(SENSOR4_ANALOG, 4);
    #endif
    if (sensorData.tSensor4 != -85) {
      // dtostrf(sensorData.tSensor4/100.0, 5, 2, dataTempChar);
      dtostrf((sensorData.tSensor4/100.0)/10.0, 5, 2, dataTempChar);
      if (!client.publish("/countryhouse/temperature/sensor4", dataTempChar)) {
        #ifdef DEBUG
          Serial.println(F("Publish sensor2 temperature failed"));
        #endif
      }
      #ifdef WL102_ON
        sendDataToServer(dataToSend);
      #endif
    }
  }

  if (millis() - previousUpdateTime5 > SENS5_UPTIME) {
    previousUpdateTime5 = millis();
    #ifdef SENSOR5_DIGITAL
      sensorData.tSensor5 = getDigitalSensorData(sensor5, 1);
    #endif
    #ifdef SENSOR5_ANALOG
      sensorData.tSensor5 = getLM35SensorData(SENSOR5_ANALOG, 5);
    #endif
    if (sensorData.tSensor5 != -85) {
      dtostrf(sensorData.tSensor5/100.0, 5, 2, dataTempChar);
      if (!client.publish("/countryhouse/temperature/sensor5", dataTempChar)) {
        #ifdef DEBUG
          Serial.println(F("Publish sensor5 temperature failed"));
        #endif
      }
      #ifdef WL102_ON
        sendDataToServer(dataToSend);
      #endif
    }
  }

/*
  if ((messageMQTT.topic == "/countryhouse/heating/Command") && !(heatingControl.heatingChanged)) {
    #ifdef DEBUG
      Serial.print("Topic: ");
      Serial.print(messageMQTT.topic);
      Serial.print(" - ");
      Serial.println(messageMQTT.payload);
    #endif
    if (messageMQTT.payload == "OFF") {
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
    #ifdef DEBUG
      Serial.print(F("Current target temperature = "));
      Serial.print(heatingControl.targetTemperature);
      Serial.println(" °C");
    #endif
    #ifdef DEBUG
      Serial.print(F("Current target temperature from topic = "));
      Serial.print(heatingControl.targetTemperature);
      Serial.println(" °C");
    #endif
    heatingControl.targetTemperature = messageMQTT.payload.toFloat();
    saveSettingsToEEPROM(&heatingControl);
  }
  if (messageMQTT.topic == "/countryhouse/heating/hysteresis") {
    #ifdef DEBUG
      Serial.print(F("Current hysteresis = "));
      Serial.print(heatingControl.hysteresis);
      Serial.println(" °C");
    #endif
    #ifdef DEBUG
      Serial.print(F("Current hysteresis from topic = "));
      Serial.print(messageMQTT.payload.toFloat());
      Serial.println(" °C");
    #endif
    heatingControl.hysteresis = messageMQTT.payload.toFloat();
    saveSettingsToEEPROM(&heatingControl);
  }
  if (messageMQTT.topic == "/countryhouse/heating/Auto") {
    #ifdef DEBUG
      Serial.print(F("Current state from topic 'Auto' = "));
      Serial.println(messageMQTT.payload);
    #endif
    if ((messageMQTT.payload == "OFF") && (heatingControl.heatingAuto == 1)) {
      heatingControl.heatingAuto = 0;
      digitalWrite(RELAY2_PIN, HIGH);
    } else if ((messageMQTT.payload == "ON") && (heatingControl.heatingAuto == 0)) {
      heatingControl.heatingAuto = 1;
      digitalWrite(RELAY2_PIN, LOW);
    }
    // saveSettingsToEEPROM(&heatingControl);
  }
  */
  client.loop();
}

/*######################## Initialization functions #########################*/

/*
 * Инициализируем LCD экран
 */
void initLCD(void){
  #ifdef DEBUG
    Serial.println(F("Initialize LCD 2004 I2C 0x27..."));
  #endif
  #ifdef LCD_ON
    lcd.init();
    lcd.backlight();
    lcd.setCursor(3,0);
    lcd.print(F("HeatingControl"));
    lcd.setCursor(3,1);
    lcd.print(F("DIP Ver: 0.0.2"));
    delay(500);
  #endif
}
/*
 * Инициализируем работу по шине UART на скорости 9600 бит/сек.
 */
void initSerial(void){
  #ifdef LCD_ON
    lcd.setCursor(0,2);
    lcd.print(F("Init serial...      "));
    delay(500);
  #endif
  Serial.begin(9600);
  while (!Serial) {
    delay(100); // hang out until serial port opens
  }
  #ifdef DEBUG
    Serial.println(F("Heating control setup start."));
    Serial.println(F("Main board DIP Ver: 0.0.2"));
    Serial.println(F("Initialize HW Serial complite..."));
  #endif
  flashLed(LED_PIN, 1000, 1);
  flashLed(LED_PIN, 50, 2);
}
/*
 * Инициализируем переменные используемые в программе
 */
void initializeVariables(void){
  #ifdef LCD_ON
    lcd.setCursor(0,2);
    lcd.print(F("Init variables...   "));
    delay(500);
  #endif
  #ifdef DEBUG
    Serial.println(F("Initializing variables..."));
  #endif
  coldStart = 1;

  dsSensor1 = -200.0;
  dsSensor2 = -200.0;
  dsSensor3 = -200.0;

  heatingControl.hysteresis = 0.5;
  restoreSettingsFromEEPROM(&heatingControl);

  previousUpdateTime1 = 0;
  previousUpdateTime2 = 0;
  previousUpdateTime3 = 0;
  previousUpdateTime4 = 0;
  previousUpdateTime5 = 0;

  sensorData.tSensor1 = -85;
  sensorData.tSensor2 = -85;
  sensorData.tSensor3 = -85;
  sensorData.tSensor4 = -85;
  sensorData.tSensor5 = -85;

}

/*###################### End Initialization functions #######################*/

/*
 * Инициализируем работу портов
 */
void initializeThePeriphery(void){
  #ifdef DEBUG
    Serial.println(F("Initialize periphery..."));
  #endif
  #ifdef LCD_ON
    lcd.setCursor(0,2);
    lcd.print(F("Init periphery...   "));
    delay(500);
  #endif
  #ifdef WL102_ON
    #ifdef DEBUG
      Serial.println(F("Initialize WL102..."));
    #endif
    #define WL102_TX_PIN PB1
    vw_setup(4800);     // Скорость соединения VirtualWire
    vw_set_tx_pin(WL102_TX_PIN);   // Вывод передачи VirtualWire
  #endif

  #ifdef DEBUG
    Serial.println(F("Initialize LED output..."));
  #endif
  pinMode(LED_PIN, OUTPUT);
  flashLed(LED_PIN, 1000, 1);
  flashLed(LED_PIN, 50, 2);
  #ifdef DEBUG
    Serial.println(F("Initialize RELAY output..."));
  #endif
  digitalWrite(RELAY1_PIN, HIGH);
  digitalWrite(RELAY2_PIN, HIGH);
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  flashLed(LED_PIN, 1000, 1);
  flashLed(LED_PIN, 50, 2);
  #ifdef DEBUG
    Serial.println(F("Initialize thermosensor..."));
  #endif
  initSensor();
}
/*
 * Инициализируем работу с сетью через EthernetShield W5500 с помощью библиотеки
 * Ethernet2, если использовать EthernetShield W5100 надо применять библиотеку
 * Ethernet
 */
void initNetwork(void){
  #ifdef LCD_ON
    lcd.setCursor(0,2);
    lcd.print("Init ethernet...   ");
    delay(500);
    lcd.noBacklight();
  #endif
  #ifdef DEBUG
    Serial.println(F("Start initialize ethernet..."));
  #endif
  #ifdef GET_DHCP
    flashLed(LED_PIN, 500, 2);
    if (Ethernet.begin(mac) == 0) {
      #ifdef LCD_ON
        lcd.setCursor(0,2);
        lcd.print("Failed to get IP add");
        lcd.setCursor(0,3);
        lcd.print("ress from DHCP server");
        delay(500);
        lcd.noBacklight();
      #endif
      #ifdef DEBUG
        Serial.println(F("!!!Failed to get IP address from DHCP server!!!"));
      #endif
      flashLed(LED_PIN, 2000, 5);
    }
    delay(500);
    // flashLed(LED_PIN, 1000, 2);
    // Выводим в консоль адрес присвоеный интерфейсу
    #ifdef DEBUG
      Serial.print(F("My DHCP IP address: "));
    #endif
    #ifdef LCD_ON
      lcd.setCursor(0,2);
      lcd.print("My DHCP IP address: ");
      delay(500);
      lcd.noBacklight();
    #endif
  #else
    Ethernet.begin(mac,ip,ip_dns,ip_gateway);
    // void EthernetClass::begin(IPAddress local_ip, IPAddress dns_server, IPAddress gateway, IPAddress subnet)
    // if (Ethernet.begin(mac,ip,ip_dns,ip_gateway,ip_subnet) == 0) {
    //   Serial.println(F("!!!Failed to get IP address from DHCP server!!!"));
    //   flashLed(LED_PIN, 250, 20);
    // }
    // Выводим в консоль адрес присвоеный интерфейсу
    #ifdef LCD_ON
      lcd.setCursor(0,2);
      lcd.print("My static IP addres:");
      delay(500);
      lcd.noBacklight();
    #endif
    #ifdef DEBUG
      Serial.print(F("My static IP address: "));
    #endif
    flashLed(LED_PIN, 100, 2);
  #endif

  #ifdef LCD_ON
    lcd.setCursor(0,3);
  #endif
  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    // print the value of each byte of the IP address:
    if (thisByte != 3) {
      #ifdef DEBUG
        Serial.print(Ethernet.localIP()[thisByte], DEC);
        Serial.print(".");
      #endif
      #ifdef LCD_ON
        lcd.print(Ethernet.localIP()[thisByte], DEC);
        lcd.print(".");
      #endif
    } else {
      #ifdef DEBUG
        Serial.println(Ethernet.localIP()[thisByte], DEC);
      #endif
      #ifdef LCD_ON
        lcd.print(Ethernet.localIP()[thisByte], DEC);
        delay(500);
        lcd.noBacklight();
      #endif
    }
  }
}
/*############################# End initNetwork ##############################*/
/*
 * Инициализируем датчики температуры
 */
void initSensor(void){
  #ifdef SENSOR1_DIGITAL
    if (sensor1.reset() == 1)
    {
      #ifdef DEBUG
        Serial.println(F("Digital sensor 1 present, send init measurement"));
      #endif
      sensor1.write(CMD_SKIPROM);
      sensor1.write(CMD_CONVERTTEMP);
      delay(1000);
    }
  #endif
  #ifdef SENSOR2_DIGITAL
    if (sensor2.reset() == 1)
    {
      #ifdef DEBUG
        Serial.println(F("Digital sensor 2 present, send init measurement"));
      #endif
      sensor2.write(CMD_SKIPROM);
      sensor2.write(CMD_CONVERTTEMP);
      delay(1000);
    }
  #endif
  #ifdef SENSOR3_DIGITAL
    if (sensor3.reset() == 1)
    {
      #ifdef DEBUG
        Serial.println(F("Digital sensor 3 present, send init measurement"));
      #endif
      sensor3.write(CMD_SKIPROM);
      sensor3.write(CMD_CONVERTTEMP);
      delay(1000);
    }
  #endif
  #ifdef SENSOR4_DIGITAL
    if (sensor4.reset() == 1)
    {
      #ifdef DEBUG
        Serial.println(F("Digital sensor 4 present, send init measurement"));
      #endif
      sensor1.write(CMD_SKIPROM);
      sensor1.write(CMD_CONVERTTEMP);
      delay(1000);
    }
  #endif
  #ifdef SENSOR5_DIGITAL
    if (sensor5.reset() == 1)
    {
      #ifdef DEBUG
        Serial.println(F("Digital sensor 5 present, send init measurement"));
      #endif
      sensor5.write(CMD_SKIPROM);
      sensor5.write(CMD_CONVERTTEMP);
      delay(1000);
    }
  #endif

  #ifdef SENSOR1_ANALOG
    // analogReference(INTERNAL);
    pinMode(SENSOR1_ANALOG, INPUT);
    #ifdef DEBUG
      Serial.println(F("Analog sensor 1 analog present, send init measurement"));
    #endif
    delay(1000);
  #endif
  #ifdef SENSOR2_ANALOG
    // analogReference(INTERNAL);
    pinMode(SENSOR2_ANALOG, INPUT);
    #ifdef DEBUG
      Serial.println(F("Analog sensor 2 analog present, send init measurement"));
    #endif
    delay(1000);
  #endif
  #ifdef SENSOR3_ANALOG
    // analogReference(INTERNAL);
    pinMode(SENSOR3_ANALOG, INPUT);
    #ifdef DEBUG
      Serial.println(F("Analog sensor 3 analog present, send init measurement"));
    #endif
    delay(500);
  #endif
  #ifdef SENSOR4_ANALOG
    // analogReference(INTERNAL);
    pinMode(SENSOR4_ANALOG, INPUT);
    #ifdef DEBUG
      Serial.println(F("Analog sensor 4 analog present, send init measurement"));
    #endif
    delay(500);
  #endif
  #ifdef SENSOR5_ANALOG
    // analogReference(INTERNAL);
    pinMode(SENSOR5_ANALOG, INPUT);
    #ifdef DEBUG
      Serial.println(F("Analog sensor 5 analog present, send init measurement"));
    #endif
    delay(1000);
  #endif
}
/*
 * Возвращает среднее значение темаературы
 */
float calcAvarage(float sensor1, float sensor2){
  // #ifdef DEBUG
  //   Serial.print(F("sensor1 = "));
  //   Serial.print(sensor1);
  //   Serial.println(F(" °C"));
  //   Serial.print(F("sensor2 = "));
  //   Serial.print(sensor2);
  //   Serial.println(F(" °C"));
  // #endif
  char tempChar[5];
  float curAvgTemp = 250.00;
  if ((sensor1 > -200) && (sensor2 > -200)){
    curAvgTemp = (sensor1 + sensor2) / 2;
    dtostrf(curAvgTemp, 5, 2, tempChar);
    // #ifdef DEBUG
    //   Serial.print(F("Average temperature = "));
    //   Serial.print(curAvgTemp);
    //   Serial.println(F(" °C"));
    // #endif
    client.publish("/countryhouse/heating/curentTemperature", tempChar);
    return curAvgTemp;
  }
  return curAvgTemp;
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

/*
 * Работаем с EEPROM
 */
uint8_t saveSettingsToEEPROM(HeatingControl* heatingStruct){
  int16_t eepromAddress = 0;
  #ifdef DEBUG
    Serial.print(F("heatingStruct->curentAverageTemperature save to EEPROM in arrdess: "));
    Serial.println(eepromAddress);
  #endif
  EEPROM.put(eepromAddress, heatingStruct->curentAverageTemperature);

  eepromAddress += sizeof(float);
  #ifdef DEBUG
    Serial.print(F("heatingStruct->currentState save to EEPROM in arrdess: "));
    Serial.println(eepromAddress);
  #endif
  EEPROM.put(eepromAddress,heatingStruct->currentState);

  eepromAddress += sizeof(uint8_t);
  #ifdef DEBUG
    Serial.print(F("heatingStruct->targetTemperature save to EEPROM in arrdess: "));
    Serial.println(eepromAddress);
  #endif
  EEPROM.put(eepromAddress,heatingStruct->targetTemperature);

  eepromAddress += sizeof(float);
  #ifdef DEBUG
    Serial.print(F("heatingStruct->hysteresis save to EEPROM in arrdess: "));
    Serial.println(eepromAddress);
  #endif
  EEPROM.put(eepromAddress,heatingStruct->hysteresis);

  eepromAddress += sizeof(float);
  #ifdef DEBUG
    Serial.print(F("heatingStruct->heatingAuto save to EEPROM in arrdess: "));
    Serial.println(eepromAddress);
  #endif
  EEPROM.put(eepromAddress,heatingStruct->heatingAuto);

  eepromAddress += sizeof(uint8_t);
  #ifdef DEBUG
    Serial.print(F("heatingStruct->heatingChanged save to EEPROM in arrdess: "));
    Serial.println(eepromAddress);
  #endif
  EEPROM.put(eepromAddress,heatingStruct->heatingChanged);

  return 0;
}
uint8_t restoreSettingsFromEEPROM(HeatingControl* heatingStruct){
  // #ifdef LCD_ON
  //   lcd.setCursor(0,2);
  //   lcd.print("Restore from EEPROM");
  //   delay(500);
  // #endif
  int16_t eepromAddress = 0;
  EEPROM.get(eepromAddress,heatingStruct->curentAverageTemperature);

  eepromAddress += sizeof(float);
  EEPROM.get(eepromAddress,heatingStruct->currentState);

  eepromAddress += sizeof(uint8_t);
  EEPROM.get(eepromAddress,heatingStruct->targetTemperature);

  eepromAddress += sizeof(float);
  EEPROM.get(eepromAddress,heatingStruct->hysteresis);

  eepromAddress += sizeof(float);
  EEPROM.get(eepromAddress,heatingStruct->heatingAuto);

  eepromAddress += sizeof(uint8_t);
  EEPROM.get(eepromAddress,heatingStruct->heatingChanged);

  return 0;
}

/*
 * Проверяем необходимость включения котла по средней температуре
 */
void checkHeatingAVG(void){
  // heatingControl.curentAverageTemperature = calcAvarage(mySensor1.currentTemperature, mySensor2.currentTemperature);
  if ((heatingControl.curentAverageTemperature - (heatingControl.hysteresis / 2)) < (heatingControl.targetTemperature + (heatingControl.hysteresis / 2))) {
    if (heatingControl.currentState == 0) {
      heatingOFF();
    }
  } else {
    if (heatingControl.currentState == 1) {
      heatingON();
    }
  }
}
/*
 * Отправляем данные термосенсора на сервер
 * dataToSend - температура для отправки
 */
uint8_t sendDataToServer(int8_t dataToSend) {
  #ifdef WL102_ON
    char msg0[3];
    itoa(dataToSend, msg0, 10);                    // Преобразование температуры в массив char
    #ifdef DEBUGRF
      // Serial.println(F("Start RF transmit"));
      Serial.println("Start RF transmit");
    #endif
    vw_send((uint8_t *)msg0, strlen(msg0)); // Передача сообщения
    vw_wait_tx();                           // Ждем завершения передачи
    #ifdef DEBUGRF
      // Serial.println(F("End RF transmit"));
      Serial.println("End RF transmit");
    #endif
  #endif
  return 0;
}
/*
 * Проверяем необходимость включения котла по температуре первого датчика
 */
void checkHeatingSensor1(void){
  // float sensor = mySensor1.currentTemperature;
  // float target = heatingControl.targetTemperature;
  // float hysteresis = heatingControl.hysteresis;
  // #ifdef DEBUG
  //   Serial.println(F("checkHeatingSensor1"));
  //   Serial.print(F("sensor = "));
  //   Serial.println(sensor);
  //   Serial.print(F("target = "));
  //   Serial.println(target);
  //   Serial.print(F("hysteresis = "));
  //   Serial.println(hysteresis);
  // #endif
  // if (sensor <= (target - hysteresis)) {
  //   if (heatingControl.currentState == 0) {
  //     heatingON();
  //   }
  // }
  // if (sensor >= (target + hysteresis)) {
  //   if (heatingControl.currentState == 1) {
  //     heatingOFF();
  //   }
  // }
}

/*
 * Включаем котл если он выключен
 */
void heatingON(void){
  client.publish("/countryhouse/heating/Command", "ON");
  digitalWrite(RELAY1_PIN, LOW);
  heatingControl.heatingChanged = 1;
  heatingControl.currentState = 1;
}
/*
 * Выключаем котл если он включен
 */
void heatingOFF(void){
  client.publish("/countryhouse/heating/Command", "OFF");
  digitalWrite(RELAY1_PIN, HIGH);
  heatingControl.heatingChanged = 1;
  heatingControl.currentState = 0;
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
  #ifdef DEBUG
    // Serial.println(F("########################################################"));
    // Serial.print(F("Current topic = "));
    // Serial.println(messageMQTT.topic);
    // Serial.print(F("Current payload = "));
    // Serial.println(messageMQTT.payload);
    // Serial.println(F("########################################################"));
  #endif

}
/*
 * Подключение к брокеру MQTT, подписка на топики и начальная инициализация состояний
 * реле, выключателей и т.д.
 */
boolean reconnect(void) {
  char tempChar[5];
  if (client.connect(DEVICE_NAME, mqtt_username, mqtt_password)) {
    //TODO Настроить загрузку начальных значений из EEPROM
    if (coldStart) {
      client.publish("/countryhouse/heating/Command", "OFF");
      client.publish("/countryhouse/heating/currentTemperature", "0.00");
      dtostrf(heatingControl.targetTemperature, 5, 2, tempChar);
      client.publish("/countryhouse/heating/targetTemperature", tempChar);
      client.publish("/countryhouse/heating/targetTemperatureHiden", tempChar);
      if (heatingControl.heatingAuto == 0) {
        client.publish("/countryhouse/heating/Auto", "OFF");
      } else {
        client.publish("/countryhouse/heating/Auto", "ON");
      }

      coldStart = 0;
    }
    client.subscribe("/countryhouse/heating/Auto");
    client.subscribe("/countryhouse/heating/State");
    #ifdef DEBUG
      Serial.println(F("Connected to: /countryhouse/heating/State"));
    #endif
    client.subscribe("/countryhouse/heating/Command");
    #ifdef DEBUG
      Serial.println(F("Connected to: /countryhouse/heating/Command"));
    #endif
    client.subscribe("/countryhouse/heating/curentTemperature");
    #ifdef DEBUG
      Serial.println(F("Connected to: /countryhouse/heating/currentTemperature"));
    #endif
    client.subscribe("/countryhouse/heating/targetTemperature");
    #ifdef DEBUG
      Serial.println(F("Connected to: /countryhouse/heating/targetTemperature"));
    #endif
    client.subscribe("/countryhouse/heating/targetTemperatureHiden");
    #ifdef DEBUG
      Serial.println(F("Connected to: /countryhouse/heating/targetTemperatureHiden"));
    #endif
    client.subscribe("/countryhouse/heating/hysteresis");
    #ifdef DEBUG
      Serial.println(F("Connected to: /countryhouse/heating/hysteresis"));
    #endif
  }
  return client.connected();
}

/*########################### End work with MQTT #############################*/

int16_t getDigitalSensorData(OneWire &sensor, uint8_t portNumber){
  int16_t result;
  float TSensor;
  byte bufData[9] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  // буфер данных
  if (sensor.reset() == 1)
  {
    sensor.write(CMD_SKIPROM);     // пропуск ROM
    sensor.write(CMD_RSCRATCHPAD); // команда чтения памяти датчика
    sensor.read_bytes(bufData, 9); // чтение памяти датчика, 9 байтов
    if ( OneWire::crc8(bufData, 8) == bufData[8] ) {  // проверка CRC
      // данные правильные
      TSensor =  (float)((int)bufData[0] | (((int)bufData[1]) << 8)) * 0.0625 + 0.03125;
      result = TSensor * 100;
      #ifdef DEBUG
        Serial.print(F("Current temperature on digital sensor"));
        Serial.print(portNumber);
        Serial.print(F(": "));
        Serial.println(TSensor);
      #endif
    }
    sensor.reset();                // сброс шины
    #ifdef DEBUG
      Serial.println(F("Digital sensor present, send init measurement"));
    #endif
    sensor.write(CMD_SKIPROM);
    sensor.write(CMD_CONVERTTEMP);
  }
  return result;
}

int16_t getLM35SensorData(int analogSensorPin, uint8_t portNumber){
  int16_t result;
  int reading;
  float TSensor;
  reading = analogRead(analogSensorPin);        // получаем значение с аналогового входа A0
  #ifdef DEBUG
    Serial.println("");
    Serial.print(F("RAW data from analog sensor"));
    Serial.print(portNumber);
    Serial.print(F(": "));
    Serial.println(reading);
  #endif
  TSensor = ( reading/1023.0 )*5.04*1000/10;
  result = TSensor * 100;
  #ifdef DEBUG
    Serial.print(F("Current temperature on analog sensor"));
    Serial.print(portNumber);
    Serial.print(F(": "));
    Serial.println(TSensor);
    Serial.println("");
  #endif
  delay(1000);                     // ждем секунду
  return result;
}
