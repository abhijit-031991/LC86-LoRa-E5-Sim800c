// #include <Arduino.h>
// #include <definitions.h>
// #include <codes.h>
// #include <Wire.h>
// #include <SPI.h>
// #include <LSM6DSL.h>
// #include <TinyGPS++.h>
// #include <TinyGsmClient.h>
// #include <PubSubClient.h>
// #include <ArduinoJson.h>
// #include <time.h>
// #include <TimeLib.h>
// #include <RadioLib.h>
// #include <STM32RTC.h>
// #include <STM32LowPower.h>
// #include <LoRaE5_SPIFlash.h>
// #include <HardwareSerial.h>

// HardwareSerial LPUART(GPS_RX, GPS_TX); // RX, TX for GPS
// HardwareSerial Serial2(PA3, PA2); // RX, TX for GPS
// TinyGPSPlus gps;
// LoRaE5_SPIFlash flash(FSS_PIN, &SPI, 8000000);
// TinyGsm modem(Serial2);
// TinyGsmClient client(modem);
// STM32RTC& rtc = STM32RTC::getInstance();
// STM32WLx radio = new STM32WLx_Module();

// static const uint32_t rfswitch_pins[] = { PA4, PA5, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC };
// static const Module::RfSwitchMode_t rfswitch_table[] = {
//   { STM32WLx::MODE_IDLE,  {LOW, LOW} },
//   { STM32WLx::MODE_RX,    {HIGH, LOW} },
//   { STM32WLx::MODE_TX_HP, {LOW, HIGH} },
//   // { STM32WLx::MODE_TX_LP, {HIGH, HIGH, HIGH} },
//   END_OF_MODE_TABLE,
// };

// int counter = 0;

// void isr(void* data){
//     // This function will be called every second
//     counter = counter + 1;
// }

// void setup(){
//     // Initialize RTC
//     rtc.setClockSource(STM32RTC::LSE_CLOCK);
//     rtc.begin(STM32RTC::HOUR_24); // Initialize RTC in 24-hour format
//     Serial.begin(115200);
//     delay(10);
//     Serial2.begin(115200);    
//     delay(100);
//     // LPUART.begin(9600);    
//     delay(100);
//     SPI.begin();
//     delay(100);
//     delay(3000);
//     Serial.println(F("Initializing modem..."));
//     pinMode(GSM_EN, OUTPUT);
//     digitalWrite(GSM_EN, HIGH);
//     delay(5000); 
//     modem.init();
//     String modemInfo = modem.getModemInfo();
//     Serial.print(F("Modem Info: "));
//     Serial.println(modemInfo);
//     Serial.print(F("Waiting for network..."));
//     if (!modem.waitForNetwork()) {
//         Serial.println(F("fail"));        
//         return;
//     }
//     Serial.println(F("success"));

    
//     if (modem.isNetworkConnected()) { 
//         Serial.println(F("Network connected")); 

//         // // MQTT Broker setup
//         // mqtt.setServer(broker, 1883);
//         // mqtt.setCallback(mqttCallback);

//         if (!modem.isGprsConnected()) {
//             Serial.println(F("GPRS not connected!"));
//             Serial.print(F("Connecting to "));
//             Serial.println(apn);
//             if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
//                 Serial.println(F(" fail"));
//                 delay(10000);
//                 return;
//             }
//         }

//         if (modem.isGprsConnected()){ 
//             Serial.println(F("GPRS reconnected")); 
//             for (int i = 0; i < 10; i++){
//                 Serial.print(F("."));
//                 delay(1000);
//             }
//         }
//     }else{
//         Serial.println(F("No Network"));
//     } 
//     modem.gprsDisconnect();

//     Serial2.end();
//     // LPUART.end();
//     SPI.end();
//     delay(100);
//     digitalWrite(GSM_EN, LOW);
//     rtc.attachSecondsInterrupt(isr);
//     LowPower.deepSleep(); // Sleep for 1 minute
// }

// void loop(){
//     if (counter >= 10){
//         Serial2.begin(115200);    
//         delay(100);
//         digitalWrite(GSM_EN, HIGH); 
//         delay(5000);
//         modem.init();
//         String modemInfo = modem.getModemInfo();
//         Serial.print(F("Modem Info: "));
//         Serial.println(modemInfo);
//         Serial.print(F("Waiting for network..."));
//         if (!modem.waitForNetwork()) {
//             Serial.println(F("fail"));        
//             return;
//         }
//         Serial.println(F("success"));

        
//         if (modem.isNetworkConnected()) { 
//             Serial.println(F("Network connected")); 

//             // // MQTT Broker setup
//             // mqtt.setServer(broker, 1883);
//             // mqtt.setCallback(mqttCallback);

//             if (!modem.isGprsConnected()) {
//                 Serial.println(F("GPRS not connected!"));
//                 Serial.print(F("Connecting to "));
//                 Serial.println(apn);
//                 if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
//                     Serial.println(F(" fail"));
//                     delay(10000);
//                     return;
//                 }
//             }

//             if (modem.isGprsConnected()){ 
//                 Serial.println(F("GPRS reconnected")); 
//                 for (int i = 0; i < 10; i++){
//                     Serial.print(F("."));
//                     delay(1000);
//                 }
//                 Serial.println();
//             }
//         }else{
//             Serial.println(F("No Network"));
//         } 
//         modem.gprsDisconnect();
//         Serial2.end();
//         digitalWrite(GSM_EN, LOW);
//     }
    

//     LowPower.deepSleep();
// }