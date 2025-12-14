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

// HardwareSerial GPSSerial(LPUART1); // RX, TX for GPS
// HardwareSerial Serial2(USART2); // RX, TX for GPS
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
//     GPSSerial.setRx(GPS_RX);
//     GPSSerial.setTx(GPS_TX);
//     GPSSerial.begin(9600);    
//     delay(100);
//     SPI.begin();
//     delay(100);
//     pinMode(GPS_EN, OUTPUT);
//     pinMode(GSM_EN, OUTPUT);
//     digitalWrite(GPS_EN, HIGH);
//     delay(3000);
// }

// void loop(){
//     while(GPSSerial.available()){
//         Serial.write(GPSSerial.read());
//     }
// }