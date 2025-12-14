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
// #include <math.h>

// // Use default I2C and address (0x6B)
// LSM6DSL imu;

// void setup() {
//   delay(3000);
//   Serial.begin(115200);
//   Wire.begin();

//   if (!imu.begin()) {
//     Serial.println("Failed to initialize LSM6DSL!");
//     while (1);
//   }
//   Serial.println("LSM6DSL initialized.");

//   if (!imu.setAccelFS(LSM6DSL_XL_FS_4g)) {
//     Serial.println("Failed to initialize LSM6DSL!");
//     while (1);
//   }
//   Serial.println("LSM6DSL FS Updated");

//   if (!imu.initMagnetometer()) {
//     Serial.println("Failed to initialize LIS3MDL via sensor hub.");
//     while (1);
//   }
//   Serial.println("Magnetometer initialized.");

//   uint8_t ctrl1 = imu.readRegister(LSM6DS3_ACC_GYRO_CTRL1_XL);
//   Serial.print("CTRL1_XL = 0x"); Serial.println(ctrl1, HEX);
//   imu.disableAccelerometer();
//   imu.disableGyroscope();
//   delay(5000);
  
// }

// void loop() {
//     imu.enableAccelerometer(LSM6DSL_XL_ODR_52Hz);
//     imu.enableGyroscope(LSM6DSL_G_ODR_52Hz);
//     int16_t ax_g, ay_g, az_g;

//     float sumX, sumY, sumZ;

//     for (size_t i = 0; i < 50; i++)
//     {
//         imu.readAccelerometer(ax_g, ay_g, az_g);

//         sumX = sumX + imu.convertRawAccelToG(ax_g, LSM6DSL_XL_FS_4g);
//         sumY = sumY + imu.convertRawAccelToG(ay_g, LSM6DSL_XL_FS_4g);
//         sumZ = sumZ + imu.convertRawAccelToG(az_g, LSM6DSL_XL_FS_4g);
//         delay(10);
//     }
    
//     Serial.println("Averages");
//     Serial.print(sumX/50); Serial.print(", ");
//     Serial.print(sumY/50); Serial.print(", ");
//     Serial.println(sumZ/50);

//   delay(2000);
// }
