#include <Arduino.h>
#include <definitions.h>
#include <codes.h>
#include <Wire.h>
#include <SPI.h>
#include <LSM6DSL.h>
#include <TinyGPS++.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <TimeLib.h>
#include <RadioLib.h>
#include <STM32RTC.h>
#include <STM32LowPower.h>
#include <LoRaE5_SPIFlash.h>

///////////////////////////////////////////////////////////////
//------------------ LIBRARY DECLARATIONS -------------------//
///////////////////////////////////////////////////////////////
// HardwareSerial SerialG(GSM_RX, GSM_TX); // RX, TX for Serial Monitor
HardwareSerial GPSSerial(LPUART1); // RX, TX for GPS
HardwareSerial Serial2(USART2);

// #define Serial Serial
// #define Serial2 Serial2

TinyGPSPlus gps;
LSM6DSL imu;
STM32RTC& rtc = STM32RTC::getInstance();
STM32WLx radio = new STM32WLx_Module();
LoRaE5_SPIFlash flash(FSS_PIN, &SPI, 8000000); // SPI pins for LoRa E5 flash
TinyGsm modem(Serial2);
TinyGsmClient client(modem);
PubSubClient  mqtt(client);

static const uint32_t rfswitch_pins[] = { PA4, PA5, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC };
static const Module::RfSwitchMode_t rfswitch_table[] = {
  { STM32WLx::MODE_IDLE,  {LOW, LOW} },
  { STM32WLx::MODE_RX,    {HIGH, LOW} },
  { STM32WLx::MODE_TX_HP, {LOW, HIGH} },
  // { STM32WLx::MODE_TX_LP, {HIGH, HIGH, HIGH} },
  END_OF_MODE_TABLE,
};
///////////////////////////////////////////////////////////////
//------------------ GLOBAL VARIABLES -----------------------//
///////////////////////////////////////////////////////////////

// Counters //
unsigned long gpsCounter = 0;
unsigned long radioCounter = 0;
unsigned long gsmCounter = 0;
unsigned long accCounter = 0;
unsigned long scheduleCounter = 0;

// Tragets //
unsigned long gpsCounterTarget; // 1 minute
unsigned long radioCounterTarget; // 15 minutes
unsigned long gsmCounterTarget; // 30 minutes
unsigned long accCounterTarget; // 5 minutes
unsigned long scheduleCounterTarget = 60; // 1 minute

// Time //
time_t currentTime = 1758986458;

// Settings Variables //
int gpsFrq = 60; // Minutes
int gpsTimeout = 120; // Seconds
int gpsHdop = 5; // Minimum HDOP
int radioFrq = 1; // Minutes 
int startHour = 6; // 24 hour format
int endHour = 10; // 24 hour format
int gsmFrq = 720; // Minutes
int accSampleSize = 50; // Number of samples to take for accelerometer
bool uploadAllData = false; // Upload all data in flash
bool radioScheduled = true; // Radio scheduled
bool isPingTime = false; // Is it time to ping

// Data variables //
float lat;
float lng;

// Flash Variables //
uint32_t writeAdd = 0; // Address to write next data    
uint32_t readAdd = 0; // Address to read next data

// State variable for RadioLib functions
int state = RADIOLIB_ERR_NONE;

// Other Variables //
uint32_t lastReconnectAttempt = 0;
unsigned long locationCount = 0;

// returns averages (in g) via references
void getAccelData(float &outAvgX, float &outAvgY, float &outAvgZ, int sampleSize) {
    imu.setAccelFS(LSM6DSL_XL_FS_4g);
    imu.enableAccelerometer(LSM6DSL_XL_ODR_52Hz);
    imu.enableGyroscope(LSM6DSL_G_ODR_52Hz);

    long sumX = 0, sumY = 0, sumZ = 0;
    int16_t ax_g, ay_g, az_g;
    for (int i = 0; i < sampleSize; ++i) {
        imu.readAccelerometer(ax_g, ay_g, az_g);
        sumX += imu.convertRawAccelToG(ax_g, LSM6DSL_XL_FS_4g) * 1000000.0; // scale to avoid FP accumulation issues
        sumY += imu.convertRawAccelToG(ay_g, LSM6DSL_XL_FS_4g) * 1000000.0;
        sumZ += imu.convertRawAccelToG(az_g, LSM6DSL_XL_FS_4g) * 1000000.0;
        delay(10);
    }

    // convert back to float g values
    outAvgX = (float)( (double)sumX / (double)sampleSize / 1000000.0 );
    outAvgY = (float)( (double)sumY / (double)sampleSize / 1000000.0 );
    outAvgZ = (float)( (double)sumZ / (double)sampleSize / 1000000.0 );

    imu.disableAccelerometer();
    imu.disableGyroscope();
}

void initPeripherals(){
    Serial.begin(115200);
    delay(10);
    Serial2.begin(115200);
    delay(10);
    GPSSerial.setRx(GPS_RX);
    GPSSerial.setTx(GPS_TX);
    GPSSerial.begin(9600);
    delay(3000);
    SPI.begin();
    delay(10);
    Wire.begin();
    delay(10);
}

void deInitPeripherals(){
    Serial.end();
    delay(10);
    Serial2.end();
    delay(10);
    GPSSerial.end();
    delay(10);
    SPI.end();
    delay(10);
    Wire.end();
    delay(10);
}

void calculateTargets() {
    gpsCounterTarget   = gpsFrq * 60;   // Convert minutes to seconds
    radioCounterTarget = radioFrq * 60; // Convert minutes to seconds
    gsmCounterTarget   = gsmFrq * 60;   // Convert minutes to seconds (fixed: was radioFrq in your code)

    char buffer[100]; // make sure it's big enough

    // Format the results into a string
    sprintf(buffer,
            "GPS Target: %ld | Radio Target: %ld | GSM Target: %ld",
            gpsCounterTarget, radioCounterTarget, gsmCounterTarget);

    // Print the string to Serial
    Serial.println(buffer);
}

void sendloRaMsg(byte code){
    reqPing rPing;
    rPing.tag = tag;
    rPing.request = code;
    state = radio.transmit((uint8_t*)&rPing, sizeof(rPing));
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("Ping sent successfully.");
    } else {
        Serial.print("Failed to send ping. Error code: ");
        Serial.println(state);
    }
    radio.sleep(); // Put radio to sleep to save power
}

void isr(void* data){
    gpsCounter = gpsCounter + 1;
    radioCounter = radioCounter + 1;
    gsmCounter = gsmCounter + 1;
    accCounter = accCounter + 1;
    scheduleCounter = scheduleCounter + 1;
}

void getlocation(bool setup){
    Serial.println("Getting GPS location...");
    data dat;
    float startHdop = 10.00;
    unsigned long timeout;
    if (setup){
        timeout = 30*60*1000; // 30 minutes in milliseconds
    }else{
        timeout = gpsTimeout * 1000; // Convert seconds to milliseconds
    }

    Serial.println(timeout);
    GPSSerial.begin(9600);
    digitalWrite(GPS_EN, HIGH);
    unsigned long start = millis();
    bool goodFixFound = false;
    
    while (millis() - start < timeout && !goodFixFound) {
        // Serial.println("Waiting for GPS fix...");
        while (GPSSerial.available() > 0) {
            if(gps.encode(GPSSerial.read())){
                if (gps.location.isValid()) {
                    Serial.print("Location Age: ");
                    Serial.println(gps.location.age());
                    Serial.print("Satellites: ");
                    Serial.println(gps.satellites.value());
                    Serial.print("HDOP: ");
                    Serial.println(gps.hdop.hdop());
                    if (gps.hdop.hdop() != 0.00) {
                        startHdop = gps.hdop.hdop();
                    }                    
                    Serial.print("Latitude: ");
                    Serial.println(gps.location.lat(), 6);
                    Serial.print("Longitude: ");
                    Serial.println(gps.location.lng(), 6);
                    
                    // Check if we have a good fix
                    if (startHdop < (double)gpsHdop && 
                        gps.location.age() < 1000 && 
                        gps.time.age() < 1000 && 
                        millis() - start > 3000) {
                        goodFixFound = true;
                        Serial.println("Good GPS fix acquired!");
                        break; // Break out of inner loop
                    }
                }
            }else{
            //   Serial.println("Acquiring... ");
            }
        }
        // Small delay to prevent overwhelming the processor
        delay(10);
    }
    
    digitalWrite(GPS_EN, LOW);
    GPSSerial.end();

    if (goodFixFound) {
        Serial.println("GPS location acquired successfully");
    } else {
        Serial.println("GPS timeout - no good fix found");
    } 
    if (setup && goodFixFound)
    {
        setTime(gps.time.hour(),gps.time.minute(),gps.time.second(),gps.date.day (), gps.date.month (),gps.date.year());
        currentTime = now();
        Serial.println(currentTime);
        rtc.setEpoch(currentTime);
        Serial.print("RTC Time Set: ");Serial.println(rtc.getEpoch());
    }    
    
    if(!setup){
        if (ACC_PRESENT)
        {
            float avgX, avgY, avgZ;

            getAccelData(avgX, avgY, avgZ, accSampleSize);

            Serial.println("Averages");
            Serial.print(avgX); Serial.print(", ");
            Serial.print(avgY); Serial.print(", ");
            Serial.println(avgZ);

            dat.x = avgX;
            dat.y = avgY;
            dat.z = avgZ;

            Serial.print(dat.x); Serial.print(", ");
            Serial.print(dat.y); Serial.print(", ");
            Serial.println(dat.z);
        }       

        locationCount = locationCount + 1;
        dat.id = tag;
        dat.lat = gps.location.isValid() ? gps.location.lat() : 0;
        dat.lng = gps.location.isValid() ? gps.location.lng() : 0;
        dat.hdop = gps.hdop.hdop();
        dat.locktime = (millis() - start) / 1000; // Lock
        dat.count = locationCount;
        // Set timestamp
        if (gps.time.isValid() && gps.date.isValid()) {
            setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), 
                    gps.date.day(), gps.date.month(), gps.date.year());
            dat.datetime = (uint32_t)now();
        } else {
            dat.datetime = rtc.getEpoch(); // Use system time if GPS time invalid
        }
        // === FIXED FLASH WRITE SECTION ===
        Serial.println(F("Writing to flash..."));
        
        flash.powerUp();
        delay(10); // Give flash time to wake up
        
        // Check if we need to erase a new sector
        // Erase at sector boundaries (typically every 4096 bytes)
        uint32_t sectorSize = flash.getSectorSize();
        if (writeAdd % sectorSize == 0) {
            Serial.print(F("Erasing sector at address: 0x"));
            Serial.println(writeAdd, HEX);
            
            uint8_t eraseResult = flash.eraseSector(writeAdd);
            if (eraseResult != FLASH_OK) {
                Serial.print(F("Sector erase FAILED! Error code: "));
                Serial.println(eraseResult);
                flash.powerDown();
                return;
            }
            Serial.println(F("Sector erased successfully"));
        }
        
        // Write the struct
        Serial.print(F("Writing struct at address: 0x"));
        Serial.println(writeAdd, HEX);
        Serial.print(F("Struct size: "));
        Serial.println(sizeof(dat));
        
        uint8_t writeResult = flash.writeStruct(writeAdd, dat, true); // Enable verify
        
        if (writeResult == FLASH_OK) {
            Serial.println(F("Data written to flash successfully."));
            
            // Verify by reading back
            data verification;
            uint8_t readResult = flash.readStruct(writeAdd, verification);
            if (readResult == FLASH_OK) {
                Serial.println(F("Verification read successful"));
                Serial.print(F("Verify lat: ")); Serial.println(verification.lat, 8);
                Serial.print(F("Verify lng: ")); Serial.println(verification.lng, 8);
                Serial.print(F("Verify time: ")); Serial.println(verification.datetime);
                Serial.print(F("Verify count: ")); Serial.println(verification.count);
                
                // Only increment address if write was truly successful
                writeAdd += sizeof(dat);
            } else {
                Serial.print(F("Verification read FAILED! Error: "));
                Serial.println(readResult);
            }
        } else {
            Serial.print(F("Failed to write data to flash! Error code: "));
            Serial.println(writeResult);
            
            // Print error code meanings
            switch(writeResult) {
                case FLASH_ERROR_TIMEOUT:
                    Serial.println(F("  -> TIMEOUT"));
                    break;
                case FLASH_ERROR_WRITE_PROTECTED:
                    Serial.println(F("  -> WRITE PROTECTED"));
                    break;
                case FLASH_ERROR_INVALID_ADDRESS:
                    Serial.println(F("  -> INVALID ADDRESS"));
                    break;
                case FLASH_ERROR_NOT_ERASED:
                    Serial.println(F("  -> NOT ERASED"));
                    break;
                case FLASH_ERROR_VERIFY_FAILED:
                    Serial.println(F("  -> VERIFY FAILED"));
                    break;
                case FLASH_ERROR_NO_RESPONSE:
                    Serial.println(F("  -> NO RESPONSE"));
                    break;
                default:
                    Serial.println(F("  -> UNKNOWN ERROR"));
                    break;
            }
        }
        
        flash.powerDown();
        
        Serial.print("Total records stored: "); 
        Serial.println(writeAdd / sizeof(dat));
        Serial.print("Write Address: 0x"); 
        Serial.println(writeAdd, HEX);
        Serial.flush();
    }
}

boolean mqttConnect() {
    Serial.print(F("Connecting to "));
    Serial.print(broker);

    // Connect to MQTT Broker
    // boolean status = mqtt.connect("GsmClientTest");

    // Or, if you want to authenticate MQTT:
    boolean status = mqtt.connect(id);
    mqtt.setKeepAlive(20000);
    mqtt.setSocketTimeout(20000);
    mqtt.setBufferSize(512);
    if (status == false) {
        Serial.println(F(" fail"));
        return false;
    }
    Serial.println(F(" success"));
    if (mqtt.subscribe(settingsSupTopic))
    {
        Serial.println(F("Set Subbed"));
    }else{
        Serial.println(F("Set Subbed"));
    }
    return mqtt.connected();
}

void mqttCallback(char* topic, byte* payload, unsigned int len) {
    Serial.print(F("Message arrived ["));
    Serial.print(topic);
    Serial.print(F("]: "));
    Serial.println();

    Serial.println(len);
    Serial.print(F("PLD:"));   
    char x[len];
    strncpy(x, (char*)payload, len);
    delay(3000);
    String top = (char*)topic;
    Serial.println(x);
    Serial.println(F("DONE"));

    if (top == "settings")
    {
        StaticJsonDocument<200> doc;

        DeserializationError error = deserializeJson(doc, x);

        if (error) {
            Serial.print(F("deserializeJson() failed: "));
            Serial.println(error.f_str());
            return;
        }else{
            bool x = doc["NEW"];
            if (x == true)
            {
                if (doc.containsKey("GFRQ")) {
                    gpsFrq = doc["GFRQ"];
                }
                if (doc.containsKey("HDOP")) {
                    gpsHdop = doc["HDOP"];
                }
                if (doc.containsKey("TFRQ")) {
                    gsmFrq = doc["TFRQ"];
                }
                if (doc.containsKey("GTO")) {
                    gpsTimeout = doc["GTO"];
                }
                if (doc.containsKey("RFRQ")) {
                    radioFrq = doc["RFRQ"];
                }
                if (doc.containsKey("STH")) {
                    startHour = doc["STH"];
                }
                if (doc.containsKey("ETH")) {
                    endHour = doc["ETH"];
                }
                if (doc.containsKey("RSCH")) {
                    radioScheduled = doc["RSCH"];
                }
                if (doc.containsKey("DNALL")) {
                    uploadAllData = doc["DNALL"];
                }


                Serial.println(gpsHdop);
                Serial.println(gpsFrq);
                Serial.println(gpsTimeout);
                Serial.println(gsmFrq);
                Serial.println(radioFrq);
                Serial.println(startHour);
                Serial.println(endHour);
                Serial.println(radioScheduled);
                calculateTargets();

                if (!mqtt.connected())
                {
                mqttConnect();
                if(mqtt.publish(telemetryTopic, settingsResponse)){
                Serial.println(F("Reset Settings"));
                }
                }else{
                if(mqtt.publish(telemetryTopic, settingsResponse)){
                Serial.println(F("Reset Settings"));
                }
                }
            }else{
                Serial.println(F("No New Settings"));
            }
            if (doc.containsKey("DNALL"))
            {
                bool y = doc["DNALL"];  
                if (y)
                {
                uploadAllData = true;
                }
            }           
        }
    }
}

void networkInit(){
    modem.init();
    String modemInfo = modem.getModemInfo();
    Serial.print(F("Modem Info: "));
    Serial.println(modemInfo);

    Serial.print(F("Waiting for network..."));
    if (!modem.waitForNetwork()) {
        Serial.println(F("fail"));        
        return;
    }
    Serial.println(F("success"));

    if (modem.isNetworkConnected()) { 
        Serial.println(F("Network connected")); 

        // MQTT Broker setup
        mqtt.setServer(broker, 1883);
        mqtt.setCallback(mqttCallback);

        if (!modem.isGprsConnected()) {
            Serial.println(F("GPRS not connected!"));
            Serial.print(F("Connecting to "));
            Serial.println(apn);
            if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
                Serial.println(F(" fail"));
                delay(10000);
                return;
            }
        }

        if (modem.isGprsConnected()){ 
            Serial.println(F("GPRS reconnected")); 
        }
        if (!mqtt.connected()) {
            Serial.println(F("=== MQTT NOT CONNECTED ==="));
            // Reconnect every 10 seconds
            uint32_t t = millis();
            if (t - lastReconnectAttempt > 10000L) {
            lastReconnectAttempt = t;
            if (mqttConnect()){
                lastReconnectAttempt = 0;
            }
            }
            delay(100);
            return;
        }
    }else{
        Serial.println(F("No Network"));
    }  
}

void postMetaAttributes(){
    Serial.println(F("Posting Meta Attributes"));
    char a[100];
    currentTime = rtc.getEpoch();
    Serial.println(currentTime);
    StaticJsonDocument<120> doc;
    doc[F("Battery")] = modem.getBattPercent();
    doc[F("Signal")] = modem.getSignalQuality();
    doc[F("Data")] = locationCount;
    doc[F("id")] = (String)tag;
    doc[F("tts")] = currentTime;
    serializeJson(doc, a);  
    Serial.println(a);
    if (!mqtt.connected())
    {
        Serial.println(F("Mqtt Disconnected.. Retrying"));
        mqttConnect();
        if(mqtt.publish(telemetryTopic, (char*)a)){
        Serial.println(F("Posted Meta Attributes"));
        } 
    }else{
        if(mqtt.publish(telemetryTopic, (char*)a)){
        Serial.println(F("Posted Meta Attributes"));
        }
    } 
}

void syncSettings() {
    static unsigned long lastAttempt = 0; 
    unsigned long currentMillis = millis();

    if (!mqtt.connected()) {
        Serial.println(F("Mqtt Disconnected.. Retrying"));
        mqttConnect();  
        mqtt.setKeepAlive(10000);
        lastAttempt = millis(); // reset timer when reconnecting
    } else {
        mqtt.subscribe(settingsSupTopic);

        // Request settings
        if (mqtt.publish(telemetryTopic, setttingsRequest)) {
            Serial.println(F("Requested Settings"));
            lastAttempt = millis(); // start timer here

            // non-blocking wait for up to 3000 ms
            while (millis() - lastAttempt <= 3000) {
                mqtt.loop();
                delay(10); // small delay to avoid busy loop  
            }
        }

        // Send alert
        if (mqtt.publish(telemetryTopic, emailAlert)) {
            Serial.println(F("Sent Alert"));
        }
    }
}

// IMPROVED locationReadSend function:
void locationReadSend(unsigned long startAddress) {
    data dat;
    char a[512];
    StaticJsonDocument<512> doc;

    flash.powerUp();
    delay(10);

    uint8_t res = flash.readStruct(startAddress, dat);
    Serial.print(F("Reading from address 0x"));
    Serial.print(startAddress, HEX);
    Serial.print(F(" => "));
    
    if (res != FLASH_OK) {
        Serial.print(F("FAIL (code "));
        Serial.print(res);
        Serial.println(F(")"));
        flash.powerDown();
        return;
    }
    Serial.println(F("OK"));

    // Debug values
    Serial.print(F("datetime: ")); Serial.println(dat.datetime);
    Serial.print(F("lat: ")); Serial.println(dat.lat, 6);
    Serial.print(F("lng: ")); Serial.println(dat.lng, 6);
    Serial.print(F("Count: ")); Serial.println(dat.count);
    Serial.print(F("hdop: ")); Serial.println(dat.hdop, 2);
    Serial.print(F("locktime: ")); Serial.println(dat.locktime);    
    Serial.print(F("id: ")); Serial.println(dat.id);
    Serial.print(F("ax: ")); Serial.println(dat.x, 3);
    Serial.print(F("ay: ")); Serial.println(dat.y, 3);
    Serial.print(F("az: ")); Serial.println(dat.z, 3);
    Serial.flush();

    // Fill JSON
    doc.clear();
    doc[F("ts")]    = (unsigned long long)dat.datetime;
    doc[F("Lat")]   = dat.lat;
    doc[F("Lng")]   = dat.lng;
    doc[F("hdop")]  = dat.hdop;
    doc[F("LT")]    = dat.locktime;
    doc[F("Count")] = dat.count;
    doc[F("id")]    = tag;
    doc[F("ax")]     = dat.x;
    doc[F("ay")]     = dat.y;
    doc[F("az")]     = dat.z;

    // Serialize
    size_t written = serializeJson(doc, a, sizeof(a));
    if (written == 0) {
        Serial.println(F("serializeJson failed!"));
        flash.powerDown();
        return;
    }
    a[min((size_t)(sizeof(a) - 1), written)] = '\0';
    Serial.print(F("Payload: "));
    Serial.println(a);
    Serial.print(F("Payload size: "));
    Serial.println(written);

    flash.powerDown();

    // Ensure MQTT is connected
    if (!mqtt.connected()) {
        Serial.println(F("MQTT disconnected, reconnecting..."));
        if (!mqttConnect()) {
            Serial.println(F("Failed to reconnect!"));
            return;
        }
    }

    // Publish with verification
    Serial.println(F("Publishing to MQTT..."));
    bool ok = mqtt.publish(telemetryTopic, (char*)a);
    
    if (ok) {
        Serial.println(F("MQTT PUBLISH SUCCESS"));
    } else {
        Serial.println(F("MQTT PUBLISH FAILED"));
        Serial.print(F("MQTT State: "));
        Serial.println(mqtt.state());
    }
    
    // Process MQTT messages
    for (int i = 0; i < 3; i++) {
        mqtt.loop();
        delay(100);
    }
}

void printBytesHex(const void* ptr, size_t len) {
  const uint8_t* p = (const uint8_t*)ptr;
  for (size_t i = 0; i < len; ++i) {
    uint8_t b = p[i];
    if (b < 0x10) Serial.print('0');    // pad
    Serial.print(b, HEX);                // prints uppercase hex (no leading zero)
    if (i + 1 < len) Serial.print(' ');
  }
  Serial.println();
}

void initSequence(){
    calibrationData calData;

    sendloRaMsg(CALIBRATION_BEGIN);
    delay(1000);
    if (flash.readStruct(0, calData))
    {
        sendloRaMsg(FLASH_SUCCESS);
        delay(1000);
    }else{
        sendloRaMsg(FLASH_ERROR);
        delay(1000);
    }
    digitalWrite(GSM_EN, HIGH);
    delay(5000);
    modem.init();
    Serial2.println(F("AT+CNETLIGHT=1"));
    unsigned long starts = millis();
    while (millis() - starts < 3000) {   // run for 3 seconds
        if (Serial2.available()) {
            Serial.println(Serial2.readString());
        }
    }
    Serial.print(F("Waiting for network..."));
    if (!modem.waitForNetwork()) {
        Serial.println(F("fail"));        
        return;
    }
    Serial.println(F("success"));
    if (modem.isNetworkConnected())
    {
        calData.network = true;
        calData.bat = modem.getBattPercent();
        calData.signal = modem.getSignalQuality();

        Serial.println(modem.getGSMDateTime(DATE_FULL));

        // MQTT Broker setup
        mqtt.setServer(broker, 1883);
        mqtt.setCallback(mqttCallback);

        if (!modem.isGprsConnected()) {
            Serial.println(F("GPRS not connected!"));
            Serial.print(F("Connecting to "));
            Serial.println(apn);
            if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
                Serial.println(F(" fail"));
                delay(10000);
                return;
            }
        }

        if (modem.isGprsConnected()){ 
            calData.gprs = true;
            if(mqttConnect()){
                if (mqtt.connected())
                {
                    calData.mqtt = true;
                }else{
                    calData.mqtt = false;
                }   
                sendloRaMsg(GSM_TEST_SUCCESS);
                delay(1000);
            }else{
                calData.mqtt = false;
                sendloRaMsg(GSM_TEST_ERROR);
                delay(1000);
            }   
        }else{
            calData.gprs = false;
            calData.mqtt = false;
        }  
    }else{
        calData.network = false;
        calData.gprs = false;
        calData.mqtt = false;
    }
    Serial2.println(F("AT+CNETLIGHT=0"));
    starts = millis();
    while (millis() - starts < 3000) {   // run for 3 seconds
        if (Serial2.available()) {
            Serial.println(Serial2.readString());
        }
    }

    Serial2.println(F("AT&W"));
    starts = millis();
    while (millis() - starts < 3000) {   // run for 3 seconds
        if (Serial2.available()) {
            Serial.println(Serial2.readString());
        }
    }

    delay(1000);
    
    digitalWrite(GSM_EN, LOW);
    delay(500);
    getlocation(true);
    if (gps.location.isValid())
    {
        calData.lat = gps.location.lat();
        calData.lng = gps.location.lng();
        calData.hdop = gps.hdop.hdop();
        sendloRaMsg(GPS_SUCCESS);
        delay(1000);
    }else{
        calData.lat = 0;
        calData.lng = 0;
        calData.hdop = 99.99;
        sendloRaMsg(GPS_ERROR);
        delay(1000);
    }   
    calData.tag = tag;
    state = radio.transmit((uint8_t*)&calData, sizeof(calData));
    sendloRaMsg(CALIBRATION_END);
    delay(1000);

    uint8_t buffer[32];
    size_t length = sizeof(buffer);
    setttings setPacket;
    reqPing reqPacket;
    int packetLength;

    unsigned long start = millis();    
    while (millis() - start < 300000)
    {
        state = radio.receive(buffer, length);
        if (state == RADIOLIB_ERR_NONE) {
            Serial.print(F("[LoRa] Bytes received: "));
            packetLength = radio.getPacketLength();
            Serial.println(packetLength);
            printBytesHex(&buffer, sizeof(buffer));

            if (packetLength == sizeof(setPacket))
            {
                memcpy(&setPacket, buffer, sizeof(setPacket));
                printBytesHex(&setPacket, sizeof(setPacket));
                if (setPacket.tag == tag)
                {
                    gpsFrq = setPacket.gpsFrq;
                    gpsTimeout = setPacket.gpsTout;
                    gpsHdop = setPacket.hdop;
                    radioFrq = setPacket.radioFrq;
                    radioScheduled = setPacket.scheduled;
                    startHour = setPacket.startHour;
                    endHour = setPacket.endHour;
                    currentTime = setPacket.currentTime;
                    rtc.setEpoch(currentTime);
                    calculateTargets();
                    sendloRaMsg(SETTINGS_UPDATED);
                    delay(1000);
                }
            } 
            if (packetLength == sizeof(reqPacket))
            {
                memcpy(&reqPacket, buffer, sizeof(reqPacket));
                printBytesHex(&reqPacket, sizeof(reqPacket));
                Serial.println(reqPacket.tag);
                Serial.println(reqPacket.request);
                if (reqPacket.tag == tag && reqPacket.request == FINISH_CALIBRATION)
                {
                    break;
                }
            }
        } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
            Serial.println(F("[LoRa] Receive timeout!"));
        } else {
            Serial.print(F("[LoRa] Receive failed, code "));
            Serial.println(state);
        }    
         
    }
    
}

void receive(unsigned long timeout){

    uint8_t buffer[28];
    size_t length = sizeof(buffer);
    reqPing reqPacket;
    setttings settingsPacket;
    longPing pingLong;
    int packetLength;

    unsigned long start = millis();    
    while (millis() - start < timeout)
    {
      state = radio.receive(buffer, length);
      if (state == RADIOLIB_ERR_NONE) {
        Serial.print(F("[LoRa] Bytes received: "));
        packetLength = radio.getPacketLength();
        Serial.println(packetLength);
      } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
        Serial.println(F("[LoRa] Receive timeout!"));
      } else {
        Serial.print(F("[LoRa] Receive failed, code "));
        Serial.println(state);
      }
    }
    
    if (packetLength == sizeof(reqPacket))
    {
      memcpy(&reqPacket, buffer, sizeof(reqPacket));
      if (reqPacket.tag == tag && reqPacket.request == SIMPLE_PING_ACK) // Check if ping request is for this device
      {
        pingLong.ta = tag;
        pingLong.cnt = writeAdd / sizeof(data); // Number of stored records
        pingLong.la = lat;
        pingLong.ln = lng;
        pingLong.devtyp = devType; // Device type
        pingLong.mortality = false; // Mortality flag

        state = radio.transmit((uint8_t*)&pingLong, sizeof(pingLong));
        if (state == RADIOLIB_ERR_NONE) {
            Serial.println("Ping response sent successfully.");
        } else {
            Serial.print("Failed to send ping response. Error code: ");
            Serial.println(state);
        } 
      }
      
    }
}

void setup() {
    delay(3000);
    initPeripherals();
    Serial.print(F("Device ID : ")); Serial.println(tag);
    Serial.println(F("Initializing..."));
    pinMode(GPS_EN, OUTPUT);
    pinMode(GSM_EN, OUTPUT);
    digitalWrite(GSM_EN, HIGH);
    delay(500);
    digitalWrite(GSM_EN, LOW);
    
    // Initialize RTC
    rtc.setClockSource(STM32RTC::LSE_CLOCK);
    rtc.begin(STM32RTC::HOUR_24); // Initialize RTC in 24-hour format

    // Initialize radio
    radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
    state = radio.begin(867.0, 125.0, 12, 5, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 22, 8, 1.6, false);
    if (state != RADIOLIB_ERR_NONE) {
      Serial.print("Radio init failed: ");
      Serial.println(state);
      while (true);
    }else {
      Serial.println("Radio initialized successfully.");
    }

    state = radio.transmit((uint8_t*)id, sizeof(id));
    if (state != RADIOLIB_ERR_NONE) {
      Serial.print("Radio transmit failed: ");
      Serial.println(state);
    } else {
      Serial.println("Radio transmit successful.");
    }

    radio.sleep(); // Put radio to sleep to save power

    // Flash Setup //
    if (flash.begin()) {
      Serial.println("SPI Flash initialized successfully.");
      flash.printChipInfo();
    } else {
      Serial.println("Failed to initialize SPI Flash.");
    }

    flash.powerDown(); // Power down flash to save power
    delay(50);

    if(ACC_PRESENT){
        // IMU Setup //
        Wire.begin(); 
        if (!imu.begin()) { 
            Serial.println(F("Failed to initialize LSM6DSL!")); while (1); 
        } Serial.println(F("LSM6DSL initialized.")); 
        if (!imu.setAccelConfig(LSM6DSL_XL_ODR_208Hz,LSM6DSL_XL_FS_8g)) {
            Serial.println(F("Failed to initialize LSM6DSL!")); while (1); 
        } Serial.println(F("LSM6DSL initialized."));
        Serial.print("CTRL1_XL after: 0x"); 
        Serial.println(imu.readRegister(0x10), HEX); 
        imu.disableGyroscope(); 
        imu.disableAccelerometer();
    }    

    initSequence();
    rtc.setEpoch(currentTime); // Set RTC time
    calculateTargets();

    deInitPeripherals();
    rtc.attachSecondsInterrupt(isr);
    LowPower.deepSleep();
}

void loop() {
    if (scheduleCounter >= scheduleCounterTarget && radioScheduled)
    {
        int hour = rtc.getHours() + 6;
        if (hour >= startHour && hour < endHour)
        {
            isPingTime = true;
        }else{
            isPingTime = false;
        }  
        scheduleCounter = 0;  
    }
    
    if (gpsCounter >= gpsCounterTarget)
    {
        initPeripherals();
        gpsCounter = 0;
        getlocation(false);
        deInitPeripherals();
    }
    if (gsmCounter >= gsmCounterTarget)
    {

        initPeripherals();
        digitalWrite(GSM_EN, HIGH);
        delay(5000);
        gsmCounter = 0;
        networkInit();
        syncSettings();
        postMetaAttributes();
        delay(1000);
        if (uploadAllData)
        {
            unsigned long tempAdd = 0;
            while (tempAdd < writeAdd)
            {
                locationReadSend(tempAdd);
                tempAdd = tempAdd + 36;
            }            
            uploadAllData = false;
        }else{
            while (readAdd < writeAdd)
            {
                locationReadSend(readAdd);
                readAdd = readAdd + 36;

            }
        }
        // Ensure all MQTT messages are sent
        for (int i = 0; i < 5; i++) {
            mqtt.loop();
            delay(100);
        }
        
        delay(500);
        deInitPeripherals();
        digitalWrite(GSM_EN, LOW);
        delay(500);
    }
    if (isPingTime && radioCounter >= radioCounterTarget)
    {
        initPeripherals();
        radioCounter = 0;
        sendloRaMsg(SIMPLE_PING);
        receive(3000);
        deInitPeripherals();
    }

    LowPower.deepSleep();

}