#define TINY_GSM_MODEM_SIM800
#define ARDUINOJSON_USE_LONG_LONG 1
#define TINY_GSM_RX_BUFFER 128
#define ARCH_STM32

// Firmware Version //
const float firmwareVersion = 1.0;

// Device Information //

///// Device Definitions /////

// Device ID //
const uint16_t tag = 10001;
const char id[] = "10001";
const uint8_t devType = 210;
const char gen[] = "6.0";
const char provider[] = "airtel";

// Hardware Definitions //

#define ACC_PRESENT false

// Pin Definitions //
#define GPS_TX PC1
#define GPS_RX PC0
#define GPS_EN PB5
#define GSM_TX PA2
#define GSM_RX PA3
#define GSM_EN PB10
#define SCL_PIN PB15
#define SDA_PIN PA15
#define MOSI_PIN PA10
#define MISO_PIN PB14
#define SCK_PIN PB13
#define FSS_PIN PB9
#define ACC_INT1_PIN PB4
#define ACC_INT2_PIN PB3
#define CS_INT_PIN PA0

//GPRS credentials //
const char apn[]      = "www";
const char gprsUser[] = "";
const char gprsPass[] = "";

// MQTT Credentials //
const char* broker = "mqtt.arcturus-telemetry.in";

const char* telemetryTopic = "telemetry";
const char* statusSubTopic = "status";
const char* settingsSupTopic = "settings";
const char* imuTelemetryTopic = "imuData";

// Request Strings //
const char statusRequest[] = "{\"id\":\"10001\", \"req\":1}";
const char setttingsRequest[] = "{\"id\":\"10001\", \"req\":2}";
const char settingsResponse[] = "{\"id\":\"10001\", \"req\":3}";
const char emailAlert[] = "{\"id\":\"10001\", \"req\":4}";
const char stImuStartAlrt[] = "{\"id\":\"10001\", \"req\":5}";
const char stImuFinishAlrt[] = "{\"id\":\"10001\", \"req\":6}";
const char emailImuAlert[] = "{\"id\":\"10001\", \"req\":7}";


int locWriteMemoryAddressStorage = 1; //Location of the location data in flash memory
int accWriteMemoryAddressStorage = 10; //Location of the IMU data in flash memory
int locReadMemoryAddressStorage = 20; //Location of the location data in flash memory
int accReadMemoryAddressStorage = 30; //Location of the IMU data in flash memory

// Structs //

struct longPing{
    uint16_t ta;    
    uint16_t cnt;
    float la;
    float ln;
    uint8_t devtyp;
    bool mortality;
  }__attribute__((__packed__));  // Ping Struct - Send recent data

struct data{
    uint32_t datetime;
    uint16_t locktime;
    float lat;
    float lng;
    float hdop;
    float x;
    float y;
    float z;
    unsigned int count;
    uint16_t id;
}__attribute__((__packed__)); // Data Struct - Store GPS data/Read and Send

struct reqPing{
    uint16_t tag;
    byte request;
  }__attribute__((__packed__)); // Request Ping Struct - Send device codes for calibrartion/start sequence.

struct setttings{
    uint16_t tag;
    int gpsFrq;
    int gpsTout;
    int hdop;
    int radioFrq;
    int startHour;
    int endHour;
    time_t currentTime;
    bool scheduled;
}__attribute__((__packed__)); // Settings Struct - Store settings received from App only.

struct calibrationData{
    float lat;
    float lng;
    float hdop;
    float bat;
    float signal;
    uint16_t tag;
    bool mqtt;
    bool gprs;
    bool network;
}__attribute__((__packed__)); // Calibration Struct - Store GPS data during calibration.