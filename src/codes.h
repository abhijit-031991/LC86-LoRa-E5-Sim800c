#define INIT_BEGIN (byte)74
#define INIT_END (byte)105
#define INIT_END_WIPE (byte)106

#define DATA_DOWNLOAD_NEW (byte)82
#define DATA_DOWNLOAD_ALL (byte)92
#define DATA_DOWNLOAD_END (byte)68

#define WIPE_MEMORY (byte)128

#define SETTINGS_UPDATED (byte)83

#define SIMPLE_PING (byte)99
#define SIMPLE_PING_ACK (byte)101


////// Calibration codes //////

#define CALIBRATION_BEGIN (byte)1
#define CALIBRATION_END (byte)2


#define GPS_CALIBRATION (byte)10
#define GPS_CALIBRATION_END (byte)11
#define GPS_ERROR (byte)12
#define GPS_SUCCESS (byte)13

#define GSM_TEST (byte)15
#define GSM_TEST_END (byte)16
#define GSM_TEST_ERROR (byte)17
#define GSM_TEST_SUCCESS (byte)18

#define FLASH_DIAGNOSTICS (byte)21
#define FLASH_DIAGNOSTICS_END (byte)22
#define FLASH_ERROR (byte)23
#define FLASH_SUCCESS (byte)24

#define FINISH_CALIBRATION (byte)30 // Sent from App to device to end calibration mode
