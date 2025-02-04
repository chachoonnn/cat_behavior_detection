#include "Arduino.h"
#include "model.h"
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <LoRa.h>
#include <avr/sleep.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>

#include <Arduino_LSM6DSOX.h>
#include <TinyGPS++.h>

//***********************************************************
// Pins
//***********************************************************
#define PIN_SD_SS     PIN_PC3
#define PIN_RF_SS     PIN_PD0
#define PIN_RF_RST    PIN_PD1
#define PIN_RF_DIO1   PIN_PD2
#define PIN_RF_IRQ    PIN_PD3

#define PIN_EN_LORA   PIN_PA7
#define PIN_EN_GNSS   PIN_PD5
#define PIN_EN_SD     PIN_PD6
#define PIN_DEBUG_RX  PIN_PA0
#define PIN_DEBUG_TX  PIN_PA1
#define PIN_SD_DETECT PIN_PA4

#define PIN_LED1      PIN_PA6
#define PIN_LED2      PIN_PA5
#define PIN_GNSS_RX   PIN_PF0
#define PIN_GNSS_TX   PIN_PF1

#define PIN_BAT_PROBE PIN_PD7
#define LORA_FREQUENCY 434E6  // Set LoRa frequency (match gateway)
// For your gateway code, these two are important
#define GATEWAY_ADDRESS  0x00
#define DEVICE_ADDRESS   0x01

// If you decide to use a new custom packet type:
#define PKT_TYPE_STRING  0x05
//***********************************************************
// Macros
//***********************************************************
#define SHORT_BLINK1()  do {              \
    digitalWrite(PIN_LED1, LOW);         \
    delay(10);                            \
    digitalWrite(PIN_LED1, HIGH);          \
} while (0)

#define SHORT_BLINK2()  do {              \
    digitalWrite(PIN_LED2, LOW);         \
    delay(10);                            \
    digitalWrite(PIN_LED2, HIGH);          \
} while (0)

//***********************************************************
// Globals
//***********************************************************
#define debug_serial Serial
#define gnss_serial  Serial2

// Real-time clock counter (seconds since device turned on)
volatile uint32_t g_secondsSinceStart = 0;

// SD variables
Sd2Card card;
SdVolume volume;
SdFile root;
SdFile myFile;
char linebuf[128];
char fileName[15];

// For geofence bounding box
bool g_enableGeofence = false;  // if false => skip geofence check
double g_minLat = 0.0, g_maxLat = 0.0;
double g_minLng = 0.0, g_maxLng = 0.0;
bool g_gpsFix = false;          // have we ever had a valid fix?
static int g_gpsWaitCount = 0;
// For time zone offset
int g_timeZoneHours = 0;        // e.g. 7 => +7 hours from UTC

// For storing last known valid GPS time
bool  g_hasLastGpsTime = false; // have we ever had a valid fix?
uint8_t g_lastHour = 0, g_lastMinute = 0, g_lastSecond = 0;

// Store how many seconds the RTC has advanced since the last valid GPS time
// Then we can add that offset to the last known time
uint32_t g_lastRtcCount = 0;

struct ImuSample {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
};

// We want 200 samples for a 3-s window at 40 Hz
static const int WINDOW_SIZE = 120;
static const int SLIDING_STEP = 20;  // every 0.5 s at 40 Hz

// Ring buffer to store the most recent 200 samples
ImuSample ringBuffer[WINDOW_SIZE];

// This points to where the next sample will go
int ringHead = 0;

// Keep track of how many samples have arrived total (ever).
uint32_t sampleCounter = 0;
uint32_t lastPredictionSample = 0;
//***********************************************************
// Forward declarations
//***********************************************************
void pullup_all_pins(void);
void init_gpio(void);
bool init_sdcard(void);
void create_file(void);
void writeFile(const char *fileName, const char *data);
bool insideSquare(double lat, double lng);

void init_imu(void);
String get_imu_data(void);

TinyGPSPlus gps;
String get_gps_data(void);

ISR(RTC_CNT_vect);
void init_RTC(void);
String getUpTimeMillisString(void);

void init_lora();
void sendLoRaString(const char *msg);
//***********************************************************
// Pull-up all pins to reduce power (optional)
//***********************************************************
void pullup_all_pins() {
    for (uint8_t i = 0; i < 8; i++) {
        //    *((uint8_t *)&PORTA + 0x10 + i) = PORT_PULLUPEN_bm;
        //    *((uint8_t *)&PORTC + 0x10 + i) = PORT_PULLUPEN_bm;
        //    *((uint8_t *)&PORTD + 0x10 + i) = PORT_PULLUPEN_bm;
        //    *((uint8_t *)&PORTF + 0x10 + i) = PORT_PULLUPEN_bm;
        if ((VPORTA.DIR & (1 << i)) == 0) *((uint8_t *)&PORTA + 0x10 + i) = PORT_PULLUPEN_bm;
        if ((VPORTC.DIR & (1 << i)) == 0) *((uint8_t *)&PORTC + 0x10 + i) = PORT_PULLUPEN_bm;
        if ((VPORTD.DIR & (1 << i)) == 0) *((uint8_t *)&PORTD + 0x10 + i) = PORT_PULLUPEN_bm;
        if ((VPORTF.DIR & (1 << i)) == 0) *((uint8_t *)&PORTF + 0x10 + i) = PORT_PULLUPEN_bm;
    }
}


//***********************************************************
// GPIO
//***********************************************************
void init_gpio() {
    pinMode(PIN_LED1, OUTPUT);
    pinMode(PIN_LED2, OUTPUT);
    pinMode(PIN_BAT_PROBE, INPUT);
    pinMode(PIN_EN_SD, OUTPUT);
    pinMode(PIN_EN_GNSS, OUTPUT);
    pinMode(PIN_EN_LORA, OUTPUT);
    pinMode(PIN_RF_SS, OUTPUT);
    pinMode(PIN_SD_SS, OUTPUT);
    pinMode(PIN_RF_RST, OUTPUT);
    pinMode(PIN_RF_IRQ, INPUT);

    digitalWrite(PIN_EN_SD, HIGH);
    digitalWrite(PIN_EN_GNSS, HIGH);
    digitalWrite(PIN_EN_LORA, HIGH);
    digitalWrite(PIN_RF_SS, HIGH);
    digitalWrite(PIN_SD_SS, HIGH);
    digitalWrite(PIN_LED1, LOW);
    digitalWrite(PIN_LED2, LOW);
}

//***********************************************************
// SD
//***********************************************************
char* read_line(SdFile& file, char* buf, uint8_t maxlen) {
  uint8_t pos = 0;
  while (true) {
    int c = file.read();
    if (c == -1) {
      if (pos == 0) return NULL;
      else break;
    }
    if (c == '\n' || c == '\r') break;
    buf[pos++] = c;
    if (pos >= maxlen - 1) break;
  }
  buf[pos] = '\0';
  return buf;
}

bool init_sdcard() {
    debug_serial.print("\nInitializing SD card...");
    digitalWrite(PIN_EN_SD, LOW);

    if (!card.init(SPI_HALF_SPEED, PIN_SD_SS)) {
        debug_serial.println("initialization failed. Check wiring and card.");
        return false;
    } else {
        debug_serial.println("Card is present.");
    }

    debug_serial.print("Card type: ");
    switch (card.type()) {
        case SD_CARD_TYPE_SD1:  debug_serial.println("SD1");  break;
        case SD_CARD_TYPE_SD2:  debug_serial.println("SD2");  break;
        case SD_CARD_TYPE_SDHC: debug_serial.println("SDHC"); break;
        default:                debug_serial.println("Unknown");
    }

    if (!volume.init(card)) {
        debug_serial.println("Could not find FAT16/FAT32 partition.");
        return false;
    }
    // ... (print volume info, etc.)

    if (!root.openRoot(volume)) {
        return false;
    }

    // We'll track how many bounding box lines we find
    bool foundMinLat = false, foundMaxLat = false;
    bool foundMinLng = false, foundMaxLng = false;

    SdFile config_file;
    if (config_file.open(&root, "CONFIG.TXT", O_READ)) {
        while (true) {
            char* s = read_line(config_file, linebuf, sizeof(linebuf));
            if (s == NULL) break; // no more lines

            // Example line: "geof_min_lat   14.0"
            // or "time_zone   7"
            // we can parse with sscanf or strncmp
            /****************************************************************************
            * Example fix: Use atof() for geofence config lines
            ****************************************************************************/

            // Inside your config‑reading loop:

            if (strncmp(s, "geof_min_lat", strlen("geof_min_lat")) == 0) {
                // s might look like: "geof_min_lat 13.0" or "geof_min_lat    13.0"
                // We'll skip up to "geof_min_lat" plus any whitespace, then parse with atof.
                char* p = s + strlen("geof_min_lat");  // move pointer just past "geof_min_lat"

                // skip spaces, tabs, or '=' if you have lines like "geof_min_lat = 13.0"
                while (*p == ' ' || *p == '\t' || *p == '=') {
                    p++;
                }

                double val = atof(p); // parse double from the remainder of the line
                g_minLat = val;
                foundMinLat = true;
                debug_serial.print("[CFG] g_minLat = ");
                debug_serial.println(val, 6);
            }
            else if (strncmp(s, "geof_max_lat", strlen("geof_max_lat")) == 0) {
                char* p = s + strlen("geof_max_lat");
                while (*p == ' ' || *p == '\t' || *p == '=') {
                    p++;
                }

                double val = atof(p);
                g_maxLat = val;
                foundMaxLat = true;
                debug_serial.print("[CFG] g_maxLat = ");
                debug_serial.println(val, 6);
            }
            else if (strncmp(s, "geof_min_lng", strlen("geof_min_lng")) == 0) {
                char* p = s + strlen("geof_min_lng");
                while (*p == ' ' || *p == '\t' || *p == '=') {
                    p++;
                }

                double val = atof(p);
                g_minLng = val;
                foundMinLng = true;
                debug_serial.print("[CFG] g_minLng = ");
                debug_serial.println(val, 6);
            }
            else if (strncmp(s, "geof_max_lng", strlen("geof_max_lng")) == 0) {
                char* p = s + strlen("geof_max_lng");
                while (*p == ' ' || *p == '\t' || *p == '=') {
                    p++;
                }

                double val = atof(p);
                g_maxLng = val;
                foundMaxLng = true;
                debug_serial.print("[CFG] g_maxLng = ");
                debug_serial.println(val, 6);
            }
            else if (strncmp(s, "time_zone", strlen("time_zone")) == 0) {
                int tz;
                if (sscanf(s, "time_zone %d", &tz) == 1) {
                    g_timeZoneHours = tz;
                    debug_serial.print("[CFG] time_zone = ");
                    debug_serial.println(tz);
                }
            }
            else {
                // Some other config line
                debug_serial.println(s);
            }
            
      }
      config_file.close();

      // If we found all 4 lines, enable geofence
      if (foundMinLat && foundMaxLat && foundMinLng && foundMaxLng) {
          g_enableGeofence = true;
          debug_serial.println("[CFG] Geofencing is ENABLED (square bounding box).");
      } else {
          g_enableGeofence = false;
          debug_serial.println("[CFG] Geofencing is DISABLED (missing bounding box lines).");
      }

    } else {
      debug_serial.println("No CONFIG.TXT found");
    }

    root.close();
    // digitalWrite(PIN_EN_SD, HIGH);

    return true;
}


void create_file() {
    if (!card.init(SPI_HALF_SPEED, PIN_SD_SS)) {
        debug_serial.println("initialization failed. Check wiring and card.");
        return false;
    } else {
        debug_serial.println("Card is present.");
    }

    if (!volume.init(card)) {
        debug_serial.println("Could not find FAT16/FAT32 partition.");
        return false;
    }

    // Try opening the root directory
    if (!root.openRoot(volume)) {
        debug_serial.println("[create_file] ERROR: Failed to open root directory.");
        debug_serial.println("             Make sure SD card is mounted and volume.init() succeeded.");
        return;
    }

    int fileIndex = 1;

    // find a new filename data_1.txt, data_2.txt, ...
    while (true) {
        sprintf(fileName, "data_%d.txt", fileIndex);
        SdFile tempFile;
        if (!tempFile.open(&root, fileName, O_READ)) {
            // file doesn't exist => can use this name
            debug_serial.print( "Got this file name: " );
            debug_serial.println( fileName );
            break;
        }
        tempFile.close();
        fileIndex++;
    }

    // Attempt to create the new file
    if (!myFile.open(&root, fileName, O_CREAT | O_WRITE | O_TRUNC)) {
        debug_serial.print("[create_file] ERROR: Could NOT create file: ");
        debug_serial.println(fileName);
        debug_serial.println("             Possibly SD permissions/format issue or the file is locked.");
        return;
    }

    debug_serial.print("[create_file] Created new file: ");
    debug_serial.println(fileName);

    // sample CSV header
    myFile.println("Time,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ");
    myFile.close();

    root.close();
}

void writeFile(const char *fileName, const char *data) {
    // Try opening the root directory
    if (!root.openRoot(volume)) {
        if (!card.init(SPI_HALF_SPEED, PIN_SD_SS)) {
            debug_serial.println("initialization failed. Check wiring and card.");
            SHORT_BLINK2();

            return false;
        } else {
            debug_serial.println("Card is present.");
        }

        if (!volume.init(card)) {
            debug_serial.println("Could not find FAT16/FAT32 partition.");
            SHORT_BLINK2();

            return false;
        }
        // Try opening the root directory
        if (!root.openRoot(volume)) {
            debug_serial.println("[writeFile] ERROR: Failed to open root directory.");
            debug_serial.println("             Make sure SD card is mounted and volume.init() succeeded.");
            SHORT_BLINK2();

            return;
        }
    }

    // Use O_APPEND to add data at the end of the file
    if (!myFile.open(&root, fileName, O_CREAT | O_WRITE | O_APPEND)) {
        debug_serial.print("[writeFile] ERROR: Could NOT open file for append: ");
        debug_serial.println(fileName);
        debug_serial.println("             Possibly SD is locked, missing, or the file is in use.");
        SHORT_BLINK2();
        return;
    }

    debug_serial.print("[writeFile] Write at ");
    debug_serial.print(fileName);
    myFile.println(data);
    myFile.close();
    SHORT_BLINK1();

    root.close();
}

//***********************************************************
// IMU
//***********************************************************
void init_imu() {
    if (!IMU.begin()) {
        debug_serial.println("Failed to initialize IMU!");
        SHORT_BLINK1();
        while (1);
    }

    debug_serial.print("Accelerometer sample rate = ");
    debug_serial.print(IMU.accelerationSampleRate());
    debug_serial.println(" Hz");
    debug_serial.println("Acceleration in g's (X, Y, Z)");
}

String get_imu_data() {
    float ac_x = 0.0, ac_y = 0.0, ac_z = 0.0;
    float gy_x = 0.0, gy_y = 0.0, gy_z = 0.0;

    bool hasAccel = false;
    bool hasGyro = false;

    // Check if acceleration is available
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(ac_x, ac_y, ac_z);
        hasAccel = true;
    }
    // Check if gyroscope is available
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gy_x, gy_y, gy_z);
        hasGyro = true;
    }

    // If we don't have data, store empty string instead
    String strAcX = hasAccel ? String(ac_x) : "";
    String strAcY = hasAccel ? String(ac_y) : "";
    String strAcZ = hasAccel ? String(ac_z) : "";
    
    String strGyX = hasGyro ? String(gy_x) : "";
    String strGyY = hasGyro ? String(gy_y) : "";
    String strGyZ = hasGyro ? String(gy_z) : "";

    // Return CSV chunk "acX,acY,acZ,gyX,gyY,gyZ"
    String imuData = strAcX + "," +
                     strAcY + "," +
                     strAcZ + "," +
                     strGyX + "," +
                     strGyY + "," +
                     strGyZ;

    return imuData;
}

//***********************************************************
// GPS
//***********************************************************

String get_gps_data() {
    // Read all bytes currently available on GNSS serial
    while (gnss_serial.available()) {
        char c = gnss_serial.read();
        gps.encode(c);
    }

    // We'll build CSV: "GpsTime,Latitude,Longitude"
    String timeStr = "";
    String latStr  = "";
    String lonStr  = "";
    double lat = 0, lon = 0;
    if (gps.location.isUpdated()) {
        lat = gps.location.lat();
        lon = gps.location.lng();
        latStr = String(lat, 6);
        lonStr = String(lon, 6);
        if (insideSquare( lat, lon )) {
            // send flag to LoRa gateway
            debug_serial.println( "Inside geofencing" );
        }
        else {
            debug_serial.println( "OUTSIDE!! geofencing" );
        }
        if (gps.time.isValid()) {
            // Format hh:mm:ss
            char buf[10];
            sprintf(buf, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
            timeStr = String(buf);
        }
    }
    else {
        debug_serial.print( "gps.location.isValid(): " );     
        debug_serial.println( gps.location.isValid() );     
    }

    // CSV chunk: "GpsTime,lat,lon"
    String gpsData = timeStr + "," + latStr + "," + lonStr;
    debug_serial.println( gpsData );
    return gpsData;
}

bool insideSquare(double lat, double lng) {
    if (!g_enableGeofence) {
        return false;  // or skip logic
    }
    // A point is inside the square if:
    //   g_minLat <= lat <= g_maxLat
    //   g_minLng <= lng <= g_maxLng
    if (lat >= g_minLat && lat <= g_maxLat &&
        lng >= g_minLng && lng <= g_maxLng) {
        return true;
    }
    return false;
}

//***********************************************************
// RTC - Internal Timer
//***********************************************************
ISR(RTC_CNT_vect)
{
    // Clear the interrupt flag
    RTC.INTFLAGS = RTC_OVF_bm;
    // Increment our global variable once per second
    g_secondsSinceStart++;
}

void init_RTC()
{
    RTC.CLKSEL = RTC_CLKSEL_INT32K_gc; 
    RTC.CNT = 0;
    RTC.PER = 31; // Overflow once per second? (Actually ~1ms if 32 ticks... adjust as needed)
    RTC.INTCTRL = RTC_OVF_bm;
    RTC.CTRLA = RTC_PRESCALER_DIV1_gc | RTC_RTCEN_bm;
    sei();
}

//***********************************************************
// Convert the up-time from seconds to milliseconds (string)
//***********************************************************
String getUpTimeMillisString() {
    uint32_t ms;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        ms = g_secondsSinceStart;
    }
    return String(ms);
}

//***********************************************************
// behavior classification
//***********************************************************
void computeWindowFeatures(float* buf, int length, 
                           float &meanVal, float &stdVal, float &minVal, float &maxVal) {
  if (length <= 0) {
    meanVal = stdVal = minVal = maxVal = 0.0f;
    return;
  }

  // Compute mean
  float sum = 0.0f;
  minVal = buf[0];
  maxVal = buf[0];
  for (int i = 0; i < length; i++) {
    sum += buf[i];
    if (buf[i] < minVal) minVal = buf[i];
    if (buf[i] > maxVal) maxVal = buf[i];
  }
  meanVal = sum / length;

  // Compute std
  float sumSq = 0.0f;
  for (int i = 0; i < length; i++) {
    float diff = buf[i] - meanVal;
    sumSq += diff * diff;
  }
  stdVal = sqrt(sumSq / length);
}

extern void score(double * input, double * output); // from your generated score.c

void classifyLatestWindow() 
{
    // 1) Reconstruct the 200 samples in chronological order
    //    The most recent sample is ringHead - 1
    //    The oldest is ringHead (modded)
    ImuSample windowData[WINDOW_SIZE];

    // We'll figure out the index of the oldest sample
    // Because ringHead points to where the *next* sample goes
    // The oldest sample is ringHead itself (if we have 200 fully).
    int startIndex = ringHead; 
    for (int i = 0; i < WINDOW_SIZE; i++) {
        int bufIndex = (startIndex + i) % WINDOW_SIZE;
        windowData[i] = ringBuffer[bufIndex];
    }

    // 2) Now compute your features from windowData[0..199]
    //    Make arrays for each axis so you can compute mean/std/min/max easily
    float axArray[WINDOW_SIZE], ayArray[WINDOW_SIZE], azArray[WINDOW_SIZE];
    float gxArray[WINDOW_SIZE], gyArray[WINDOW_SIZE], gzArray[WINDOW_SIZE];

    for (int i = 0; i < WINDOW_SIZE; i++) {
        axArray[i] = windowData[i].ax;
        ayArray[i] = windowData[i].ay;
        azArray[i] = windowData[i].az;
        gxArray[i] = windowData[i].gx;
        gyArray[i] = windowData[i].gy;
        gzArray[i] = windowData[i].gz;
    }
    float axMean, axStd, axMin, axMax;
    computeWindowFeatures(axArray, WINDOW_SIZE, axMean, axStd, axMin, axMax);

    float ayMean, ayStd, ayMin, ayMax;
    computeWindowFeatures(ayArray, WINDOW_SIZE, ayMean, ayStd, ayMin, ayMax);

    float azMean, azStd, azMin, azMax;
    computeWindowFeatures(azArray, WINDOW_SIZE, azMean, azStd, azMin, azMax);

    float gxMean, gxStd, gxMin, gxMax;
    computeWindowFeatures(gxArray, WINDOW_SIZE, gxMean, gxStd, gxMin, gxMax);

    float gyMean, gyStd, gyMin, gyMax;
    computeWindowFeatures(gyArray, WINDOW_SIZE, gyMean, gyStd, gyMin, gyMax);

    float gzMean, gzStd, gzMin, gzMax;
    computeWindowFeatures(gzArray, WINDOW_SIZE, gzMean, gzStd, gzMin, gzMax);
  // 3) Create the 24D input array in same order as your Python code
  //    Then do the scaling: scaled_value = (value - mean[i]) / std[i]

    double input[24];
    input[0]  = axMean;
    input[1]  = axStd;
    input[2]  = axMin;
    input[3]  = axMax;
    input[4]  = ayMean;
    input[5]  = ayStd;
    input[6]  = ayMin;
    input[7]  = ayMax;
    input[8]  = azMean;
    input[9]  = azStd;
    input[10] = azMin;
    input[11] = azMax;
    input[12] = gxMean;
    input[13] = gxStd;
    input[14] = gxMin;
    input[15] = gxMax;
    input[16] = gyMean;
    input[17] = gyStd;
    input[18] = gyMin;
    input[19] = gyMax;
    input[20] = gzMean;
    input[21] = gzStd;
    input[22] = gzMin;
    input[23] = gzMax;

  // scale them with your hardcoded means/stds
    // const char* behaviors[3] = {"lay down", "stand", "sit"};    
    const char* behaviors[3] = {"lay down","sit", "stand", };    
    // const char* behaviors[3] = { "stand", "sit", "lay down"};    
    // const char* behaviors[3] = { "stand", "sit", "lay down"};    
    static const double means[24] = {
        6.37140849e-03 , 7.61203792e-02, -9.89806174e-02 , 1.12806892e-01,
        -7.14863176e-01 , 8.60156097e-02, -8.31035535e-01 ,-5.95821967e-01,
        5.43291430e-01 , 1.05850135e-01 , 3.98492462e-01 , 6.87936109e-01,
        6.43611471e-01 , 2.29076341e+01, -3.12973169e+01 , 3.29208668e+01,
        -6.22653624e-01 , 2.35146466e+01, -3.37848403e+01 , 3.23195729e+01,
        -4.52030123e-02 , 2.24754662e+01, -3.10248134e+01 , 3.21552028e+01
    };

    static const double stds[24] = {
        0.18892042  ,0.09119041  ,0.22929603 , 0.24994066  ,0.2372586   ,0.11192168,
        0.28942015  ,0.29364816  ,0.354913   , 0.12425447  ,0.40079765  ,0.39388221,
        19.71975359 ,29.5395564  ,47.40253715, 50.88442037 ,18.83502771 ,25.79482671,
        46.67216502 ,49.13733964 ,17.90931593, 25.61447513 ,37.93129893 ,50.43662942,
    };


    for (int i = 0; i < 24; i++) {
        // scaled_value = (value - mean[i]) / std[i]
        input[i] = (input[i] - means[i]) / stds[i];
    }
    // 4) Call the model
    double output[3];
    score(input, output);

    // 5) Find which class is best
    int bestClass = 0;
    double bestVal = output[0];
    for (int i = 1; i < 3; i++) {
        if (output[i] > bestVal) {
        bestVal = output[i];
        bestClass = i;
        }
    }
    const char* predicted = behaviors[bestClass];

    // Print or log the result
    // Serial.print("Predicted Behavior = ");
    // Serial.println(predicted);
    if (predicted == "stand") {
        digitalWrite(PIN_LED1, LOW);
        digitalWrite(PIN_LED2, HIGH);
    }
    else if (predicted == "sit") {
        digitalWrite(PIN_LED1, LOW);
        digitalWrite(PIN_LED2, LOW);
    }
    else if (predicted == "lay down") {
        digitalWrite(PIN_LED1, HIGH);
        digitalWrite(PIN_LED2, LOW);
    }

    // ... you can store or send the result over LoRa, etc.
    sendLoRaString(predicted);
}
//***********************************************************
// LoRa
//***********************************************************
void init_lora() {
    Serial.println("innit LoRa...");
    pinMode(PIN_EN_LORA, OUTPUT);
    digitalWrite(PIN_EN_LORA, LOW);
    delay(100);
    LoRa.setPins(PIN_RF_SS, PIN_RF_RST, PIN_RF_DIO1);
    if (!LoRa.begin(434E6)) {
        Serial.println("LoRa init failed!");
        while (true) {
            SHORT_BLINK1();
            delay(250);
        }
    }
    LoRa.setSpreadingFactor(12);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(8);
    LoRa.setTxPower(15);
    LoRa.beginPacket();
    LoRa.print("START: Smart Cat Collar");
    LoRa.endPacket();
    Serial.println("LoRa init Done");
}

void sendLoRaString(const char *msg) {
    Serial.print("sending... ");Serial.println(msg);
    LoRa.beginPacket();
    LoRa.print(msg);
    LoRa.endPacket();
    Serial.print("Sent: ");Serial.println(msg);
}
//***********************************************************
// Main
//***********************************************************
void setup() {
// void setup1() {
    pullup_all_pins();
    init_gpio();
    debug_serial.begin(115200);
    delay(1000);

    debug_serial.println("System starting...");
    // Wire.swap(0);
    // Wire.begin();
    gnss_serial.begin(9600);
    SPI.swap(SPI1_SWAP_DEFAULT);
    delay(1000);
    init_lora();


    // // SD
    // SD.begin(PIN_SD_SS);
    // if (!init_sdcard()) {
    //   debug_serial.println("SD card initialization failed!");
    // }

    // IMU
    init_imu();

    // RTC
    // init_RTC();
    digitalWrite(PIN_EN_GNSS, LOW);

    g_enableGeofence = true;
    g_minLat = 13.9229;
    g_maxLat = 13.923695;
    g_minLng = 100.57059;
    g_maxLng = 100.57145;

    ringHead = 0;
    sampleCounter = 0;
    lastPredictionSample = 0;
}

void loop() {
    // 1) Read one IMU sample
    float ax = 0, ay = 0, az = 0;
    float gx = 0, gy = 0, gz = 0;
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
        IMU.readAcceleration(ax, ay, az);
        IMU.readGyroscope(gx, gy, gz);
    }

    // 2) Store in ring buffer
    ringBuffer[ringHead].ax = ax;
    ringBuffer[ringHead].ay = ay;
    ringBuffer[ringHead].az = az;
    ringBuffer[ringHead].gx = gx;
    ringBuffer[ringHead].gy = gy;
    ringBuffer[ringHead].gz = gz;

    // Move ringHead forward by 1 (wrap around with modulo)
    ringHead = (ringHead + 1) % WINDOW_SIZE;

    // 3) Increment sampleCounter
    sampleCounter++;

    // 4) Check if we have at least one full window of 200 samples
    //    and if 25 new samples have arrived since last classification
    if ( (sampleCounter >= WINDOW_SIZE) 
        && ((sampleCounter - lastPredictionSample) >= SLIDING_STEP) )
    {
        // We do classification on the LAST 200 samples in the ring buffer
        classifyLatestWindow();
        // Update lastPredictionSample
        lastPredictionSample = sampleCounter;
        g_gpsWaitCount++;
    }
    // 1) Continuously read from GNSS serial
    while (gnss_serial.available()) {
        char c = gnss_serial.read();
        gps.encode(c);
    }

    // 2) If we have a new updated location/time, print it
    //    (TinyGPS++ sets these flags when a new full NMEA sentence has been parsed)
    
    if (gps.location.isUpdated()) {
        g_gpsFix = true;
        debug_serial.println();
        double lat = gps.location.lat();
        double lng = gps.location.lng();
        char latStr[15];
        char lngStr[15];
        char geofencingStr[15];
        dtostrf(lat, 10, 6, latStr); 
        dtostrf(lng, 10, 6, lngStr);
        debug_serial.print("Lat: ");
        debug_serial.print(lat, 6);
        debug_serial.print("  Lon: ");
        debug_serial.print(lng, 6);

        char gpsData[50];
        // debug_serial.println( gpsData );
        // Send GPS data over LoRa
        if (insideSquare( lat, lng )) {
            // send flag to LoRa gateway
            // debug_serial.println( "Inside geofencing" );
            strcpy(geofencingStr, "inside");
        }
        else {
            // debug_serial.println( "OUTSIDE!! geofencing" );
            strcpy(geofencingStr, "outside");
        }
        // sprintf(gpsData, "Lat: %s, Lon: %s, Geofencing: %s", latStr, lngStr, geofencingStr);
        sprintf(gpsData, "Lat: %s, Lon: %s, Geofencing: %s", latStr, lngStr, geofencingStr);
        if (g_gpsWaitCount > 10) {
            sendLoRaString( gpsData );
            g_gpsWaitCount = 0;
        }
    }
  // ... any other loop code ...
  delay(25); //40 hz
}
