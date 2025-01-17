#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
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

//***********************************************************
// Macros
//***********************************************************
#define SHORT_BLINK1()  do {              \
    digitalWrite(PIN_LED1, HIGH);         \
    delay(10);                            \
    digitalWrite(PIN_LED1, LOW);          \
} while (0)

#define SHORT_BLINK2()  do {              \
    digitalWrite(PIN_LED2, HIGH);         \
    delay(10);                            \
    digitalWrite(PIN_LED2, LOW);          \
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

// For time zone offset
int g_timeZoneHours = 0;        // e.g. 7 => +7 hours from UTC

// For storing last known valid GPS time
bool  g_hasLastGpsTime = false; // have we ever had a valid fix?
uint8_t g_lastHour = 0, g_lastMinute = 0, g_lastSecond = 0;

// Store how many seconds the RTC has advanced since the last valid GPS time
// Then we can add that offset to the last known time
uint32_t g_lastRtcCount = 0;

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

//***********************************************************
// Pull-up all pins to reduce power (optional)
//***********************************************************
void pullup_all_pins() {
  // ...
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
            debug_serial.println("[writeFile] ERROR: Failed to open root directory.");
            debug_serial.println("             Make sure SD card is mounted and volume.init() succeeded.");
            return;
        }
    }

    // Use O_APPEND to add data at the end of the file
    if (!myFile.open(&root, fileName, O_CREAT | O_WRITE | O_APPEND)) {
        debug_serial.print("[writeFile] ERROR: Could NOT open file for append: ");
        debug_serial.println(fileName);
        debug_serial.println("             Possibly SD is locked, missing, or the file is in use.");
        return;
    }

    debug_serial.print("[writeFile] Write at ");
    debug_serial.print(fileName);
    myFile.println(data);
    myFile.close();

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
// GET DATA FROM IMU INTO SD CARD
//***********************************************************

// void setup() {
void setup1() {
    pullup_all_pins();
    init_gpio();
    debug_serial.begin(115200);
    delay(1000);

    debug_serial.println("System starting...");
    Wire.swap(0);
    Wire.begin();
    gnss_serial.begin(9600);
    SPI.swap(SPI1_SWAP_DEFAULT);


    // SD
    SD.begin(PIN_SD_SS);
    if (!init_sdcard()) {
      debug_serial.println("SD card initialization failed!");
    }

    // IMU
    init_imu();

    // Create a new data file
    create_file();

    // RTC
    init_RTC();
}

// void loop() {
void loop1() {

    // 1) Acquire IMU data (or blank if not available)
    String imuData = get_imu_data();

    // 2) Get device uptime in milliseconds (as string)
    String uptimeMs = getUpTimeMillisString();

    // 3) Build CSV line (adding 3 empty columns for MagX,MagY,MagZ)
    String csvLine = uptimeMs + "," + 
                     imuData + "," +
                     ",," +   // blank placeholders for Mag
                     "\r\n";

    // 4) Append to file named "data_1.txt" or "data_2.txt", etc.
    writeFile(fileName, csvLine.c_str());

    debug_serial.print("Data: ");
    debug_serial.println(csvLine);

    delay(500);
}

//***********************************************************
// READ GPS DATA
//***********************************************************

void setup() {
// void setup2() {
    init_gpio();

    // Start serial for debug
    debug_serial.begin(115200);
    delay(1000);
    debug_serial.println("GPS Starting...");

    // If your board supports Wire.swap(0) or similar, do it if needed
    // Wire.swap(0);
    // Wire.begin();

    // Initialize GNSS Serial 
    // (check your board's docs for the correct pins or usage of Serial2)
    gnss_serial.begin(9600);

    // Blink once to show we're alive
    SHORT_BLINK1();

    // Power GNSS on (active-low)
    digitalWrite(PIN_EN_GNSS, LOW);

    g_enableGeofence = true;
    g_minLat = 13.0;
    g_maxLat = 15.0;
    g_minLng = 99.0;
    g_maxLng = 101.0;
}

void loop() {
// void loop2() {
    // 1) Continuously read from GNSS serial
    while (gnss_serial.available()) {
        char c = gnss_serial.read();
        gps.encode(c);
    }

    // 2) If we have a new updated location/time, print it
    //    (TinyGPS++ sets these flags when a new full NMEA sentence has been parsed)
    if (gps.location.isUpdated()) {
        debug_serial.println();
        double lat = gps.location.lat();
        double lng = gps.location.lng();
        debug_serial.print("Lat: ");
        debug_serial.print(lat, 6);
        debug_serial.print("  Lon: ");
        debug_serial.print(lng, 6);

        // Print time if available
        if (gps.time.isValid()) {
            debug_serial.print("  Time (UTC): ");
            debug_serial.print(gps.time.hour());
            debug_serial.print(":");
            debug_serial.print(gps.time.minute());
            debug_serial.print(":");
            debug_serial.println(gps.time.second());
        }
        if (insideSquare( lat, lng )) {
            // send flag to LoRa gateway
            debug_serial.println( "Inside geofencing" );
        }
        else {
            debug_serial.println( "OUTSIDE!! geofencing" );
        }

        // Blink LED2 to indicate we got new data
        SHORT_BLINK2();
    }

    // Optional small delay
    delay(100);
}