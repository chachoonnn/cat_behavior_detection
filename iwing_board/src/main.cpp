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

//***********************************************************
// Forward declarations
//***********************************************************
void pullup_all_pins(void);
void init_gpio(void);
bool test_sdcard(void);
void createFile(void);
void writeFile(const char *fileName, const char *data);


void init_imu(void);
String get_imu_data(void);

TinyGPSPlus gps;
String get_gps_data(void);

ISR(RTC_CNT_vect);
void initRTC(void);
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

bool test_sdcard() {
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
    debug_serial.print("Clusters:          ");
    debug_serial.println(volume.clusterCount());
    debug_serial.print("Blocks x Cluster:  ");
    debug_serial.println(volume.blocksPerCluster());
    debug_serial.print("Total Blocks:      ");
    debug_serial.println(volume.blocksPerCluster() * volume.clusterCount());
    debug_serial.println();

    uint32_t volumesize;
    debug_serial.print("Volume type is:    FAT");
    debug_serial.println(volume.fatType(), DEC);

    volumesize = volume.blocksPerCluster();
    volumesize *= volume.clusterCount();
    volumesize /= 2; // 512 bytes/block => 2 blocks = 1KB
    debug_serial.print("Volume size (Kb):  ");
    debug_serial.println(volumesize);
    debug_serial.print("Volume size (Mb):  ");
    volumesize /= 1024;
    debug_serial.println(volumesize);
    debug_serial.print("Volume size (Gb):  ");
    debug_serial.println((float)volumesize / 1024.0);

    if (!root.openRoot(volume)) {
        return false;
    }
    
    // Example read from CONFIG.TXT
    SdFile config_file;
    if (config_file.open(&root, "CONFIG.TXT", O_READ)) {
      while (true) {
          char* s = read_line(config_file, linebuf, sizeof(linebuf));
          if (s == NULL) break;
          debug_serial.print(s);
          debug_serial.print("\n");
      }
      config_file.close();
    } else {
      debug_serial.println("No CONFIG.TXT found");
    }

    return true;
}

//***********************************************************
// CHANGED: createFile()
//***********************************************************
void createFile() {
     if (!volume.init(card)) {
        debug_serial.println("Could not find FAT16/FAT32 partition.");
        return false;
    }

    // Try opening the root directory
    if (!root.openRoot(volume)) {
        debug_serial.println("[createFile] ERROR: Failed to open root directory.");
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
        debug_serial.print("[createFile] ERROR: Could NOT create file: ");
        debug_serial.println(fileName);
        debug_serial.println("             Possibly SD permissions/format issue or the file is locked.");
        return;
    }

    debug_serial.print("[createFile] Created new file: ");
    debug_serial.println(fileName);

    // sample CSV header
    myFile.println("Time,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ");
    myFile.close();
}

//***********************************************************
// CHANGED: writeFile()
//***********************************************************
void writeFile(const char *fileName, const char *data) {
    // Try opening the root directory
    if (!root.openRoot(volume)) {
        debug_serial.println("[writeFile] ERROR: Failed to open root directory.");
        debug_serial.println("             Make sure volume.init() succeeded and card is still powered.");
        return;
    }

    // Use O_APPEND to add data at the end of the file
    if (!myFile.open(&root, fileName, O_CREAT | O_WRITE | O_APPEND)) {
        debug_serial.print("[writeFile] ERROR: Could NOT open file for append: ");
        debug_serial.println(fileName);
        debug_serial.println("             Possibly SD is locked, missing, or the file is in use.");
        return;
    }

    myFile.print(data);
    myFile.close();
    debug_serial.print("[writeFile] Write complete to ");
    debug_serial.println(fileName);
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

    if (gps.time.isUpdated()) {
        // Format hh:mm:ss
        char buf[10];
        sprintf(buf, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
        timeStr = String(buf);
    }
    if (gps.location.isUpdated()) {
        latStr = String(gps.location.lat(), 6);
        lonStr = String(gps.location.lng(), 6);
    }

    // CSV chunk: "GpsTime,lat,lon"
    String gpsData = timeStr + "," + latStr + "," + lonStr;
    return gpsData;
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

void initRTC()
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
// SETUP
//***********************************************************
void setup() {
    pullup_all_pins();
    init_gpio();
    debug_serial.begin(115200);
    delay(1000);

    debug_serial.println("System starting...");
    Wire.swap(0);
    Wire.begin();
    gnss_serial.begin(9600);
    SPI.swap(SPI1_SWAP_DEFAULT);

    // RTC
    initRTC();

    // SD
    SD.begin(PIN_SD_SS);
    if (!test_sdcard()) {
      debug_serial.println("SD card initialization failed!");
    }

    // IMU
    init_imu();

    // Create a new data file
    createFile();

    // Power GNSS on (active-low)
    digitalWrite(PIN_EN_GNSS, LOW);
}

//***********************************************************
// LOOP
//***********************************************************
void loop() {
    // 1) Acquire GPS data (HH:MM:SS or blank)
    String gpsData = get_gps_data();

    // 2) Acquire IMU data (or blank if not available)
    String imuData = get_imu_data();

    // 3) Get device uptime in milliseconds (as string)
    String uptimeMs = getUpTimeMillisString();

    // 4) Build CSV line (adding 3 empty columns for MagX,MagY,MagZ)
    String csvLine = uptimeMs + "," + 
                     imuData + "," +
                     ",,," +   // blank placeholders for Mag
                     gpsData + "\r\n";

    // 5) Append to file named "data_1.txt" or "data_2.txt", etc.
    writeFile(fileName, csvLine.c_str());

    debug_serial.print("Data: ");
    debug_serial.println(csvLine);

    delay(500);
}
