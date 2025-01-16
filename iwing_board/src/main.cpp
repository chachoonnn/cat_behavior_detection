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
#include <RTClib.h>

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
#define PIN_GNSS_RX    PIN_PF0
#define PIN_GNSS_TX    PIN_PF1

#define PIN_BAT_PROBE PIN_PD7

#define SHORT_BLINK1()  do { digitalWrite(PIN_LED1, HIGH); delay(10); digitalWrite(PIN_LED1, LOW); } while (0);
#define SHORT_BLINK2()  do { digitalWrite(PIN_LED2, HIGH); delay(10); digitalWrite(PIN_LED1, LOW); } while (0);


#define debug_serial Serial
#define gnss_serial Serial2

/************************************************************
 * Enable pull-up on all pins to lower power
 * taken from https://www.avrfreaks.net/comment/3209046#comment-3209046
 */
void pullup_all_pins() {
    //for (uint8_t i = 0; i < 8; i++) {
    //    *((uint8_t *)&PORTA + 0x10 + i) = PORT_PULLUPEN_bm;
    //    *((uint8_t *)&PORTB + 0x10 + i) = PORT_PULLUPEN_bm;
    //    *((uint8_t *)&PORTC + 0x10 + i) = PORT_PULLUPEN_bm;
    //    *((uint8_t *)&PORTD + 0x10 + i) = PORT_PULLUPEN_bm;
    //    *((uint8_t *)&PORTF + 0x10 + i) = PORT_PULLUPEN_bm;
    //}
}

/************************************************************
 *
 */
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

/************************************************************
 *      SD 
 */

Sd2Card card;
SdVolume volume;
SdFile root;
SdFile myFile;

void init_card() {
  SdVolume volume;
  card.init(SPI_HALF_SPEED, PIN_SD_SS);
}

char* read_line(SdFile& file, char* buf, uint8_t maxlen) {
  uint8_t pos = 0;

  while (true) {
    int c = file.read();
    if (c == -1)
      if (pos == 0)
        return NULL;
      else
        break;
    if (c == '\n' || c=='\r') break;
    buf[pos++] = c;
    if (pos >= maxlen-1) break;
  }
  buf[pos] = '\0';
  return buf;
}

bool test_sdcard() {
    debug_serial.print("\nInitializing SD card...");
    digitalWrite(PIN_EN_SD, LOW);

    // we'll use the initialization code from the utility libraries
    // since we're just testing if the card is working!
    if (!card.init(SPI_HALF_SPEED, PIN_SD_SS)) {
        debug_serial.println("initialization failed. Things to check:");
        debug_serial.println("* is a card inserted?");
        debug_serial.println("* is your wiring correct?");
        debug_serial.println("* did you change the chipSelect pin to match your shield or module?");
        return false;
    } else {
        debug_serial.println("Wiring is correct and a card is present.");
    }

    // print the type of card
    debug_serial.println();
    debug_serial.print("Card type:         ");
    switch (card.type()) {
        case SD_CARD_TYPE_SD1:
            debug_serial.println("SD1");
            break;
        case SD_CARD_TYPE_SD2:
            debug_serial.println("SD2");
            break;
        case SD_CARD_TYPE_SDHC:
            debug_serial.println("SDHC");
            break;
        default:
            debug_serial.println("Unknown");
    }

    // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
    if (!volume.init(card)) {
        debug_serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
        while (1);
    }

    debug_serial.print("Clusters:          ");
    debug_serial.println(volume.clusterCount());
    debug_serial.print("Blocks x Cluster:  ");
    debug_serial.println(volume.blocksPerCluster());

    debug_serial.print("Total Blocks:      ");
    debug_serial.println(volume.blocksPerCluster() * volume.clusterCount());
    debug_serial.println();

    // print the type and size of the first FAT-type volume
    uint32_t volumesize;
    debug_serial.print("Volume type is:    FAT");
    debug_serial.println(volume.fatType(), DEC);

    volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
    volumesize *= volume.clusterCount();       // we'll have a lot of clusters
    volumesize /= 2;                           // SD card blocks are always 512 bytes (2 blocks are 1KB)
    debug_serial.print("Volume size (Kb):  ");
    debug_serial.println(volumesize);
    debug_serial.print("Volume size (Mb):  ");
    volumesize /= 1024;
    debug_serial.println(volumesize);
    debug_serial.print("Volume size (Gb):  ");
    debug_serial.println((float)volumesize / 1024.0);

    //debug_serial.println("\nFiles found on the card (name, date and size in bytes): ");
    //root.openRoot(volume);

    //// list all files in the card with date and size
    //root.ls(LS_R | LS_DATE | LS_SIZE);

    // Open and print the content of CONFIG.TXT
    SdFile config_file;
    char linebuf[128];
    if (!root.openRoot(volume)) {
        return false;
    }

    if (!config_file.open(&root, "CONFIG.TXT", O_READ)) {
        root.close();
        return false;
    }

    while (true) {
        char* s = read_line(config_file, linebuf, sizeof(linebuf));
        if (s == NULL) break;
        debug_serial.printf("%s\n", s);
    }
    config_file.close();
    root.close();
    digitalWrite(PIN_EN_SD, HIGH);
}

SdFile myFile;
char fileName[15];

void createFile() {
    if (!root.openRoot(volume)) {
        Serial.println("Failed to open root directory.");
        fileName = "data_1.txt" 
    }

    int fileIndex = 1;

    while (true) {
        sprintf(fileName, "data_%d.txt", fileIndex);
        
        SdFile tempFile;
        if (!tempFile.open(&root, fileName, O_READ)) {
            break;
        }
        
        tempFile.close();
        fileIndex++; 
    }

    if ( !myFile.open( fileName, O_CREAT | O_WRITE | O_TRUNC)) {
        Serial.print("Error opening file: ");
        Serial.println(fileName);
        return;
    }

    debug_serial.print( 'Create new file: ' ); debug_serial.println( fileName )
    myFile.println( "Time,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,UtcTime,Latitude,Longitude" );
    myFile.close();
}

void writeFile(const char *fileName, const char *data) {
    if (!root.openRoot(volume)) {
        debug_serial.println("Failed to open root directory.");
        return;
    }

    if (!myFile.open(&root, fileName, O_CREAT | O_WRITE | O_TRUNC)) {
        debug_serial.print("Error opening file: ");
        debug_serial.println(fileName);
        return;
    }

    myFile.print(data);
    myFile.close();      
    debug_serial.println("Write complete!");
}


/************************************************************
 *  IMU handling
 */

void init_imu() {
    if (!IMU.begin()) {
        debug_serial.println("Failed to initialize IMU!");
        SHORT_BLINK1();
        while (1);
    }

    debug_serial.print("Accelerometer sample rate = ");
    debug_serial.print(IMU.accelerationSampleRate());
    debug_serial.println(" Hz");
    debug_serial.println();
    debug_serial.println("Acceleration in g's");
    debug_serial.println("X\tY\tZ");
}

String get_imu_data() {
    String imuData = ',';

    float ac_x = 0.0, ac_y = 0.0, ac_z = 0.0;
    float gy_x = 0.0, gy_y = 0.0, gy_z = 0.0;

    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(ac_x, ac_y, ac_z);

        debug_serial.print("Accel: ");
        debug_serial.print(ac_x); debug_serial.print(", ");
        debug_serial.print(ac_y); debug_serial.print(", ");
        debug_serial.println(ac_z);
    }
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gy_x, gy_y, gy_z);

        debug_serial.print("Gyro: ");
        debug_serial.print(gy_x); debug_serial.print(", ");
        debug_serial.print(gy_y); debug_serial.print(", ");
        debug_serial.println(gy_z);
    }
    
    imuData += String(ac_x) + ',' + String(ac_y) + ',' + String(ac_z) + ',' + String(gy_x) + ',' + String(gy_y) + ',' + String(gy_z) + ',,,'; // add 3 more '' for magnatic that obsolete
    return imuData
}

/************************************************************
 *  GPS
 */

TinyGPSPlus gps;

void get_gps_data() {
    String gpsData = ',';

    if (gnss_serial.available()) {
        char c = gnss_serial.read();
        gps.encode(c); // Feed data into TinyGPS++

        if (gps.time.isUpdated()) {
            debug_serial.print("UTC Time: ");
            debug_serial.print(gps.time.hour());
            debug_serial.print(":");
            debug_serial.print(gps.time.minute());
            debug_serial.print(":");
            debug_serial.print(gps.time.second());
            gpsData += String(gps.time.hour()) + ':' + String(gps.time.minute()) + ':' + String(gps.time.second()) + ',';
        }
        else {
            debug_serial.print( "can't find Time" );
            gpsData += ',';
        }
        debug_serial.println();

        if (gps.location.isUpdated()) {
            debug_serial.print("Latitude: ");
            debug_serial.print(gps.location.lat(), 6);
            debug_serial.print("Longitude: ");
            debug_serial.print(gps.location.lng(), 6);
            debug_serial.print(", ");
            gpsData += String(gps.location.lat(), 6) + ',' + String(gps.location.lng(), 6) + ',';
        }
        else {
            debug_serial.print( "can't find location" );
            debug_serial.print(", ");
            gpsData += ',,';
        }

    }
    else {
        debug_serial.println( "gnss_serial not available" );
        gpsData += ',,,'
    }
    return gpsData
}


/************************************************************
 *  main
 */
void setup() {
    pullup_all_pins();
    init_gpio();
    debug_serial.begin(115200);
    delay(1000);
    debug_serial.println("Ready!!!");
    Wire.swap(0);
    Wire.begin();
    gnss_serial.begin(9600);
    SPI.swap(SPI1_SWAP_DEFAULT);
    SD.begin(PIN_SD_SS);
    
    init_imu();

    createNewFile();
    test_sdcard();

    digitalWrite(PIN_EN_GNSS, LOW);
}

/************************************************************
 *
 */
void loop() {
    unsigned long timeStamp = millis();
    char timeString[20];

    get_gps_data();
    get_imu_data();

    delay(500);
}

