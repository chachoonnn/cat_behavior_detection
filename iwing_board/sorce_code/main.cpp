#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <Arduino_LSM6DSOX.h>

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

#define SHORT_BLINK()  do { digitalWrite(PIN_LED1, HIGH); delay(10); digitalWrite(PIN_LED1, LOW); } while (0);

#define debug_serial Serial
#define gnss_serial Serial2

Sd2Card card;
SdVolume volume;
SdFile root;

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
 *
 */
void init_card() {
  SdVolume volume;
  card.init(SPI_HALF_SPEED, PIN_SD_SS);
}

/*******************************
 *
 */
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

/************************************************************
 *
 */
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

/************************************************************
 *
 */
void i2c_scanner() {
    byte error, address;
    int nDevices;

    debug_serial.println("Scanning...");

    nDevices = 0;
    for(address = 1; address < 127; address++ )
    {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
        {
            debug_serial.print("I2C device found at address 0x");
            if (address<16)
                debug_serial.print("0");
            debug_serial.print(address,HEX);
            debug_serial.println("  !");

            nDevices++;
        }
        else if (error==4)
        {
            debug_serial.print("Unknow error at address 0x");
            if (address<16)
                debug_serial.print("0");
            debug_serial.println(address,HEX);
        }
    }
    if (nDevices == 0)
        debug_serial.println("No I2C devices found\n");
    else
        debug_serial.println("done\n");
}

/************************************************************
 *
 */
void init_imu() {
    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");

        while (1);
    }

    Serial.print("Accelerometer sample rate = ");
    Serial.print(IMU.accelerationSampleRate());
    Serial.println(" Hz");
    Serial.println();
    Serial.println("Acceleration in g's");
    Serial.println("X\tY\tZ");
}

/************************************************************
 *
 */
void setup1() {
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

    //test_sdcard();
    digitalWrite(PIN_EN_GNSS, LOW);
}

/************************************************************
 *
 */
void loop1() {
    digitalWrite(PIN_LED1, HIGH);
    digitalWrite(PIN_LED2, LOW);
    i2c_scanner();
    delay(500);
    digitalWrite(PIN_LED1, LOW);
    digitalWrite(PIN_LED2, HIGH);
    delay(500);
}

/************************************************************
 *
 */
void loop2() {
    float x, y, z;
    int temp;

    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(x, y, z);
        Serial.println("Gyroscope");
        Serial.print(x);
        Serial.print('\t');
        Serial.print(y);
        Serial.print('\t');
        Serial.println(z);
    }
    if (IMU.temperatureAvailable()) {
        IMU.readTemperature(temp);
        Serial.println("Temperature");
        Serial.println(temp);
        delay(500);
    }
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(x, y, z);

        Serial.println("Acceleration");
        Serial.print(x);
        Serial.print('\t');
        Serial.print(y);
        Serial.print('\t');
        Serial.println(z);
    }
    delay(100);
}

/************************************************************
 *
 */
void loop3() {
    if (gnss_serial.available()) {
        Serial.print((char)gnss_serial.read());
    }
    if (Serial.available()) {
        gnss_serial.print((char)Serial.read());
    }
}
