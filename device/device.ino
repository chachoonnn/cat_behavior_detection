#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>

// Uncomment to enable debug output
 #define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(x)   Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

// Chip select pins
#define SD_CS    10
#define RFM95_CS  8
#define LED_PIN  13

Adafruit_LSM6DSOX lsm6dsox;
Adafruit_LIS3MDL lis3mdl;
RTC_PCF8523 rtc;  // RTC object

char fileName[15];  // Buffer for file name

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

#ifdef DEBUG
  Serial.begin(115200);
  delay(500); // Allow time for Serial to initialize
#endif

  // Set CS pins as outputs
  pinMode(SD_CS, OUTPUT);
  pinMode(RFM95_CS, OUTPUT);

  // Disable RFM95 to prevent SPI conflicts
  digitalWrite(RFM95_CS, HIGH);

  DEBUG_PRINTLN("Initializing IMUs...");

  // Initialize IMUs
  Wire.begin();
  if (!lsm6dsox.begin_I2C()) {
    DEBUG_PRINTLN("Failed to find LSM6DSOX!");
    errorBlink();
  }
  if (!lis3mdl.begin_I2C()) {
    DEBUG_PRINTLN("Failed to find LIS3MDL!");
    errorBlink();
  }

  DEBUG_PRINTLN("Initializing RTC...");
  if (!rtc.begin()) {
    DEBUG_PRINTLN("Couldn't find RTC! Timestamps will use millis().");
  }

  DEBUG_PRINTLN("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    DEBUG_PRINTLN("SD card initialization failed!");
    errorBlink();
  }

  createNewFile();  // Generate a unique file name
  DEBUG_PRINTLN("Setup complete.");
}

void loop() {
  unsigned long timeStamp = millis();
  char timeString[20];

  // Get the current timestamp
  if (rtc.begin() && rtc.initialized()) {
    DateTime now = rtc.now();
    snprintf(timeString, sizeof(timeString), "%04d-%02d-%02d %02d:%02d:%02d",
             now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  } else {
    snprintf(timeString, sizeof(timeString), "%lu", timeStamp);
  }

  // Get IMU data
  sensors_event_t accel, gyro, temp;
  lsm6dsox.getEvent(&accel, &gyro, &temp);
  sensors_event_t magEvent;
  lis3mdl.getEvent(&magEvent);

#ifdef DEBUG
  DEBUG_PRINT("Time: ");
  DEBUG_PRINTLN(timeString);
  DEBUG_PRINT("Accel: ");
  DEBUG_PRINT(accel.acceleration.x); DEBUG_PRINT(", ");
  DEBUG_PRINT(accel.acceleration.y); DEBUG_PRINT(", ");
  DEBUG_PRINTLN(accel.acceleration.z);
  DEBUG_PRINT("Gyro: ");
  DEBUG_PRINT(gyro.gyro.x); DEBUG_PRINT(", ");
  DEBUG_PRINT(gyro.gyro.y); DEBUG_PRINT(", ");
  DEBUG_PRINTLN(gyro.gyro.z);
  DEBUG_PRINT("Mag: ");
  DEBUG_PRINT(magEvent.magnetic.x); DEBUG_PRINT(", ");
  DEBUG_PRINT(magEvent.magnetic.y); DEBUG_PRINT(", ");
  DEBUG_PRINTLN(magEvent.magnetic.z);
#endif

  // Write data to SD card
  File dataFile = SD.open(fileName, FILE_WRITE);
  if (dataFile) {
    dataFile.print(timeString); dataFile.print(",");
    dataFile.print(accel.acceleration.x); dataFile.print(",");
    dataFile.print(accel.acceleration.y); dataFile.print(",");
    dataFile.print(accel.acceleration.z); dataFile.print(",");
    dataFile.print(gyro.gyro.x); dataFile.print(",");
    dataFile.print(gyro.gyro.y); dataFile.print(",");
    dataFile.print(gyro.gyro.z); dataFile.print(",");
    dataFile.print(magEvent.magnetic.x); dataFile.print(",");
    dataFile.print(magEvent.magnetic.y); dataFile.print(",");
    dataFile.println(magEvent.magnetic.z);
    dataFile.close();
    blinkLED();
  } else {
    DEBUG_PRINTLN("Failed to write to file.");
  }

  delay(1000);
}

void createNewFile() {
  int fileIndex = 1;

  do {
    snprintf(fileName, sizeof(fileName), "DATA%04d.TXT", fileIndex);
    fileIndex++;
  } while (SD.exists(fileName));

  DEBUG_PRINT("New file created: ");
  DEBUG_PRINTLN(fileName);

  // Open and write a header to the new file
  File dataFile = SD.open(fileName, FILE_WRITE);
  if (dataFile) {
    dataFile.println("Time,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ");
    dataFile.close();
  } else {
    DEBUG_PRINTLN("Failed to create new file.");
    errorBlink();
  }
}

void blinkLED() {
  digitalWrite(LED_PIN, HIGH);
  delay(50);
  digitalWrite(LED_PIN, LOW);
}

void errorBlink() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}
