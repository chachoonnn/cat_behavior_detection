#include "Arduino.h"
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <TinyGPS++.h>

//***********************************************************
// Pins (adjust as needed for your board)
//***********************************************************
#define PIN_EN_GNSS   PIN_PD5
#define PIN_LED1      PIN_PA6
#define PIN_LED2      PIN_PA5
#define PIN_GNSS_RX   PIN_PF0
#define PIN_GNSS_TX   PIN_PF1

//***********************************************************
// Macros
//***********************************************************
#define SHORT_BLINK1()  do {             \
    digitalWrite(PIN_LED1, HIGH);        \
    delay(10);                           \
    digitalWrite(PIN_LED1, LOW);         \
} while (0)

#define SHORT_BLINK2()  do {             \
    digitalWrite(PIN_LED2, HIGH);        \
    delay(10);                           \
    digitalWrite(PIN_LED2, LOW);         \
} while (0)

//***********************************************************
// Globals
//***********************************************************
#define debug_serial Serial
#define gnss_serial  Serial2

TinyGPSPlus gps;

//***********************************************************
// Setup pins
//***********************************************************
void init_gpio() {
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(PIN_EN_GNSS, OUTPUT);

  // If your hardware is active-low for GNSS power:
  digitalWrite(PIN_EN_GNSS, LOW);  // Power GNSS module ON
  // If your hardware is active-high, you might do digitalWrite(PIN_EN_GNSS, HIGH);
}

//***********************************************************
// Setup function
//***********************************************************
void setup() {
  init_gpio();
  
  // Start serial for debug
  debug_serial.begin(115200);
  delay(1000);
  debug_serial.println("GPS Test Starting...");

  // If your board supports Wire.swap(0) or similar, do it if needed
  // Wire.swap(0);
  // Wire.begin();

  // Initialize GNSS Serial 
  // (check your board's docs for the correct pins or usage of Serial2)
  gnss_serial.begin(9600);

  // Blink once to show we're alive
  SHORT_BLINK1();
}

//***********************************************************
// Main loop
//***********************************************************
void loop() {
    // 1) Continuously read from GNSS serial
    while (gnss_serial.available()) {
        char c = gnss_serial.read();
        gps.encode(c);
    }

    // 2) If we have a new updated location/time, print it
    //    (TinyGPS++ sets these flags when a new full NMEA sentence has been parsed)
    if (gps.location.isUpdated()) {
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
            debug_serial.print(gps.time.second());
        }
        debug_serial.println();

        // Blink LED2 to indicate we got new data
        SHORT_BLINK2();
    }

    // Optional small delay
    delay(100);
}
