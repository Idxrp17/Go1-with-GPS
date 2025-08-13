#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// GPS module pins (adjust if needed)
static const int RXPin = 4; // GPS TX → Arduino pin 4
static const int TXPin = 3; // GPS RX ← Arduino pin 3
static const uint32_t GPSBaud = 9600; // Default NEO-M8N baud rate

// Create GPS object
TinyGPSPlus gps;

// Create serial connection to GPS
SoftwareSerial gpsSerial(RXPin, TXPin);

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(GPSBaud);

  Serial.println("u-blox NEO-M8N GPS Reader");
  Serial.println("--------------------------------");
}

void loop() {
  // Feed data from GPS to TinyGPS++
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // Only display data if we have a valid fix
  if (gps.location.isValid() && gps.satellites.isValid()) {
    Serial.print("Satellites: ");
    Serial.println(gps.satellites.value());

    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);

    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);

    Serial.print("Speed (MPH): ");
    Serial.println(gps.speed.mph());

    Serial.println("--------------------------------");
  } else {
    Serial.println("Waiting for GPS fix...");
  }

  delay(100); // Update every second
}
