// Rocket Avionics Datalogger
// 2021-07-14
// 
// This program is used to log sensor data during the launch of a 
// model rocket. The program is run on a microcontroller connected 
// to a BMP280 to obtain altitude and a MicroSD card reader to 
// store the data. 
//
// The BMP280 is connected via I2C. 
// The MicroSD card reader is connected using SPI.
// The datalogger runs at its maximum sample rate (currently 25/sec). 
// The data is stored to the MicroSD card as a CSV file.
// The datalogger starts at power-up. 


// Libraries:
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

// Global variables:
const int pinChipSelect = 10;
const int pinErrorLED = 9;
const int pinSuccessLED = 8;

// Classes:
Adafruit_BMP280 bmp;


void setup() {

  Serial.begin(9600);
  
  // Set up the status LEDs.
  pinMode(pinErrorLED, OUTPUT);
  pinMode(pinSuccessLED, OUTPUT);
  digitalWrite(pinErrorLED, LOW);
  digitalWrite(pinSuccessLED, LOW);
 
  // Initialize the MicroSD card.
  Serial.print("Initializing SD card...");
  if (!SD.begin(pinChipSelect)) {
    Serial.println("Failed to find the SD card.");
    digitalWrite(pinErrorLED, HIGH);
    while (1);
  }
  Serial.println("Card initialized.");

  // Initialize the BMP280 sensor.
  if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
    Serial.println(F("Failed to find the BMP280 sensor."));
    digitalWrite(pinErrorLED, HIGH);
    while (1); 
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Operating Mode 
                  Adafruit_BMP280::SAMPLING_X2,     // Temp. oversampling 
                  Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling 
                  Adafruit_BMP280::FILTER_X16,      // Filtering 
                  Adafruit_BMP280::STANDBY_MS_1);   // Standby time
  Serial.println("BMP280 initialized.");

  // Create the field headings within the data file on the MicroSD card.
  File dataFile = SD.open("DATALOG.TXT", FILE_WRITE);

  if (dataFile) {
    dataFile.println(F("Microseconds, Temperature (C), Air Pressure (Pa), Altitude (m), "));
    dataFile.close();
  }
  else {
    Serial.println("Error opening the log file.");
    digitalWrite(pinErrorLED, HIGH);
  }

  // Report successful completion of initialization stage.
  Serial.println("Initialization successful.");
  digitalWrite(pinSuccessLED, HIGH);
  
}


void loop() {

  // Open the file. If the file is available, write to it. If not, report an error.
  File dataFile = SD.open("DATALOG.TXT", FILE_WRITE);

  if (dataFile) {
    dataFile.print(micros());
    dataFile.print(",");
    dataFile.print(bmp.readTemperature(),2);
    dataFile.print(",");
    dataFile.print(bmp.readPressure(),2);
    dataFile.print(",");
    dataFile.println(bmp.readAltitude(1013.25),2);
    dataFile.close();
  }
  else {
    Serial.println("Error opening the log file.");
    digitalWrite(pinErrorLED, HIGH);
  }
    
}
