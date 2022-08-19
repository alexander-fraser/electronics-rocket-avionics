// Rocket Avionics Datalogger - Rev2-1 STM32 Altimeter Only
// 2022-07-30
// 
// This program is used to log sensor data during the launch of a 
// model rocket. The program is run on a microcontroller connected 
// to a BMP280 to obtain altitude, and a MicroSD card reader to 
// store the data. 
//
// The BMP280 is connected via I2C. 
// The MicroSD card reader is connected using SPI.
// The data is stored to the MicroSD card as a CSV file.
// The datalogger starts at power-up. 


// Libraries:
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP280.h>


// Global variables:
const int pinChipSelect = PA4;
const int pinLED = PC13;
unsigned long intervalLED = 1000;
unsigned long intervalLEDfailure = 250;
unsigned long intervalLEDsuccess = 2000;
int stateLED = LOW;
unsigned long previousMillis = 0;


// Classes:
Adafruit_BMP280 bmp;


void setup(void) {

  Serial.begin(115200);
  
  // Set up the status LED.
  pinMode(pinLED, OUTPUT);
  digitalWrite(pinLED, stateLED);
 
  // Initialize the MicroSD card.
  Serial.print("Initializing SD card...");
  if (!SD.begin(pinChipSelect)) {
    Serial.println("Failed to find the SD card.");
    blinkLEDError(intervalLEDfailure);
  }
  Serial.println("Card initialized.");

  // Create the field headings within the data file on the MicroSD card.
  File dataFile = SD.open("DATALOG.TXT", FILE_WRITE);

  if (dataFile) {
    dataFile.println("Microseconds, Temperature (C), Air Pressure (Pa), Altitude (m), ");
    dataFile.close();
  }
  else {
    Serial.println("Error opening the log file.");
    blinkLEDError(intervalLEDfailure);
  }

  // Initialize the BMP280 sensor.
  if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
    Serial.println("Failed to find the BMP280 sensor.");
    blinkLEDError(intervalLEDfailure);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Operating Mode 
                  Adafruit_BMP280::SAMPLING_X2,     // Temp. oversampling 
                  Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling 
                  Adafruit_BMP280::FILTER_X16,      // Filtering 
                  Adafruit_BMP280::STANDBY_MS_1);   // Standby time
  Serial.println("BMP280 initialized.");

  // Report successful completion of initialization stage.
  Serial.println("Initialization successful.");
  intervalLED = intervalLEDsuccess;
  
}


void loop(void) {

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
    intervalLED = intervalLEDfailure;
  }

  // Blink the LED to report successful initialization or runtime error, while program keeps running.
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= intervalLED) {
    previousMillis = currentMillis;

    if (stateLED == LOW) {
      stateLED = HIGH;
    } else {
      stateLED = LOW;
    }

    digitalWrite(pinLED, stateLED);
  }
    
}


// Blink the LED to report initialization failure and stop program.
void blinkLEDError(unsigned long blinkInterval) {
  while (1) {
    digitalWrite(pinLED, HIGH);
    delay(blinkInterval);
    digitalWrite(pinLED, LOW);
    delay(blinkInterval);
  }
}
