// Rocket Avionics Datalogger
// 2021-07-14
// 
// This program is used to log sensor data during the launch of a 
// model rocket. The program is run on a microcontroller connected 
// to a MPU6050 6DOF module to obtain acceleration and orientation 
// data, a BMP280 to obtain altitude, and a MicroSD card reader to 
// store the data. 
//
// The MPU6050 and BMP280 are connected via I2C. 
// The MicroSD card reader is connected using SPI.
// The datalogger runs at 100Hz (i.e. 100 samples per second). 
// The data is stored to the MicroSD card as a CSV file. 
// The datalogger starts when a button is pushed and runs for 5 minutes.


// Libraries:
#include <SPI.h>
#include <SD.h>


// Constants:
const int chipSelect = 10;


void setup() {

  Serial.begin(9600);

  // Initialize the MicroSD card.
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present.");
    while (1);
    // This would be a great place to light a red LED as a warning.
  }
  Serial.println("Card initialized.");
  // This would be a great place to light a green LED as a success.
  
}


void loop() {
  
  // Make a string for assembling the data for a log entry.
  String dataString = "";

  // Read the sensors and append to the string.
  dataString = "2" + "," + "3";

  // Open the file. If the file is available, write to it. If not, report and error.
  File dataFile = SD.open("rocketlog.csv", FILE_WRITE);

  if (dataFile) {
    Serial.println(dataString);
    dataFile.println(dataString);
    dataFile.close();
  }
  else {
    Serial.println("Error opening the log file.");
  }
  
}
