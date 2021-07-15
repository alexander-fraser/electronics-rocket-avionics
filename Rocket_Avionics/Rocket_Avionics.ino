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
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

// Constants:
const int chipSelect = 10;

// Classes:
Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;


void setup() {

  Serial.begin(9600);

  // Initialize the MicroSD card.
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("Failed to find the SD card.");
    // This would be a great place to light a red LED as a warning.
    while (1);
  }
  Serial.println("Card initialized.");

  // Initialize the BMP280 sensor.
  if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
    Serial.println(F("Failed to find the BMP280 sensor."));
    // This would be a great place to light a red LED as a warning.
    while (1); 
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Operating Mode 
                  Adafruit_BMP280::SAMPLING_X2,     // Temp. oversampling 
                  Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling 
                  Adafruit_BMP280::FILTER_X16,      // Filtering 
                  Adafruit_BMP280::STANDBY_MS_1);   // Standby time
  Serial.println("BMP280 initialized.");

  // Initialize the MPU6050 sensor.
  if (!mpu.begin()) {
    Serial.println("Failed to find the MPU6050 sensor.");
    // This would be a great place to light a red LED as a warning.
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 initialized.");
  // This would be a great place to light a green LED to show success.
  Serial.println(mpu.getAccelerometerRange());
  Serial.println(mpu.getGyroRange());
  Serial.println(mpu.getFilterBandwidth());
  
}


void loop() {
  
  // Make a string for assembling the data for a log entry.
  String dataString = "";

  // Read the sensors and append to the string.
  int sensorDataTemp = bmp.readTemperature();
  int sensorDataPressure = bmp.readPressure();
  int sensorDataAltitude = bmp.readAltitude(1013.25);

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  int sensorDataAccelerationX = a.acceleration.x;
  int sensorDataAccelerationY = a.acceleration.y;
  int sensorDataAccelerationZ = a.acceleration.z;
  int sensorDataRotationX = g.gyro.x;
  int sensorDataRotationY = g.gyro.y;
  int sensorDataRotationZ = g.gyro.z;
  
  dataString = sensorDataTemp + "," 
              + sensorDataPressure + "," 
              + sensorDataAltitude + ","
              + sensorDataAccelerationX + ","
              + sensorDataAccelerationY + ","
              + sensorDataAccelerationZ + ","
              + sensorDataRotationX + ","
              + sensorDataRotationY + ","
              + sensorDataRotationZ + ",";

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
