// Rocket Avionics Datalogger
// 2021-07-14
// 
// This program is used to log sensor data during the launch of a 
// model rocket. The program is run on a microcontroller connected 
// to a BNO055 IMU & an MPU6050 6DOF module to obtain acceleration and orientation 
// data, a BMP280 to obtain altitude, and a MicroSD card reader to 
// store the data. 
//
// The BNO055, MPU6050, and BMP280 are connected via I2C. 
// The MicroSD card reader is connected using SPI.
// The datalogger runs at its maximum sample rate (currently 22/sec). 
// The data is stored to the MicroSD card as a CSV file.
// The datalogger starts at power-up. 


// Libraries:
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD.h>


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
Adafruit_MPU6050 mpu;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);


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

  // Initialize the BMP280 sensor.
  if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
    Serial.println(F("Failed to find the BMP280 sensor."));
    blinkLEDError(intervalLEDfailure);
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
    blinkLEDError(intervalLEDfailure);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  Serial.println("MPU6050 initialized.");

  // Initialise the BNO055 sensor.
  if (!bno.begin()) {
    Serial.println("Failed to find the BNO055 sensor.");
    blinkLEDError(intervalLEDfailure);
  }

  // Create the field headings within the data file on the MicroSD card.
  File dataFile = SD.open("DATALOG.TXT", FILE_WRITE);

  if (dataFile) {
    dataFile.println(F("Microseconds, Temperature (C), Air Pressure (Pa), Altitude (m), " 
                      "MPU-Acceleration-X, MPU-Acceleration-Y, MPU-Acceleration-Z, "
                      "MPU-Rotation-X, MPU-Rotation-Y, MPU-Rotation-Z, "
                      "BNO-Orientation-X, BNO-Orientation-Y, BNO-Orientation-Z, "
                      "BNO-Gyro-X, BNO-Gyro-Y, BNO-Gyro-Z, "
                      "BNO-Linear-X, BNO-Linear-Y, BNO-Linear-Z, "
                      "BNO-Magnetometer-X, BNO-Magnetometer-Y, BNO-Magnetometer-Z, "
                      "BNO-Accelerometer-X, BNO-Accelerometer-Y, BNO-Accelerometer-Z, "
                      "BNO-Gravity-X, BNO-Gravity-Y, BNO-Gravity-Z, "
                      "BNO-Temperature, BNO-Calibration-System, "
                      "BNO-Calibration-Gyro, BNO-Calibration-Accel, BNO-Calibration-Mag"));
    dataFile.close();
  }
  else {
    Serial.println("Error opening the log file.");
    blinkLEDError(intervalLEDfailure);
  }

  // Report successful completion of initialization stage.
  Serial.println("Initialization successful.");
  intervalLED = intervalLEDsuccess;
  
}


void loop(void) {
       
  // Request the data from the BNO055 and MPU6050.
  sensors_event_t a, g, temp, orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  mpu.getEvent(&a, &g, &temp);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
  int8_t boardTemp = bno.getTemp();
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  // Open the file. If the file is available, write to it. If not, report an error.
  File dataFile = SD.open("DATALOG.TXT", FILE_WRITE);

  if (dataFile) {
    dataFile.print(micros());
    dataFile.print(",");
    dataFile.print(bmp.readTemperature(),2);
    dataFile.print(",");
    dataFile.print(bmp.readPressure(),2);
    dataFile.print(",");
    dataFile.print(bmp.readAltitude(1013.25),2);
    dataFile.print(",");
    dataFile.print(a.acceleration.x,3);
    dataFile.print(",");
    dataFile.print(a.acceleration.y,3);
    dataFile.print(",");
    dataFile.print(a.acceleration.z,3);
    dataFile.print(",");
    dataFile.print(g.gyro.x,4);
    dataFile.print(",");
    dataFile.print(g.gyro.y,4);
    dataFile.print(",");
    dataFile.print(g.gyro.z,4);
    dataFile.print(",");
    dataFile.print(orientationData.orientation.x,4);
    dataFile.print(",");
    dataFile.print(orientationData.orientation.y,4);
    dataFile.print(",");
    dataFile.print(orientationData.orientation.z,4);
    dataFile.print(",");  
    dataFile.print(angVelocityData.gyro.x,4);
    dataFile.print(",");
    dataFile.print(angVelocityData.gyro.y,4);
    dataFile.print(",");
    dataFile.print(angVelocityData.gyro.z,4);
    dataFile.print(",");
    dataFile.print(linearAccelData.acceleration.x,4);
    dataFile.print(",");
    dataFile.print(linearAccelData.acceleration.y,4);
    dataFile.print(",");
    dataFile.print(linearAccelData.acceleration.z,4);
    dataFile.print(",");
    dataFile.print(magnetometerData.magnetic.x,4);
    dataFile.print(",");
    dataFile.print(magnetometerData.magnetic.y,4);
    dataFile.print(",");
    dataFile.print(magnetometerData.magnetic.z,4);
    dataFile.print(",");
    dataFile.print(accelerometerData.acceleration.x,4);
    dataFile.print(",");
    dataFile.print(accelerometerData.acceleration.y,4);
    dataFile.print(",");
    dataFile.print(accelerometerData.acceleration.z,4);
    dataFile.print(",");
    dataFile.print(gravityData.acceleration.x,4);
    dataFile.print(",");
    dataFile.print(gravityData.acceleration.y,4);
    dataFile.print(",");
    dataFile.print(gravityData.acceleration.z,4);
    dataFile.print(",");
    dataFile.print(boardTemp);
    dataFile.print(",");
    dataFile.print(system);
    dataFile.print(","); 
    dataFile.print(gyro);
    dataFile.print(","); 
    dataFile.print(accel);
    dataFile.print(","); 
    dataFile.println(mag);                          
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
