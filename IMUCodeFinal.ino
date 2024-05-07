#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <RP2040_SD.h>
#include "RTClib.h"

// For SD Card pins
#if defined(ARDUINO_ARCH_MBED)
  #define PIN_SD_MOSI PIN_SPI_MOSI
  #define PIN_SD_MISO PIN_SPI_MISO
  #define PIN_SD_SCK PIN_SPI_SCK
  #define PIN_SD_SS PIN_SPI_SS
#else
  #define PIN_SD_MOSI PIN_SPI0_MOSI
  #define PIN_SD_MISO PIN_SPI0_MISO
  #define PIN_SD_SCK PIN_SPI0_SCK
  #define PIN_SD_SS PIN_SPI0_SS
#endif

#define BNO055_SAMPLERATE_DELAY_MS 100
#define CALIBRATION_FILE_NAME "cal.bin"

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
RTC_PCF8523 rtc;
File calibrationFile;
File IMUData;
char filename[15];

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("Initializing SD card...");
    if (!SD.begin(PIN_SD_SS)) {
        Serial.println("SD Initialization failed!");
        while (true);
    }
    Serial.println("SD Initialization done.");

    Serial.println("Initializing orientation sensor...");
    if (!bno.begin()) {
        Serial.println("Ooops, no BNO055 detected... Check your wiring or I2C ADDR!");
        while (true);
    }

    if (!rtc.begin()) {
        Serial.println("Couldn't find RTC");
        while (true);
    }

    if (!rtc.initialized() || rtc.lostPower()) {
        Serial.println("RTC is not initialized, setting time...");
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    rtc.start();

    strcpy(filename, "/DATA00.CSV");
    //This is a handy filesystem created by the folks at Adafruit that allows for a new file to be opened and written to each time the device is reset
    for (uint8_t i = 0; i < 100; i++) {
      filename[5] = '0' + i/10;
      filename[6] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (!SD.exists(filename)) {
      break;
    }
  }

    // Load calibration data from SD card
    calibrationFile = SD.open(CALIBRATION_FILE_NAME, FILE_READ);
    if (calibrationFile) {
        adafruit_bno055_offsets_t calibData;
        calibrationFile.read(&calibData, sizeof(adafruit_bno055_offsets_t));
        bno.setSensorOffsets(calibData);
        calibrationFile.close();
        Serial.println("Loaded calibration data from SD card.");
    } else {
        Serial.println("No calibration data found on SD card.");
    }

    bno.setExtCrystalUse(true);
    IMUData = SD.open(filename, FILE_WRITE);
    IMUData.print("RTCTime, OrientationX, OrientationY, OrientationZ, GyroscopeX, GyroscopeY, GyroscopeZ, LinearAccelX, LinearAccelY, LinearAccelZ, MagFieldX, MagFieldY, MagFieldZ, AccelX, AccelY, AccelZ, GravX, GravY, GravZ, Temp");
    IMUData.println();
    IMUData.close();

}

void loop() {
    // Get and print data from different sensors
    sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;

    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  /*
    // Print data for each sensor (comment out for final)
    printEvent(&orientationData);
    printEvent(&angVelocityData);
    printEvent(&linearAccelData);
    printEvent(&magnetometerData);
    printEvent(&accelerometerData);
    printEvent(&gravityData);
  */

    //for sd writing
    IMUData = SD.open(filename,FILE_WRITE);
    DateTime now = rtc.now();
    IMUData.printf("%ld:%ld:%ld, ",now.hour(),now.minute(),now.second());
    printSDEvent(&orientationData);
    printSDEvent(&angVelocityData);
    printSDEvent(&linearAccelData);
    printSDEvent(&magnetometerData);
    printSDEvent(&accelerometerData);
    printSDEvent(&gravityData);
    int8_t boardTemp = bno.getTemp();
    IMUData.print(boardTemp);
    IMUData.println();
    IMUData.close();

    // Serial.println("--");
    delay(BNO055_SAMPLERATE_DELAY_MS);
}
void printSDEvent(sensors_event_t* event) {
    double x = event->acceleration.x;
    double y = event->acceleration.y;
    double z = event->acceleration.z;
    switch (event->type) {
        case SENSOR_TYPE_ORIENTATION:
            IMUData.printf("%.4f, %.4f, %.4f, ", event->orientation.x, event->orientation.y, event->orientation.z);
            break;
        case SENSOR_TYPE_GYROSCOPE:
            IMUData.printf("%.4f, %.4f, %.4f, ", event->gyro.x, event->gyro.y, event->gyro.z);
            break;
        case SENSOR_TYPE_LINEAR_ACCELERATION:
            IMUData.printf("%.4f, %.4f, %.4f, ", event->acceleration.x, event->acceleration.y, event->acceleration.z);
            break;
        case SENSOR_TYPE_MAGNETIC_FIELD:
            IMUData.printf("%.4f, %.4f, %.4f, ", event->magnetic.x, event->magnetic.y, event->magnetic.z);
            break;
        case SENSOR_TYPE_GRAVITY:
            IMUData.printf("%.4f, %.4f, %.4f, ", event->acceleration.x, event->acceleration.y, event->acceleration.z);
            break;
        case SENSOR_TYPE_ACCELEROMETER:
            IMUData.printf("%.4f, %.4f, %.4f, ", event->acceleration.x, event->acceleration.y, event->acceleration.z);
            break;
    }
}

//comment out for final version
/*
void printEvent(sensors_event_t* event) {
    double x = event->acceleration.x;
    double y = event->acceleration.y;
    double z = event->acceleration.z;
    Serial.print("\t");
    switch (event->type) {
        case SENSOR_TYPE_ORIENTATION:
            Serial.print("Orientation: ");
            Serial.print("X=");
            Serial.print(event->orientation.x, 4);
            Serial.print(" Y=");
            Serial.print(event->orientation.y, 4);
            Serial.print(" Z=");
            Serial.println(event->orientation.z, 4);
            break;
        case SENSOR_TYPE_GYROSCOPE:
            Serial.print("Gyroscope: ");
            Serial.print("X=");
            Serial.print(event->gyro.x, 4);
            Serial.print(" Y=");
            Serial.print(event->gyro.y, 4);
            Serial.print(" Z=");
            Serial.println(event->gyro.z, 4);
            break;
        case SENSOR_TYPE_LINEAR_ACCELERATION:
            Serial.print("Linear Acceleration: ");
            Serial.print("X=");
            Serial.print(event->acceleration.x, 4);
            Serial.print(" Y=");
            Serial.print(event->acceleration.y, 4);
            Serial.print(" Z=");
            Serial.println(event->acceleration.z, 4);
            break;
        case SENSOR_TYPE_MAGNETIC_FIELD:
            Serial.print("Magnetic Field: ");
            Serial.print("X=");
            Serial.print(event->magnetic.x, 4);
            Serial.print(" Y=");
            Serial.print(event->magnetic.y, 4);
            Serial.print(" Z=");
            Serial.println(event->magnetic.z, 4);
            break;
        case SENSOR_TYPE_GRAVITY:
            Serial.print("Gravity: ");
            Serial.print("X=");
            Serial.print(event->acceleration.x, 4);
            Serial.print(" Y=");
            Serial.print(event->acceleration.y, 4);
            Serial.print(" Z=");
            Serial.println(event->acceleration.z, 4);
            break;
        case SENSOR_TYPE_ACCELEROMETER:
            Serial.print("Accel: ");
            Serial.print("X=");
            Serial.print(event->acceleration.x, 4);
            Serial.print(" Y=");
            Serial.print(event->acceleration.y, 4);
            Serial.print(" Z=");
            Serial.println(event->acceleration.z, 4);
            break;
            
    }
}
*/
