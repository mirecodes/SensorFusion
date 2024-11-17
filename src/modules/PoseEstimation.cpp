#include <Arduino.h>
#include "modules/PoseEstimation.h"
#include "config.h"

#include "MPU9250.h"
#include "eeprom_utils.h"
#include "TinyGPSPlus.h"

MPU9250 mpu;
float accX = 0, accY = 0, accZ = 0;
float roll = 0, pitch = 0, yaw = 0;
float rollRate = 0, pitchRate = 0, yawRate = 0;
uint32_t prev_ms_imu = 0;

TinyGPSPlus gps;
int sats = 0;
float hdop = 0, latitude = 0, longitude = 0;
uint32_t age = 0;
float altitude = 0;
uint32_t prev_ms_gps = 0;

void init_pose_estimation()
{
    // GPS Communication Start
    SER_GPS.begin(9600);

    // IMU Communication Start
    Wire.begin();
    delay(2000);
    if (!mpu.setup(0x68))
    {
        while (1)
        {
            Serial.println("MPU connection failed");
            digitalWrite(BUZ, HIGH);
            delay(50);
            digitalWrite(BUZ, LOW);
            delay(50);
        }
    }
    loadCalibration();
    // mpu.setFilterIterations(10);
}

const int IMU_UPDATE_PERIOD = 10;  // [ms]
const int GPS_UPDATE_PERIOD = 100; // [ms]

void loop_pose_estimation()
{
    uint32_t curr_ms = millis();

    if (curr_ms - prev_ms_imu >= IMU_UPDATE_PERIOD)
    {
        update_IMU_data();
        prev_ms_imu = curr_ms;
    }

    if (curr_ms - prev_ms_gps >= GPS_UPDATE_PERIOD)
    {
        update_GPS_data();
        prev_ms_gps = curr_ms;
    }

    while (SER_GPS.available())
    {
        gps.encode(SER_GPS.read());
    }
}

void update_IMU_data()
{
    if (mpu.update())
    {
        // Get axial acceleration excluding gravitational accleration
        accX = mpu.getLinearAccX();
        accY = mpu.getLinearAccY();
        accZ = mpu.getLinearAccZ();

        // Get rotational information
        // Euler angles
        roll = mpu.getEulerX();
        pitch = mpu.getEulerY();
        yaw = mpu.getEulerZ();
        // Angular velocities
        rollRate = mpu.getGyroX();
        pitchRate = mpu.getGyroY();
        yawRate = mpu.getGyroZ();
    }
}

void update_GPS_data()
{
    sats = gps.satellites.value();    // the num of satelites under commnunication (default : 0)
    hdop = gps.hdop.hdop();           // Horizontal Dilution of Precision, 0.01 (precise) - 99.99 (inaccurate) (default : 99.99)
    latitude = gps.location.lat();    // lattitude (default : 0.0)
    longitude = gps.location.lng();   // longitude (default : 0.0)
    age = gps.location.age();         // the time passed from the last communication [ms] (default : 4294967295 (ms))
    altitude = gps.altitude.meters(); // altitude (default : 0.0)
}

void calibration()
{
    Serial.println("Accel Gyro calibration will start.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);

    digitalWrite(BUZ, HIGH);
    delay(100);
    digitalWrite(BUZ, LOW);
    delay(1000);
    digitalWrite(BUZ, HIGH);
    delay(100);
    digitalWrite(BUZ, LOW);
    delay(1000);
    digitalWrite(BUZ, HIGH);
    delay(100);
    digitalWrite(BUZ, LOW);
    delay(1000);
    mpu.calibrateAccelGyro();
    digitalWrite(BUZ, HIGH);
    delay(1000);
    digitalWrite(BUZ, LOW);
    delay(3000);

    Serial.println("Mag calibration will start.");
    Serial.println("Please Wave device in a figure eight until done.");

    digitalWrite(BUZ, HIGH);
    delay(50);
    digitalWrite(BUZ, LOW);
    delay(50);
    digitalWrite(BUZ, HIGH);
    delay(50);
    digitalWrite(BUZ, LOW);
    delay(50);
    mpu.calibrateMag();
    digitalWrite(BUZ, HIGH);
    delay(1000);
    digitalWrite(BUZ, LOW);

    // print_calibration();
    mpu.verbose(false);

    saveCalibration();
    loadCalibration();
}

void print_calibration()
{
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}
