#ifndef IMU_POSITION_TRACKER_H
#define IMU_POSITION_TRACKER_H

#include <Arduino.h>

class IMUPositionTracker
{
private:
    // Current linear acceleration, velocity, position
    float accX, accY, accZ; // acceleration
    float velX, velY, velZ; // velocity
    float posX, posY, posZ; // position

    // Previous acceleration and velocity
    float prevAccX, prevAccY, prevAccZ;
    float prevVelX, prevVelY, prevVelZ;

    // Previous time
    unsigned long prevTime;

    // Filter hyperparameters
    const float BETA = 0.9;  // Lowpass filter coefficient
    const float ALPHA = 0.1; // Highpass filter coefficient

public:
    // Constructor
    IMUPositionTracker();

    // Update Position, Velocity, Acceleration
    void updatePosition(float newAccX, float newAccY, float newAccZ);

    // Get Current Position
    void getPosition(float &x, float &y, float &z);

    // Get Current Velocity
    void getVelocity(float &vX, float &vY, float &vZ);

    // Reset Position and Velocity
    void resetPosition();
};

#endif // IMU_POSITION_TRACKER_H
