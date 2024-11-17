#include "IMUPositionTracker.h"

// Constructor
IMUPositionTracker::IMUPositionTracker()
    : accX(0), accY(0), accZ(0),
      velX(0), velY(0), velZ(0),
      posX(0), posY(0), posZ(0),
      prevAccX(0), prevAccY(0), prevAccZ(0),
      prevVelX(0), prevVelY(0), prevVelZ(0),
      prevTime(0) {}

// Update Position, Velocity, Acceleration
void IMUPositionTracker::updatePosition(float newAccX, float newAccY, float newAccZ)
{
    // Get current time
    unsigned long currentTime = millis();
    if (prevTime == 0)
    {
        prevTime = currentTime;
        return;
    }

    // Get time difference
    float dt = (currentTime - prevTime) / 1000.0; // Convert to seconds

    // Highpass filtering for acceleration
    accX = ALPHA * newAccX + (1 - ALPHA) * prevAccX;
    accY = ALPHA * newAccY + (1 - ALPHA) * prevAccY;
    accZ = ALPHA * newAccZ + (1 - ALPHA) * prevAccZ;

    // Integrate to get velocity
    velX += accX * dt;
    velY += accY * dt;
    velZ += accZ * dt;

    // Highpass filtering for velocity
    velX = ALPHA * velX + (1 - ALPHA) * prevVelX;
    velY = ALPHA * velY + (1 - ALPHA) * prevVelY;
    velZ = ALPHA * velZ + (1 - ALPHA) * prevVelZ;

    // Integrate to get position
    posX += velX * dt;
    posY += velY * dt;
    posZ += velZ * dt;

    // Update previous values
    prevAccX = accX;
    prevAccY = accY;
    prevAccZ = accZ;
    prevVelX = velX;
    prevVelY = velY;
    prevVelZ = velZ;
    prevTime = currentTime;
}

// Get Current Position
void IMUPositionTracker::getPosition(float &x, float &y, float &z)
{
    x = posX;
    y = posY;
    z = posZ;
}

// Get Current Velocity
void IMUPositionTracker::getVelocity(float &vX, float &vY, float &vZ)
{
    vX = velX;
    vY = velY;
    vZ = velZ;
}

// Reset Position and Velocity
void IMUPositionTracker::resetPosition()
{
    posX = 0;
    posY = 0;
    posZ = 0;
    velX = 0;
    velY = 0;
    velZ = 0;
}
