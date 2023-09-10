#ifndef DRIVE_H
#define DRIVE_H

#include <cmath>

class OmniWheelDrive
{
private:
    double wheelRadius;
    double wheelSeparation;
    double wheelAngle;

public:
    OmniWheelDrive(double radius, double separation) : wheelRadius(radius), wheelSeparation(separation)
    {
        wheelAngle = 120.0 * (M_PI / 180.0);
    }

    void calculateWheelSpeeds(double xVel, double yVel, double zAngular, int Speed[3])
    {
        Speed[0] = (-xVel - (wheelSeparation / 2.0) * zAngular) / wheelRadius;
        Speed[1] = (-xVel * cos(wheelAngle) + yVel * sin(wheelAngle) - (wheelSeparation / 2.0) * zAngular) / wheelRadius;
        Speed[2] = (-xVel * cos(wheelAngle) - yVel * sin(wheelAngle) - (wheelSeparation / 2.0) * zAngular) / wheelRadius;
    }
};

#endif
