#ifndef ANGLE_H_
#define ANGLE_H_

#include <math.h>
#include <HamsterAPIClientCPP/Hamster.h>
using namespace HamsterAPI;

double convertDegreesToRadians(double angleDegrees);
double convertRadiansToDegrees(double angleRadians);
void rotateMapOnOrigin(cv::Mat* source, cv::Mat* dest, double rotationAngle);
double getYawInOneCiricle(double yaw);

#endif
