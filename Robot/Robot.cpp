#include "Robot.h"

Robot::Robot()
{
	 cv::namedWindow("Room-Map");

	this->leHamster = new HamsterAPI::Hamster(1);

	// Safety
	sleep(3);

	this->mapHeight = -1;
	this->mapWidth = -1;
	this->resolution = -1;
	this->localizationManager = NULL;
}

// Get the hamster's current location by returning a structure of {positionState} type
positionState Robot::getRealHamsterLocation()
{
	// Get location of robot using getPose()
	// TODO: Stop using getPose() and start using Particles!
	Pose currPose = leHamster->getPose();

	// Calculate {x,y} by resolution
	double x = currPose.getX() / resolution;
	double y = currPose.getY() * (-1) / resolution;

	// Hamster's angle
	double currYaw = getYawInOneCiricle(currPose.getHeading());

	position currentPos = { .x = x, .y = y };
	positionState currPosState = { .pos = currentPos, .yaw = 360 - currYaw };

	return currPosState;
}

// Update the hamster's location
//
void Robot::updateHamsterRealLocation()
{
	realLocation = getRealHamsterLocation();
}

// Move hamster in given speed and angle
void Robot::setHamsterSpeed(double speed, double wheelsAngle)
{
	this->leHamster->sendSpeed(speed, wheelsAngle);
}

void Robot::updateParticle()
{
	HamsterAPI::LidarScan scan = this->leHamster->getLidarScan();
	std::vector<double> distances;
	getScansBetween(0,360,distances);
	
	this->localizationManager->updateParticles(this->prevBeliefedLocation.x - this->currBeliefedLocation.x,this->prevBeliefedLocation.y - this->currBeliefedLocation.y,this->prevBeliefedLocation.yaw - this->currBeliefedLocation.yaw,distances, scan.getScanAngleIncrement() * DEG2RAD, scan.getMaxRange());		
}

void Robot::getScansBetween(double min, double max, std::vector<double> & distances) 
{
	HamsterAPI::LidarScan scan = this->leHamster->getLidarScan();

	for (size_t i = 0; i < scan.getScanSize(); i++) {
		double degree = scan.getScanAngleIncrement() * i;
		if (degree >= min && degree <= max)
			distances.push_back(scan.getDistance(i));
	}
}

HamsterAPI::Hamster * Robot::getHamster()
{
	return leHamster;
}

// Destructor
Robot::~Robot()
{
	this->leHamster = NULL;
}

