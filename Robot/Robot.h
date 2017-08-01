/**
 * This class represents the Hamster robot.
 * Responsible of the robot's movement, location and everything related to the robot's API should be used through here
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include <HamsterAPIClientCPP/Hamster.h>
#include <math.h>
#include "../NodeMap/Node.h"
#include "../NodeMap/NodeMap.h"
#include "../Gui/MapDrawer.h"
#include "../Helpers/Constants.h"
#include "../Helpers/AngleUtils.h"
#include "../Helpers/Structures.h"
#include "../Localization/LocalizationManager.h"

using namespace HamsterAPI;
using namespace std;

class Robot
{
	public:
		positionState prevBeliefedLocation, currBeliefedLocation, realLocation;

		// Constructor
		Robot(Hamster * h, LocalizationManager * locMng, double height, double width, double res);

		// Destructor
		~Robot();

		// Get hamster location
		positionState getRealHamsterLocation();
		void updateHamsterRealLocation();
		void setHamsterSpeed(double speed, double wheelsAngle);


	private:
		// The robot instance. Should be initialized once!
		Hamster * leHamster;

		// Localization manager instance
		LocalizationManager * localizationManager;

		//
		double resolution, mapHeight, mapWidth;
};

#endif /* ROBOT_H_ */
