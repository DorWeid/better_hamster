#ifndef LOCALIZATION_MANAGER_H
#define LOCALIZATION_MANAGER_H
#include "../Particle/Particle.h"
#include "../Helpers/Structures.h"
#include <HamsterAPIClientCPP/Hamster.h>
#include <list>

using namespace HamsterAPI;
using namespace std;

class LocalizationManager
{

	public:
	  std::list<Particle*> listOfParticle;

	  // Constructor
	  LocalizationManager(struct positionState startPositionState, HamsterAPI::OccupancyGrid *map);

	  // Update particles on screen
	  void updateParticles(int deltaX, int deltaY, int deltaYaw, vector<double> sensorRead, double scanAngle, float maxRangeScan);
};

#endif
