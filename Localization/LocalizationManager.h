#ifndef LOCALIZATION_MANAGER_H
#define LOCALIZATION_MANAGER_H
#include "../Particle/Particle.h"
#include "../Helpers/Structures.h"

#include <HamsterAPIClientCPP/Hamster.h>
#include <list>
using namespace std;
using namespace HamsterAPI;

class LocalizationManager
{
	public:
	  std::list<Particle*> listOfParticle;
	  LocalizationManager(struct positionState startPositionState, HamsterAPI::OccupancyGrid *map);
	  void updateParticles(int deltaX, int deltaY, int deltaYaw, vector<double> sensorRead, double scanAngle, float maxRangeScan);
  
};

#endif
