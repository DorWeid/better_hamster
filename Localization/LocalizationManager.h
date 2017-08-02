#ifndef LOCALIZATION_MANAGER_H
#define LOCALIZATION_MANAGER_H
#include "../Particle/Particle.h"
#include <HamsterAPIClientCPP/Hamster.h>
#include <list>
using namespace std;
using namespace HamsterAPI;

class LocalizationManager
{
	public:
	  std::list<Particle*> listOfParticle;
	  LocalizationManager();
	  void updateParticles(int deltaX, int deltaY, int deltaYaw, vector<double> sensorRead, LidarScan lidarScan);
  
};

#endif
