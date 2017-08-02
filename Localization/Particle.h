#ifndef PARTICLE_H_
#define PARTICLE_H_

#include <HamsterAPICommon/Messages/OccupancyGrid.h>
#include <HamsterAPIClientCPP/Hamster.h>
#include <list>
using namespace std;
using namespace HamsterAPI;
class Particle {

public:
  // Location of the Particle on the grid
  int i;
  int j;
  
  int x;
  int y;
  int yaw;
  double bel;
 
  Particle();
  Particle(int posX, int posY, int yaw, double bel, HamsterAPI::OccupancyGrid *map);
  
  HamsterAPI::OccupancyGrid *mapGrid;
    
  void update(int deltaX, int deltaY, int deltaYaw, vector<double> laserTrace, LidarScan lidarScan);
  
  double getBel();
  void setBel(double newBel);
  
  std::list<Particle*> generateParticle();
  void updateMapLocationToIndex();
  void updateIndexToLocationOnMap();
	
private:
  int ogridHeight, ogridWidth;
	double ogridResolution;
  
  double probByMove(int deltaX, int deltaY, int deltaYaw);
  
  // TODO:: laser trace must be a different thing than a int
  double probByMes(vector<double> laserTrace, double scanAngle, float maxRangeScan);
  
  void updateParticlePosition(int deltaX, int deltaY, int deltaYaw);
};

#endif
