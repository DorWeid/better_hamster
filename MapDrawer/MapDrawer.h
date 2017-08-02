/**
 * This class responsible to draw the map on the UI 
 */

#ifndef MAPDRAWER_MAPDRAWER_H_
#define MAPDRAWER_MAPDRAWER_H_

#include <HamsterAPIClientCPP/Hamster.h>
#include "../Node/Node.h"
#include "../NodeMap/NodeMap.h"
#include "../Helpers/Position.h"
#include "../Helpers/Angle.h"
#include "../Robot/Robot.h"
#include "../Particle/Particle.h"
#include "opencv2/imgproc.hpp"
#include <list>

using namespace std;
using namespace HamsterAPI;

enum MapPointType {
	Unknown,
	Free,
	Obstacle,
	Path,
	PathStart,
	PathEnd,
	Waypoint,
	ParticleType,
	LidarScanObstacle,
	GoodParticle,
	BadParticle
};

class MapDrawer {

public:
	MapDrawer(int width, int height);

	void SetPointType(int x, int y, MapPointType mapPointType);
	void DrawMap(OccupancyGrid* occupancyGridMap, double rotationAngle);
	void DrawNodeMap(NodeMap* nodeMap);
	void DrawPath(Node* goal);
	void DrawRobot(positionState pos, cv::Mat * map);
	void Show(positionState robotPos);
	void drawParticles(std::list<Particle*> particles);
	cv::Mat* getMap();
	//void DrawLidarScan(std::vector<positionState> obstacles);
private:
	const string WINDOW_TITLE;
	cv::Mat* _map;
	void SetPointColor(int x, int y, int red, int green, int blue);

};

#endif /* MAPDRAWER_MAPDRAWER_H_ */
