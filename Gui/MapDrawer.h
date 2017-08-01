#ifndef GUI_MAPDRAWER_H_
#define GUI_MAPDRAWER_H_

#include <HamsterAPIClientCPP/Hamster.h>
#include "../Node/Node.h"
#include "../NodeMap/NodeMap.h"
#include "../Localization/LocalizationParticle.h"
#include "../Helpers/Position.h"
#include "../Robot/Robot.h"
#include "opencv2/imgproc.hpp"

#include "../Helpers/Angle.h"
using namespace HamsterAPI;
using namespace std;

enum MapPointType {
	Unknown,
	Free,
	Obstacle,
	Path,
	PathStart,
	PathEnd,
	Waypoint,
	Particle,
	LidarScanObstacle,
	GoodParticle,
	BadParticle
};

class MapDrawer {
private:
	const string WINDOW_TITLE;
	cv::Mat* _map;
	void SetPointColor(int x, int y, int red, int green, int blue);
public:
	MapDrawer(int width, int height);

	void SetPointType(int x, int y, MapPointType mapPointType);
	void DrawMap(OccupancyGrid* occupancyGridMap, double rotationAngle);
	void DrawNodeMap(NodeMap* nodeMap);
	void DrawPath(Node* goal);
	void DrawRobot(positionState pos, cv::Mat * map);
	void Show(positionState robotPos);
	double DrawPatricles(std::vector<LocalizationParticle *>* particles);
	cv::Mat* getMap();
	//void DrawLidarScan(std::vector<positionState> obstacles);
};

#endif /* GUI_MAPDRAWER_H_ */
