#include <HamsterAPIClientCPP/Hamster.h>
#include <iostream>
#include "NodeMap/NodeMap.h"
#include "PathPlanner/PathPlanner.h"
#include <stdlib.h>
#include "MapDrawer/MapDrawer.h"
#include "Localization/LocalizationManager.h"
#include "Robot/Robot.h"
#include "MovementManager/MovementManager.h"
#include "math.h"

#include "Helpers/Position.h"
#include "Helpers/Position.h"
#include "Helpers/Structures.h"

using namespace std;
using namespace HamsterAPI;

Robot * rRobot = new Robot();

void startRobot();

int main()
{
	try
	{
		// Start the robot
		startRobot();
	}
	catch (const HamsterAPI::HamsterError & connection_error)
	{
		HamsterAPI::Log::i("Client", connection_error.what());
	}

	return 0;
}

// Check if the given waypoint was reached
bool isWaypointReached(const positionState& locationRobot, const Node& locationWaypoint) {
	// Check distance between current location to the waypoint location
	double distanceFromWaypoint = sqrt(
			pow(locationRobot.pos.x - locationWaypoint.x, 2)
					+ pow(locationRobot.pos.y - locationWaypoint.y, 2));

	// Return if the distance is smaller than the tolerance distance
	return distanceFromWaypoint <= DISTANCE_FROM_WAYPOINT_TOLERANCE;
}

// Main function of the robot
void startRobot()
{
	NodeMap roomRealMap;
	NodeMap roomBlownMap;
	PathPlanner *pathPlanner;

	// Get the map of the hamster
	OccupancyGrid roomRealMapFromMemory = rRobot->getHamster()->getSLAMMap();
	// Send to the map drawer the map give by the hamster
	MapDrawer* mapDrawer = new MapDrawer(roomRealMapFromMemory.getWidth(), roomRealMapFromMemory.getHeight());

	// Draw the map
	mapDrawer->DrawMap(&roomRealMapFromMemory, 0);

	// Create a mat in the size of the grid
	cv::Mat* drawedMap = new cv::Mat(roomRealMapFromMemory.getWidth(), roomRealMapFromMemory.getHeight(),CV_8UC3,cv::Scalar(0,0,0));

	// Rotate according to the MAP_ROTATION const, the grid into the created mat
	rotateMapOnOrigin(mapDrawer->getMap(), drawedMap, MAP_ROTATION);

	// Init the real map with the mat
	roomRealMap.loadMap(drawedMap);

	// Init a blown map according to the resolution of the map, and size of the robot  with the mat
	roomBlownMap.loadBlownMap(drawedMap);

	// Init the robot start and goal positions, and get is goal and start position in the Blown map
	Node *startPos = roomBlownMap.getNodeByCoordinates(ROBOT_START_X, ROBOT_START_Y);
	Node *goalPos = roomBlownMap.getNodeByCoordinates(GOAL_X, GOAL_Y);

	// Check if the goal is an obstacle
	if (roomBlownMap.getNodeByCoordinates(goalPos->x, goalPos->y)->isObstacle)
	{
		cout << "Goal is obstacle, cannot proceed." << endl;
		return;
	}

	// Send to the path planner the blown map, start positon and goal position and get from him the path
	pathPlanner->calculatePath(&roomBlownMap,startPos,goalPos);

	// Get the waypoint on the path
	std::list<Node* > waypoints = pathPlanner->getWaypoints(startPos, goalPos);

	// Draw the map
	mapDrawer->DrawMap(&roomRealMapFromMemory, MAP_ROTATION);

	// Draw the blown map
	mapDrawer->DrawNodeMap(&roomBlownMap);

	// Draw the goal position
	mapDrawer->DrawPath(goalPos);

	// Get the position of the robot on the hamster grid
	Pose robotStartPose = rRobot->getHamster()->getPose();
	
	struct position startPosition = {.x =
		 ROBOT_START_X + robotStartPose.getX(), .y = ROBOT_START_Y -  robotStartPose.getX()};

	struct positionState startPositionState = {.pos = startPosition, .yaw = robotStartPose.getHeading()};

	// Get the map resolution
	double mapResolution = roomRealMapFromMemory.getResolution();

	
	// TO DO: CHANGE LOCALIZATION MANAGER TO USE PARTICLE
	LocalizationManager * localizationManager = new LocalizationManager(startPositionState, &roomRealMapFromMemory);

	rRobot->localizationManager = localizationManager;
	rRobot->mapHeight = roomRealMapFromMemory.getHeight();
	rRobot->mapWidth = roomRealMapFromMemory.getWidth();
	rRobot->resolution = mapResolution;

	MovementManager movementManager(rRobot, mapDrawer);

	// If the robot is conected
	if(rRobot->getHamster()->isConnected())
	{
		int w = 1;
		// Run on all the waypoint list
		for (std::list<Node*>::reverse_iterator iter = waypoints.rbegin(); iter != waypoints.rend(); ++iter)
		{
			Node* currWaypoint = *iter;

			Node hamsterWaypoint = ConvertToHamsterLocation(currWaypoint);
			rRobot->realLocation = rRobot->currBeliefedLocation = rRobot->getRealHamsterLocation();

			if (isWaypointReached(rRobot->currBeliefedLocation, hamsterWaypoint))
			{
				cout << endl << "Reached waypoint #" << w << ": (" << hamsterWaypoint.x << ", " << hamsterWaypoint.y << ")" << endl << endl;
				w++;
			}
			else
			{
				movementManager.NavigateToWaypoint(&hamsterWaypoint);
			}
		}
	
		 cout << "Reached goal after " << w << " waypoints: (" << GOAL_X << ", " << GOAL_Y << ")" << endl;
	}
}



