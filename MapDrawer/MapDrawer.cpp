#include "../MapDrawer/MapDrawer.h"

MapDrawer::MapDrawer(int width, int height) : WINDOW_TITLE("Room-Map")
{
	cv::namedWindow("Room-Map");
	_map = new cv::Mat(width, height,CV_8UC3,cv::Scalar(0,0,0));

}

// Show on the cv the drawed map
void MapDrawer::Show(positionState robotPos)
{
	cv::Point2f center(ROBOT_START_X,ROBOT_START_Y);
	cv::Mat resultMap;
	cv::Mat scaled;

	cv::Mat tempMap;
	scaled = cv::getRotationMatrix2D(center, 0, 1.0);
	cv::warpAffine(*_map, tempMap, scaled, _map->size());

	DrawRobot(robotPos,&tempMap);

	scaled = cv::getRotationMatrix2D(center, 0, 2.0);
	cv::warpAffine(tempMap, resultMap, scaled, _map->size());

	cv::imshow(MapDrawer::WINDOW_TITLE, resultMap);
	cv::waitKey(100);
}

cv::Mat* MapDrawer::getMap()
{
	return _map;
}

// Draw the grid with a rotation angle
void MapDrawer::DrawMap(OccupancyGrid* occupancyGridMap, double rotationAngle)
{
	int width = occupancyGridMap->getWidth();
	int height = occupancyGridMap->getHeight();

	/*
	for (int x = 0; x < height; x++)
	{
		 for (int y = 0; y < width; y++)
		 {
			MapDrawer::SetPointType(x,y, Free);
		 }
	} */

	// Run over all pixel of the map
	for (int x = 0; x < height; x++)
	{
	     for (int y = 0; y < width; y++)
	     {
		     // Set the point type according to the cell of the grid
			  if (occupancyGridMap->getCell(x, y) == CELL_FREE)
			  {
				  MapDrawer::SetPointType(x,y, Free);
			  }
			  else if (occupancyGridMap->getCell(x, y) == CELL_OCCUPIED)
			  {
				  MapDrawer::SetPointType(x,y, Obstacle);
			  }
			  else
			  {
				  MapDrawer::SetPointType(x,y, Unknown);
			  }

	     }
	}

	// Rotate the map according to the parameter
	if(rotationAngle != 0)
	{
		rotateMapOnOrigin(_map, _map, rotationAngle);
	}
}

// Draw a Node map
void MapDrawer::DrawNodeMap(NodeMap* nodeMap)
{
	int width = nodeMap->getWidth();
	int height = nodeMap->getHeight();

	// Run over all the pixel on the map
	for (int y = 0; y < height; y++) {
	     for (int x = 0; x < width; x++) {
		     
	      // Check if the node at this point is a obstacle
	      if (nodeMap->getNodeByCoordinates(y,x)->isObstacle)
	      {
	    	  MapDrawer::SetPointType(x, y, Obstacle);
	      }
	   }

	}
}

// Draw robot as a circle on a given map
void MapDrawer::DrawRobot(positionState pos, cv::Mat * map)
{
	float robot_i, robot_j;
	robot_i = ROBOT_START_Y + pos.pos.y;
	robot_j = pos.pos.x + ROBOT_START_X;
	cv::Scalar_<int> * color = new cv::Scalar_<int>(255,0,0);
	cv::Point_<float>* position = new cv::Point_<float>(robot_j,robot_i);
	cv:circle(*map, *position,4,*color,1,8,0);
}

// Draw a path
void MapDrawer::DrawPath(Node* goal)
{
	// Set the given node to be the end path
	MapDrawer::SetPointType(goal->y , goal->x , PathEnd);

	Node* currentNode = goal->parent;
	
	// Run over all the path
	while(currentNode != NULL)
	{
		Node* nextNode = currentNode->parent;
		
		// If the next node was null, so the curr node is the start point
		if(nextNode == NULL) {
			MapDrawer::SetPointType(currentNode->y ,currentNode->x  , PathStart);
		}
		// If the node is a waypoint
		else if(currentNode->isWaypoint)
		{
			MapDrawer::SetPointType(currentNode->y ,currentNode->x , Waypoint);
		}
		else
		{
			MapDrawer::SetPointType(currentNode->y ,currentNode->x , Path);
		}

		currentNode = nextNode;
	}

	//MapDrawer::SetPointType(goal->y ,goal->x , PathStart);
}

// Draw given particles on map
void MapDrawer::drawParticles(std::list<Particle*> particles)
{
	double bestParticalesAvrageBelief = 0;
	double particalesCounter = 0 ;

	for (std::list<Particle*>::iterator listIterator = particles.begin(); listIterator != particles.end(); listIterator++)
	{
		if ((*listIterator)->bel > 0.02)
		{
			MapDrawer::SetPointType((*listIterator)->y ,(*listIterator)->x, GoodParticle);
			bestParticalesAvrageBelief += (*listIterator)->bel;
			particalesCounter++;
		}
		else
		{
			MapDrawer::SetPointType((*listIterator)->y ,(*listIterator)->x, BadParticle);
		}
	}
}

// Set the color of a point according to his point type
void MapDrawer::SetPointType(int x, int y, MapPointType mapPointType)
{
	switch(mapPointType)
	{
		case(Unknown) :
		{
			MapDrawer::SetPointColor(x, y, 128, 128, 128);
			break;
		}
		case(Free) :
		{
			MapDrawer::SetPointColor(x, y, 255, 255, 255);
			break;
		}
		case(Obstacle) :
		{
			MapDrawer::SetPointColor(x, y, 0, 0, 0);
			break;
		}
		case(PathStart) : {
			MapDrawer::SetPointColor(x, y, 0, 0, 255);
			break;
		}
		case(PathEnd): {
			MapDrawer::SetPointColor(x, y, 0, 255, 0);
			break;
		}
		case(Path) :
		{
			MapDrawer::SetPointColor(x, y, 255, 0, 0);
			break;
		}
		case(Waypoint) :
		{
			MapDrawer::SetPointColor(x, y, 255, 255, 0);
			break;
		}
		case(ParticleType) :
		{
			MapDrawer::SetPointColor(x, y, 0, 0, 255);
			break;
		}
		case(LidarScanObstacle) :
		{
			MapDrawer::SetPointColor(x, y, 255, 0, 255);
			break;
		}
		case(GoodParticle) :
		{
			MapDrawer::SetPointColor(x, y,0, 255, 0);
			break;
		}
		case(BadParticle) :
		{
			MapDrawer::SetPointColor(x, y, 0,255, 0);
			break;
		}

	}
}

//Change the color of a point
void MapDrawer::SetPointColor(int x, int y, int red, int green, int blue)
{
	if(x >= 0 && y >= 0)
	{
		MapDrawer::_map->at<cv::Vec3b>(x, y)[0] = blue;
		MapDrawer::_map->at<cv::Vec3b>(x, y)[1] = green;
		MapDrawer::_map->at<cv::Vec3b>(x, y)[2] = red;
	}
}


