/**
 * Represents the current map using Nodes
 * Each coordinate is a node
 */

#ifndef MAP_H_
#define MAP_H_

#include <vector>
#include <HamsterAPIClientCPP/Hamster.h>
#include "../Node/Node.h"
#include "../Helpers/Constants.h"
#include "../Helpers/Position.h"
#include "../Helpers/Structures.h"

using namespace std;
using namespace HamsterAPI;

class NodeMap
{
	public:
		// Constructor
		NodeMap();

		// Size of map (amount of nodes in row / column)
		int getWidth() const;
		int getHeight() const;

		// Retrieve the node at the given coordinates
		Node* getNodeByCoordinates(int x, int y) const;
		void loadMap(cv::Mat* roomRealMapFromMemory);
		void loadBlowMap(cv::Mat* roomRealMapFromMemory);
		NodeMap getBlownNodeMap();
		void resizeMap(int pixelsPerCell, NodeMap* output) const;
		bool isAreaAnObstacle(int colIndex, int rowIndex, int resolution) const;
		void colorArea(unsigned width, unsigned height,	struct position pos, int r, int g, int b);

		// Destructor
		~NodeMap();

	private:
		// Represents the map as an array of node vectors
		vector<vector<Node*> > _matrix;

		// Calculates the required blowing size
		int calcBlowSize(double robotSizeCm, double resolutionCm);

		// Get the blown rectangle "wrapping" the given coordinates
		rectangle getCurrentRectangle(int blowRange, unsigned currX, unsigned currY, unsigned width, unsigned height);
};

#endif
