#include "Particle.h"

Particle::Particle()
{
}

Particle::Particle(int posX, int posY, int yaw, double bel, HamsterAPI::OccupancyGrid *map)
{
	this->x = posX;
	this->y = posY;
	this->yaw = yaw;
	this->bel = bel;
	this->mapGrid = map;
	this->ogridResolution = this->mapGrid->getResolution();
	this->ogridHeight = this->mapGrid->getWidth();
	this->ogridWidth = this->mapGrid->getHeight();

}

// Update the Bel of the current particle according to his new Place, and the sensor 
void Particle::update(int deltaX, int deltaY, int deltaYaw, vector<double> laserTrace, LidarScan lidarScan)
{
  updateParticlePosition(deltaX, deltaY, deltaYaw);
  
  double preBel = this->bel * (this->probByMove(deltaX,deltaY,deltaYaw));
  
  // prebel * probByMes * N (normal that need to be find ! TODO)
  this->bel = preBel*probByMes(laserTrace, lidarScan);
}

// Get the belief of the particle
double Particle::getBel()
{
  return this->bel;
}

// Set a new belief to the particle
void Particle::setBel(double newBel)
{
	this->bel = newBel;
}

// Generate new particles according to the current particle
std::list<Particle*> Particle::generateParticle()
{
	std::list<Particle*> newListParticle;
	for(int i =0; i < 5; i++){
		Particle* newParticle = new Particle();
		
		do
		{
			int distX = 10 - rand() % 20;
			int distY = 10 - rand() % 20;
			newParticle->i = distX + this->i;
			newParticle->j = distY + this->j;

		} while (this->mapGrid->getCell(newParticle->i, newParticle->j) != CELL_FREE);
		
		newParticle->bel = this->bel;
		newParticle->mapGrid = this->mapGrid;
		newParticle->updateIndexToLocationOnMap();
		newListParticle.push_back(newParticle);
	}
	
  	return newListParticle;
}

// Get a probability according to the change of the X,Y and Angle
double Particle::probByMove(int deltaX, int deltaY, int deltaYaw)
{
  // If delta Z is low then the returned probability will be Strong
  // If delta X and Y are low too the same as Z     
  // 1 / (1 + Z) * (X + Y)^1/2     ?????
	
	return 1/((1+ deltaYaw) * sqrt(deltaX + deltaY));
}

// Get a probability according to the sensor
double Particle::probByMes(vector<double> laserTrace, LidarScan lidarScan)
{	
	int hits = 0;
	int misses = 0;
	int i= 0;
	
	for(std::vector<double>::reverse_iterator iterator = laserTrace.rbegin(); iterator != laserTrace.rend(); ++iterator) {
		
		double angle = lidarScan.getScanAngleIncrement() * i * DEG2RAD;

		if (lidarScan.getDistance(i) < lidarScan.getMaxRange() - 0.001){
			double obsX = this->x + *iterator * cos(angle + this->yaw * DEG2RAD- 180 * DEG2RAD);
			double obsY = this->y + *iterator * sin(angle + this->yaw * DEG2RAD- 180 * DEG2RAD);
			
			int k = (double) ogridHeight / 2 - obsY / ogridResolution;
			int j = obsX / ogridResolution + ogridWidth / 2;

			// Determine if there was a hit according to the grid
			if (this->mapGrid->getCell(k, j) == CELL_FREE)
			{
				misses++;
			}
			else if(this->mapGrid->getCell(k, j) == CELL_OCCUPIED)
			{
				hits++;
			}
			else
			{
				// not sure about the syntax...
				this->mapGrid->setCell(k,j, CELL_OCCUPIED);
				hits++;
			}
		}
		
		i++;
  	}	
  	
	double hitRate = (double) hits / (hits + misses);

	return hitRate;
}

void Particle::updateParticlePosition(int deltaX, int deltaY, int deltaYaw)
{
	this->x += deltaX;
	this->y += deltaY;

	this->yaw += deltaYaw;
	this->yaw = (this->yaw >= 360) ? this->yaw - 360 : this->yaw;
	this->yaw = (this->yaw < 0) ? this->yaw + 360 : this->yaw;
}

void Particle::updateMapLocationToIndex()
{
	this->i = (double) this->ogridHeight / 2 - this->y / this->ogridResolution;
	this->j = this->x / this->ogridResolution + (double) this->ogridWidth / 2;
	/*currParticle->i = currParticle->y * ogridResolution + (double)ogridHeight / 2;
	currParticle->j = currParticle->x * ogridResolution + (double)ogridWidth / 2;*/
}

void Particle::updateIndexToLocationOnMap()
{
	this->x = (this->j - (double) this->ogridWidth / 2) * this->ogridResolution;
	this->y = ((double) this->ogridHeight / 2 - this->i) * this->ogridResolution;
	/*particle->x = (2 * particle->j - (double) ogridWidth) / ogridResolution;
	particle->y = (2 * particle->i - (double) ogridHeight) / ogridResolution;*/
}
