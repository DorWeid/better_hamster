#include "LocalizationManager.h"

LocalizationManager::LocalizationManager(struct positionState startPos, HamsterAPI::OccupancyGrid *map)
{
	Particle * par = new Particle(startPos.pos.x,startPos.pos.y,startPos.yaw,1,map);
	this->listOfParticle.push_back(par);
}

void LocalizationManager::updateParticles(int deltaX, int deltaY, int deltaYaw, vector<double> sensorRead, double scanAngle, float maxRangeScan)
{
    // Normalization factor is here to keep the ratio between all the partcile, and to keep the belief be between 0 to 1
    double normalizationFactor = 0;
    std::list<Particle*> listOfNewParticle;
    
    // Run on all the particle, and update them according to the deltas
    for (std::list<Particle*>::iterator listIterator = this->listOfParticle.begin(); listIterator != this->listOfParticle.end(); listIterator++)
    {
        (*listIterator)->update(deltaX, deltaY, deltaYaw, sensorRead, scanAngle, maxRangeScan);
        double belCurParticle = (*listIterator)->getBel();
        
        normalizationFactor += belCurParticle;
    }
    
    std::list<Particle*>::iterator iterator = this->listOfParticle.begin();
    while (iterator != this->listOfParticle.end())
    {
    	(*iterator)->setBel((*iterator)->getBel() / normalizationFactor);

        if((*iterator)->getBel() < 0.01)
        {
          this->listOfParticle.erase(iterator++);
        }
        else if((*iterator)->getBel() > 0.03)
        {
        	// need to add the particle generated
            std::list<Particle*> listOfGenerateParticle = (*iterator)->generateParticle();

			for (std::list<Particle*>::iterator listIterator = listOfGenerateParticle.begin(); listIterator != listOfGenerateParticle.end(); listIterator++)
			{
				listOfNewParticle.push_back(*listIterator);
			}
            
            ++iterator;
        }
        else
        {
            ++iterator;
        }
    }
    
    for (std::list<Particle*>::iterator listIterator = listOfNewParticle.begin(); listIterator != listOfNewParticle.end(); listIterator++)
    {
        this->listOfParticle.push_back(*listIterator);
    } 
}
