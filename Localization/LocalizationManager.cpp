#include "LocalizationManager.h"

LocalizationManager::LocalizationManager()
{
}

void LocalizationManager::updateParticles(int deltaX, int deltaY, int deltaYaw, vector<double> sensorRead, LidarScan lidarScan)
{
    // Normalization factor is here to keep the ratio between all the partcile, and to keep the belief be between 0 to 1
    double normalizationFactor = 0;
    std::list<Particle*> listOfNewParticle;
    
    // Run on all the particle, and update them according to the deltas
    for (std::list<Particle*>::iterator listIterator = this->listOfParticle.begin(); listIterator != this->listOfParticle.end(); listIterator++)
    {
        (*listIterator)->update(deltaX, deltaY, deltaYaw, sensorRead, lidarScan);
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
           listOfNewParticle.push_back((*iterator)->generateParticle());
            ++iterator;
        }
        else
        {
            ++iterator;
        }
    }
    
   // this->listOfParticle.push_back(listOfNewParticle);
}
