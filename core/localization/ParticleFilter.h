#pragma once

#include <math/Pose2D.h>
#include <memory/MemoryCache.h>
#include <memory/LocalizationBlock.h>
#include <localization/Logging.h>

#define numOfParticles = 3000;  // number of particles

class ParticleFilter {
  public:
    ParticleFilter(MemoryCache& cache, TextLogger*& tlogger);
    void init(Point2D loc, float orientation);
    void processFrame();
    void NeedUpdate();
    const Pose2D& pose() const;
    inline const std::vector<Particle>& particles() const {
      return cache_.localization_mem->particles;
    }

  protected:
    inline std::vector<Particle>& particles() {
      return cache_.localization_mem->particles;
    }

  private:
    void PFAlgorithm();
    Particle& sampling(String& ut, Particle& xm);
    float getWeight(Particle & x); 
    Particle& resampling(std::vector<Particle>& particles, 
                float (&weights)[numOfParticles]);


    MemoryCache& cache_;
    TextLogger*& tlogger_;

    mutable Pose2D mean_;
    mutable bool dirty_;
    mutable bool backToRandom;
    mutable bool needToUpdate;
    

    String Controls[6] = ['forward', 
                          'backward', 
                          'left', 
                          'right', 
                          'turn_left',
                          'turn_right'
                          ]
};
