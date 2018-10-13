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
    const Pose2D& pose() const;
    inline const std::vector<Particle>& particles() const {
      return cache_.localization_mem->particles;
    }

  protected:
    inline std::vector<Particle>& particles() {
      return cache_.localization_mem->particles;
    }

  private:
    void RandomParticleMCL();
    Particle& sample_motion_model(Particle& xtm, auto& disp, Particle& xm);
    Particle& randPose();
    float getWeight(Particle & x); 
    Particle& resampling(std::vector<Particle>& particles, 
                float (&weights)[numOfParticles]);


    MemoryCache& cache_;
    TextLogger*& tlogger_;

    mutable Pose2D mean_;
    mutable bool dirty_;
    mutable bool backToRandom = false;
    mutable bool needToUpdate;
    float w_slow;
    float w_fast;
    float a_slow;
    float a_fast;
    float X_MIN = -2500.0;
    float X_MAX = 2500.0;
    float Y_MIN = -1250.0;
    float Y_MAX = 1250.0;
    
    // Beacons World Locations
    static map<WorldObjectType, Point2D> beaconLocation = {
      { WO_BEACON_BLUE_YELLOW,    {1500, -1000} },
      { WO_BEACON_YELLOW_BLUE,    {1500, 1000} },
      { WO_BEACON_BLUE_PINK,      {0, -1000} },
      { WO_BEACON_PINK_BLUE,      {0, 1000} },
      { WO_BEACON_PINK_YELLOW,    {-1500, -1000} },
      { WO_BEACON_YELLOW_PINK,    {-1500, 1000} }
    };
};
