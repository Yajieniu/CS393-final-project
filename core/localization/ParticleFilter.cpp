#include <localization/ParticleFilter.h>
#include <memory/FrameInfoBlock.h>
#include <memory/OdometryBlock.h>
#include <common/Random.h>

ParticleFilter::ParticleFilter(MemoryCache& cache, TextLogger*& tlogger) 
  : cache_(cache), tlogger_(tlogger), dirty_(true) {
    M = 100;
    particles().resize(M); // TODO: clear in destructor?
}

void ParticleFilter::init(Point2D loc, float orientation) {
  mean_.translation = loc;
  mean_.rotation = orientation;
}

void ParticleFilter::processFrame() {
  // Indicate that the cached mean needs to be updated
  dirty_ = true;

  // Beacons World Locations
  static map<WorldObjectType, Point2D> beaconLocation = {
      { WO_BEACON_BLUE_YELLOW,    {1500, -1000} },
      { WO_BEACON_YELLOW_BLUE,    {1500, 1000} },
      { WO_BEACON_BLUE_PINK,      {0, -1000} },
      { WO_BEACON_PINK_BLUE,      {0, 1000} },
      { WO_BEACON_PINK_YELLOW,    {-1500, -1000} },
      { WO_BEACON_YELLOW_PINK,    {-1500, 1000} }
  };

  // Retrieve odometry update - how do we integrate this into the filter?
  const auto& disp = cache_.odometry->displacement;
  log(41, "Updating particles from odometry: %2.f,%2.f @ %2.2f", disp.translation.x, disp.translation.y, disp.rotation * RAD_T_DEG);
  
  // Action update
  for (auto& particle : particles() ) {
    particle.x += disp.translation.x;
    particle.y += disp.translation.y;
    particle.t += disp.rotation;
  }

  // Sensor Update (one for each Beacons)
  for ( const auto &beacon : beaconLocation ) {
    auto& object =  cache_.world_object->objects_[beacon.first];
    if ( object.seen == false )
      continue;

    // write particle filter code hear using the beacon location

  }

  // Generate random particles for demonstration
  // particles().resize(100);
  // auto frame = cache_.frame_info->frame_id;
  // for(auto& p : particles()) {
  //   p.x = Random::inst().sampleN() * 250 + (frame * 5); //static_cast<int>(frame * 5), 250);
  //   p.y = Random::inst().sampleN() * 250; // 0., 250);
  //   p.t = Random::inst().sampleN() * M_PI / 4;  //0., M_PI / 4);
  //   p.w = Random::inst().sampleU();
  // }
}

const Pose2D& ParticleFilter::pose() const {
  if(dirty_) {
    // Compute the mean pose estimate
    mean_ = Pose2D();
    using T = decltype(mean_.translation);
    for(const auto& p : particles()) {
      mean_.translation += T(p.x,p.y);
      mean_.rotation += p.t;
    }
    if(particles().size() > 0)
      mean_ /= static_cast<float>(particles().size());
    dirty_ = false;
  }
  return mean_;
}
