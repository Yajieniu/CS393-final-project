#include <localization/ParticleFilter.h>
#include <memory/FrameInfoBlock.h>
#include <memory/OdometryBlock.h>
#include <memory/WorldObjectBlock.h>
#include <common/Random.h>
#include <common/WorldObject.h>
#include <assert.h>
#include <math.h>

/*
Assuming theta = 0 when we are facing away from the goal
i.e.,, robot's back towards the goal.
*/


// Beacons World Locations
static map<WorldObjectType, Point2D> beaconLocation = {
  { WO_BEACON_BLUE_YELLOW,    {1500, 1000} },
  { WO_BEACON_YELLOW_BLUE,    {1500, -1000} },
  { WO_BEACON_BLUE_PINK,      {0, 1000} },
  { WO_BEACON_PINK_BLUE,      {0, -1000} },
  { WO_BEACON_PINK_YELLOW,    {-1500, 1000} },
  { WO_BEACON_YELLOW_PINK,    {-1500, -1000} }
};

/* 
 * Create an instance of class Particle Filter. 
 * Or to say create a particle filter. 
 * Only needed to be called once at the beginning. 
 */
ParticleFilter::ParticleFilter(MemoryCache& cache, TextLogger*& tlogger) 
  : cache_(cache), tlogger_(tlogger), dirty_(true),
  backToRandom(true){
    w_slow = 0;
    w_fast = 0;
    a_slow = 0.001;
    a_fast = 0.9;

}

/* 
 * Initialize the position of our robot. 
 * Location and orientation are given.
 */
void ParticleFilter::init(Point2D loc, float orientation) {
  mean_.translation = loc;
  mean_.rotation = orientation;
}

/* 
 * Update particles. 
 * But this function is different from the Particle Filter 
 * algorithm. 
 */
void ParticleFilter::processFrame() {
  // Indicate that the cached mean needs to be updated
  dirty_ = true;
  
  // TEMP printing stuff
  for (const auto& beacon : beaconLocation) {
    const auto& object = cache_.world_object->objects_[beacon.first];
    if ( object.seen == false )
      cout << getName(beacon.first) << " not seen.\n\n";
    else {
      cout << getName(beacon.first) << " seen.\n";
      cout << "At distance : " << object.visionDistance << endl;
      cout << "At bearing : " << object.visionBearing << endl;
    }
  }
  // We have a fixed number of particles.
  cout << "Current size of particles() = " << particles().size() << endl;

  if (backToRandom) {
    particles().resize(numOfParticles); 
    auto frame = cache_.frame_info->frame_id;
    for(auto& p : particles()) {
      p.x = Random::inst().sampleU() * X_MAX * 2 - X_MAX ;
      p.y = Random::inst().sampleU() * Y_MAX * 2 - Y_MAX;
      p.t = Random::inst().sampleU() * 360 - 180;
      p.w = 1;
    }
    backToRandom = false;
  }
  else {
    for (auto p : particles()) {
      // cout <<"("<<p.x<<", "<<p.y<<", "<<p.t<<", "<<p.w<<")\n";
      break;
    }
    RandomParticleMCL();
  }
}

/* 
 * The Implementation of the random particle MCL algorithm. 
 */
void ParticleFilter::RandomParticleMCL() {

  // Getting the last particle set
  std::vector<Particle> particles = ParticleFilter::particles();
  std::vector<Particle> X;
  std::vector<Particle> X1;
  Particle tempP;
  float weights[numOfParticles] = {};
  float totalWeight = 0.0;
  float w_avg = 0.0;
  float randNumber;

  // Retrieve odometry update - how do we integrate this into the filter?
  // This is the observation, z_t
  // contains x, and y and theta
  // But might need further translation
  const auto& disp = cache_.odometry->displacement;
  log(41, "Updating particles from odometry: %2.f,%2.f @ %2.2f", disp.translation.x, 
            disp.translation.y, disp.rotation * RAD_T_DEG); 

  // First part of the algorithm
  for (int i = 0; i < numOfParticles; i++) {
    tempP = sample_motion_model(tempP, disp, particles[i]); 
    assert(weights[i] == 0);
    totalWeight += getWeight(tempP);
    X.push_back(tempP);
    w_avg = w_avg + 1/numOfParticles * tempP.w;
  }

  w_slow = w_slow + a_slow * (w_avg - w_slow);
  w_fast = w_fast + a_fast * (w_avg - w_fast);

  // Instead of normalizing the weight, here is getting 
  // the sum of all weights
  weights[0] = X[0].w;
  for (int i = 1; i < numOfParticles; i++) {
    assert(X[i].w >= 0);
    weights[i] = weights[i-1] + X[i].w;
  }

  // Second part of the algorithm

  
  float counter = 0.0;  // count how many particles we should resample
  for (int i = 0; i < numOfParticles; i++) {
    randNumber = Random::inst().sampleU();
    if (randNumber <= std::max(0.0, 1.0 - w_fast/w_slow)) {
      X1.push_back(ParticleFilter::randPose(tempP));
    }
    else {
      counter += 1.0;   
    }
  } 

  // 1: Roulette wheel, resample according to random number
  // 2: Systematic resampling, low variance
  int resample_version = 2;  // 1: more random, 2: morn even
  int newParticleIndex = 0;
  assert(resample_version == 1 || resample_version == 2);  // must be 1 or 2

  for (int i = 0; i < counter; i++) {
    newParticleIndex = floor(i * counter / numOfParticles);  // not used if resample_version is 1
    X1.push_back(ParticleFilter::resampling(tempP, particles, weights, totalWeight, resample_version, newParticleIndex));
  }
  
  // store the result back into memory
  // is this the way to save to cache??????????
  cache_.localization_mem->particles = X1;
}

/* 
 * Generate a random particle. 
 * random x, random y, and random theta.
 * weight is 0 since it will not be used anywhere.
 */
Particle& ParticleFilter::randPose(Particle& p) {
  p.x = Random::inst().sampleU() * X_MAX * 2 - X_MAX;
  p.y = Random::inst().sampleU() * Y_MAX * 2 - Y_MAX;
  p.t = Random::inst().sampleU() * 360 - 180;
  p.w = 0;  // will not be used

  return p;
}

/*  
 * Algorithm line 9. Resample a new particle. 
 * Generate a random number to pick one existed particle p,
 * the new particle wil fall into a small range around p, 
 * and have a similar theta.
 */
Particle& ParticleFilter::resampling(Particle& newP, std::vector<Particle>& particles, 
  float weights[], float totalWeight, int version, int newParticleIndex) {

  int i = 0;
  // assert(weights[0] == 0); // Abheek: Why should it be 0? It was set to X[0].w
  assert(version == 1 || version == 2);

  if (version == 1) {
    float randNumber = Random::inst().sampleU() * totalWeight;
    assert(randNumber <= weights[numOfParticles-1]);
    
    // resampled particle should be close to particles[i-1]
    while(randNumber >= weights[i]) {
      i++;
    }
  }
  else {  // version 2, Systematic resampling
    i = newParticleIndex + 1;
  }

  assert(i>0);

  std::vector<float> x_range;
  std::vector<float> y_range;
  std::vector<float> t_range;

  // the location of the newly sample particle falls in this range
  float range = 2.2;
  float tRange = 1.8;
  x_range.push_back(std::max(X_MIN, particles[i-1].x-range));
  x_range.push_back(std::min(X_MAX, particles[i-1].x+range)); 
  y_range.push_back(std::max(Y_MIN, particles[i-1].y-range));
  y_range.push_back(std::min(Y_MAX, particles[i-1].y+range));
  t_range.push_back(std::max((float) -180, particles[i-1].t-tRange));
  t_range.push_back(std::min((float) 180, particles[i-1].t-tRange));


  // determine the location of new particle
  newP.x = Random::inst().sampleU() * 
               (x_range[1] - x_range[0]) + x_range[0];

  newP.y = Random::inst().sampleU() * 
               (y_range[1] - y_range[0]) + y_range[0];

  newP.t = Random::inst().sampleU() * 
               (t_range[1] - t_range[0]) + t_range[0];

  newP.w = 0;  // whatever, not used in the future

  return newP;
}

/* Algorithm line 6. Get weights for each new sample. */
float ParticleFilter::getWeight(Particle & p) {
  p.w = 1;

  // TODO: set variance for gaussian, and may need to debug to adjust signs.
  // Also haven't compiled because other parts not complete. May have error.
  for (const auto& beacon : beaconLocation) {
    const auto& object = cache_.world_object->objects_[beacon.first];
    if ( object.seen == false )
      continue;

    float dist = sqrt( (beacon.second.x - p.x)*(beacon.second.x - p.x)
                + (beacon.second.y - p.y)*(beacon.second.y - p.y) );
    p.w *= gaussianPDF ( object.visionDistance, dist, 1000 ); // TODO: may need to change sigma for real robot

    /*
      We have to decide what is the best way to work with theta from
      -pi to +pi, while functions like tanh() return -pi/2 to pi/2.
      Currently setting everything to (-pi/2, pi/2], which is incorrect
      if we have error of exactly pi!!!
    */

    /*
    float world_theta = tanh((beacon.second.y - x.y) / (beacon.second.x - x.x));
    while (world_theta <= -M_PI/2) world_theta += M_PI;
    while (world_theta > M_PI/2) world_theta -= M_PI;

    // TODO: May have to debug to use correct sign.
    float relative_theta = x.t + object.visionBearing;
    while (relative_theta <= -M_PI/2) relative_theta += M_PI;
    while (relative_theta > M_PI/2) relative_theta -= M_PI;
    */

    float beacon_theta = atan((beacon.second.y - p.y) / (beacon.second.x - p.x));

    assert(beacon_theta >= -M_PI && beacon_theta <= M_PI);    

    // assuming visionBearing is from -pi/2 to pi/2
    float beacon_bearing = object.visionBearing;
    assert(beacon_bearing >= -M_PI && beacon_bearing <= M_PI);

    float theta = beacon_theta - beacon_bearing;

    if (beacon.second.x - p.x < 0 && beacon.second.y - p.y < 0) {
      theta -= M_PI;
    }

    if (beacon.second.x - p.x < 0 && beacon.second.y - p.y > 0) {
      theta += M_PI;
    }

    // Print check
    // cout << atan(beacon.second.y / (beacon.second.x + 0.1) ) * RAD_T_DEG << ", " << endl;
    // cout <<"("<<p.x<<", "<<p.y<<", "<<p.t<<", "<<p.w<<") ";
    // cout << beacon_theta*RAD_T_DEG << ", " << beacon_bearing*RAD_T_DEG << ", " << p.t <<", "<<theta*RAD_T_DEG<< endl;

    p.w *= gaussianPDF (p.t, theta * RAD_T_DEG, 100 );
    // if (p.w > 0) cout << "Got a particle with non zero weight "<<"("<<p.x<<", "<<p.y<<", "<<p.t<<", "<<p.w<<")\n";
  }

  // If no beacon seen, then everyone gets weight 1
  return p.w;
}

inline float ParticleFilter::gaussianPDF( float x, float mu, float sigma = 100) {
  return (1. / sqrt(2*M_PI*sigma*sigma)) * exp(- ((x-mu)*(x-mu)) / (2*sigma*sigma));
}


/* 
 * Algorithm line 5. 
 * Get new estimated samples from the motion. 
 */
Particle& ParticleFilter::sample_motion_model(Particle& newp, auto& disp, Particle& p) {

  // add noise to the new sample, noise is Guassian with 0 mean
  std::default_random_engine generator;
  std::normal_distribution<float> distribution(0.0, 2.0);  // inputs are mean and variance

  // update location
  newp.x = p.x + disp.translation.x; // * cos(p.t * RAD_T_DEG) + distribution(generator);
  newp.y = p.y + disp.translation.y; // * sin(p.t * RAD_T_DEG) + distribution(generator);
  newp.t = p.t + disp.rotation * RAD_T_DEG; // + distribution(generator);


  // Assuming theta is between (-180, 180) degree
  newp.t -= newp.t >  180.0 ? 360.0 : 0;
  newp.t += newp.t < -180.0 ? 360.0 : 0;

  assert(newp.t >= -180.0 && newp.t <= 180.0);

  newp.w = p.w;
  return newp;
}

const Pose2D& ParticleFilter::pose() const {
  if(dirty_) {
    // Compute the mean pose estimate
    mean_ = Pose2D();
    using T = decltype(mean_.translation);
    for(const auto& p : particles()) {
      // no need to weight since they have equal weight
      // weights are only used in resample process
      mean_.translation += T(p.x,p.y);
      mean_.rotation += p.t;
    }
    if(particles().size() > 0)
      mean_ /= static_cast<float>(particles().size());
    dirty_ = false;
  }
  return mean_;
}
