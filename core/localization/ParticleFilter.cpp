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

ParticleFilter::ParticleFilter(MemoryCache& cache, TextLogger*& tlogger) 
  : cache_(cache), tlogger_(tlogger), dirty_(true),
  backToRandom(true){
    w_slow = 0;
    w_fast = 0;
    a_slow = 0.05;
    a_fast = 0.5;

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
  
  if (backToRandom) {
    particles().resize(numOfParticles); 
    auto frame = cache_.frame_info->frame_id;
    for(auto& p : particles()) {
      p.x = Random::inst().sampleU() * X_MAX * 2 - X_MAX ;
      p.y = Random::inst().sampleU() * Y_MAX * 2 - Y_MAX;
      p.t = Random::inst().sampleU() * M_2PI - M_PI;
      p.w = 1e-8;
    }
    backToRandom = false;
  }
  else {
    RandomParticleMCL();
  }
}

/* 
 * The Implementation of the random particle MCL algorithm. 
 */
void ParticleFilter::RandomParticleMCL() {

  // Getting the last particle set
  std::vector<Particle> particles = ParticleFilter::particles();
  std::vector<Particle> X0;
  std::vector<Particle> X1;
  Particle tempP;
  float weights[numOfParticles] = {};
  float totalWeight = 0.0;
  float w_old = 0.0;
  float w_avg = 0.0;
  float randNumber;

  const auto& disp = cache_.odometry->displacement;

  log(41, "Updating particles from odometry: %2.f,%2.f @ %2.2f", disp.translation.x, 
            disp.translation.y, disp.rotation * RAD_T_DEG); 

  for (auto p: particles) w_old += 1./numOfParticles * p.w;

  for (int i = 0; i < numOfParticles; i++) {
    tempP = sample_motion_model(tempP, disp, particles[i]); 
    assert(weights[i] == 0);
    totalWeight += getWeight(tempP, w_old);
    X0.push_back(tempP);
    w_avg = w_avg + 1./numOfParticles * tempP.w;
  }


  w_slow = w_slow + a_slow * (w_avg - w_slow);
  w_fast = w_fast + a_fast * (w_avg - w_fast);

  // Instead of normalizing the weight, here is getting 
  // the sum of all weights
  for (int i = 0; i < numOfParticles; i++) {
    if (i > 0) {
      weights[i] = weights[i-1] + X0[i].w;
    } else {
      weights[i] = X0[i].w;
    }
  }
  
  // Second part of the algorithm
  int counter = 0;  // count how many particles we should resample
  for (int i = 0; i < numOfParticles; i++) {
    randNumber = Random::inst().sampleU();
    if (randNumber <= std::max(0.0, std::min(1.0 - w_fast/w_slow, 0.1))) {
    // if (randNumber < 0) {
      X1.push_back(ParticleFilter::randPose(tempP, w_avg));
    }
    else {
      counter += 1;
    }
  }

  // Roulette wheel, resample according to random number
  int resample_version = 1;
  int newParticleIndex = 0;
  assert(resample_version == 1 || resample_version == 2);  // must be 1 or 2

  for (int i = 0; i < counter; i++) {
    newParticleIndex = floor(i * counter / numOfParticles);  // not used if resample_version is 1
    X1.push_back(resampling(tempP, X0, weights, weights[numOfParticles-1], resample_version, newParticleIndex));
  }
  
  // store the result back into memory
  cache_.localization_mem->particles = X1;
}

/* 
 * Generate a random particle. 
 * random x, random y, and random theta.
 */
Particle& ParticleFilter::randPose(Particle& p, float w_avg) {
  // p.x = Random::inst().sampleU() * X_MAX * 2 - X_MAX;
  // p.y = Random::inst().sampleU() * Y_MAX * 2 - Y_MAX;
  // p.t = Random::inst().sampleU() * M_2PI - M_PI;
  p.x = mean_.translation.x + Random::inst().sampleN() * X_MAX/4;
  p.y = mean_.translation.y + Random::inst().sampleN() * Y_MAX/4;
  p.t = mean_.rotation + Random::inst().sampleN() * M_PI/4;
  p.w = w_avg;

  return p;
}

/*  
 * Resample a new particle. 
 */
Particle& ParticleFilter::resampling(Particle& newP, std::vector<Particle>& particles, 
  float weights[], float totalWeight, int version, int newParticleIndex) {

  int i = 0;
  assert(version == 1 || version == 2);

  if (version == 1) {
    float randNumber = Random::inst().sampleU() * totalWeight;
    assert(randNumber <= weights[numOfParticles-1]);
    
    // resampled particle should be close to particles[i-1]
    while(randNumber > weights[i]) {
      i++;
    }
  }
  else {  
    // version 2, Systematic resampling
    // Not implemented
  }

  newP.x = particles[i].x;
  newP.y = particles[i].y;
  newP.t = particles[i].t;

  return newP;
}

/* Algorithm line 6. Get weights for each new sample. */
float ParticleFilter::getWeight(Particle & p, float w_old) {
  auto w = 1.0;

  int count = 0;

  for (const auto& beacon : beaconLocation) {
    const auto& object = cache_.world_object->objects_[beacon.first];
    if ( object.seen == false )
      continue;

    count++;
    float dist = sqrt( (beacon.second.x - p.x)*(beacon.second.x - p.x)
                + (beacon.second.y - p.y)*(beacon.second.y - p.y) );
    w *= gaussianPDF ( object.visionDistance, dist, 500);

    /*
      Setting everything to (-pi, pi].
    */

    float beacon_theta = atan((beacon.second.y - p.y) / (beacon.second.x - p.x));

    assert(beacon_theta >= -M_PI && beacon_theta <= M_PI);    

    float beacon_bearing = object.visionBearing;
    assert(beacon_bearing >= -M_PI && beacon_bearing <= M_PI);

    float theta = beacon_theta - beacon_bearing;

    if (beacon.second.x - p.x < 0 && beacon.second.y - p.y < 0) {
      theta -= M_PI;
    }

    if (beacon.second.x - p.x < 0 && beacon.second.y - p.y > 0) {
      theta += M_PI;
    }

    if (theta >= M_PI) theta -= M_2PI;
    if (theta <= -M_PI) theta += M_2PI;

    w *= gaussianPDF (p.t, theta, 180/RAD_T_DEG );

  }


  if (count > 0) p.w = w + 1e-8;
  else p.w = w_old;            

  return p.w;
}

inline float ParticleFilter::gaussianPDF( float x, float mu, float sigma = 10) {
  return (1. / sqrt(2*M_PI*sigma*sigma)) * exp(- ((x-mu)*(x-mu)) / (2*sigma*sigma));
}


/* 
 * Get new estimated samples from the motion. 
 */
Particle& ParticleFilter::sample_motion_model(Particle& newp, auto& disp, Particle& p) {

  // add noise to the new sample, noise is Guassian with 0 mean
  std::default_random_engine generator;
  std::normal_distribution<float> distribution(0.0, 2.0);  // inputs are mean and variance

  // update location
  auto dx = disp.translation.x;
  auto dy = disp.translation.y;

  newp.x = p.x + dx*cos(p.t) - dy*sin(p.t) + Random::inst().sampleN() * 15; 
  newp.y = p.y + dx*sin(p.t) + dy*cos(p.t) + Random::inst().sampleN() * 15;
  newp.t = p.t + disp.rotation + Random::inst().sampleN() * 5/RAD_T_DEG; 

  // Assuming theta is between -pi, pi degree
  while (newp.t <= -M_PI) newp.t += M_2PI;
  while (newp.t >= M_PI) newp.t -= M_2PI;
  assert(newp.t >= -M_PI && newp.t <= M_PI);

  newp.w = p.w;
  return newp;
}

const Pose2D& ParticleFilter::pose() { // Our Pose
  if(dirty_) {
    // Compute the mean pose estimate
    mean_ = kMeans();
    dirty_ = false;
  }
  return mean_;
}

// const Pose2D& ParticleFilter::pose() { // Josiah's code
//   if(dirty_) {
//     // Compute the mean pose estimate
//     mean_ = Pose2D();
//     using T = decltype(mean_.translation);
//     for(const auto& p : particles()) {
//       mean_.translation += T(p.x,p.y);
//       mean_.rotation += p.t;
//     }
//     if(particles().size() > 0)
//       mean_ /= static_cast<float>(particles().size());
//     dirty_ = false;
//   }
//   return mean_;
// }

const Pose2D& ParticleFilter::kMeans() {
  auto &P = particles(); // for easier implementation
  static vector<Particle> means(K), old_means(K);
  static vector<int> Near(numOfParticles);
  const float threshold = 100; // to stop k-means early

  // init: randomly select some means;
  for (auto &mean : means) {
    mean = P[int(Random::inst().sampleU() * P.size())];
  }

  // run k-means for atmost 10 cycles
  for (int run = 0; run < 10; ++run) {
    old_means = means;

    // setting count for each mean to 0
    for (auto &mean : means) mean.w = 0;

    // finding nearest mean / assignment
    for (int i = 0; i < numOfParticles; ++i) {
      int n = -1; float m = INT_MAX; // temp variables to find minimum.
      for (int j = 0; j < K; j++) {
        if (distance (means[j], P[i]) < m) {
          m = distance (means[j], P[i]);
          n = j;
        }
      }
      Near[i] = n;
      means[n].w += 1;
    }

    // finding new means
    for (auto &mean : means) { mean.x = mean.y = mean.t = 0; }
    for (int i = 0; i < numOfParticles; ++i) {
      means[Near[i]].x += P[i].x;
      means[Near[i]].y += P[i].y;
      means[Near[i]].t += P[i].t;
    }

    for (auto &mean : means) {
      if (mean.w < 1) continue;
      mean.x /= mean.w;
      mean.y /= mean.w;
      mean.t /= mean.w;
    }

    float error = 0;
    for (int j = 0; j < K; j++) {
      error += distance(old_means[j], means[j]);
    }

    if (error < threshold) break;
  }

  // Find the best mean
  // Compute the mean pose estimate
  mean_ = Pose2D();
  using T = decltype(mean_.translation);
  float m = -1;
  for(const auto& mean : means) {
    if (m < mean.w) {
      mean_.translation = T(mean.x,mean.y);
      mean_.rotation = mean.t;
      m = mean.w;
    }
  }

  return mean_;

}

// A normalized distance function
float ParticleFilter::distance (Particle a, Particle b) {
  float ret = 0;
  ret += ((a.x - b.x)*(a.x - b.x)) / (X_MAX*X_MAX);
  ret += ((a.y - b.y)*(a.y - b.y)) / (Y_MAX*Y_MAX);
  ret += ((a.t - b.t)*(a.t - b.t)) / (M_PI*M_PI);
  return ret;
}