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
    a_slow = 0.01;
    a_fast = 0.1;

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
    if ( object.seen == false ) {
      // cout << getName(beacon.first) << " not seen.\n\n";
    }
    else {
      // cout << getName(beacon.first) << " seen.\n";
      // cout << "At distance : " << object.visionDistance << endl;
      // cout << "At bearing : " << object.visionBearing << endl;
    }
  }
  // We have a fixed number of particles.
  // cout << "Current size of particles() = " << particles().size() << endl;

  if (backToRandom) {
    particles().resize(numOfParticles); 
    auto frame = cache_.frame_info->frame_id;
    for(auto& p : particles()) {
      p.x = Random::inst().sampleU() * X_MAX * 2 - X_MAX ;
      p.y = Random::inst().sampleU() * Y_MAX * 2 - Y_MAX;
      p.t = Random::inst().sampleU() * M_2PI - M_PI;
      // p.x = Random::inst().sampleU() * 500-250; // For debugging. Use above 
      // p.y = Random::inst().sampleU() * 500-250; // For debugging. Use above 
      // p.t = 0;                                  // For debugging. Use above 
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
  std::vector<Particle> X0;
  std::vector<Particle> X1;
  Particle tempP;
  float weights[numOfParticles] = {};
  float totalWeight = 0.0;
  float w_old = 0.0;
  float w_avg = 0.0;
  float randNumber;

  // Retrieve odometry update - how do we integrate this into the filter?
  // This is the observation, z_t
  // contains x, and y and theta
  // But might need further translation
  const auto& disp = cache_.odometry->displacement;

  // cout << "Updating particles from odometry: " << disp.translation.x ", " << disp.translation.y << " @" << disp.rotation %2.f,%2.f @ %2.2f", disp.translation.x, 
  //           disp.translation.y, disp.rotation * RAD_T_DEG); 

  log(41, "Updating particles from odometry: %2.f,%2.f @ %2.2f", disp.translation.x, 
            disp.translation.y, disp.rotation * RAD_T_DEG); 

  // First part of the algorithm
  for (auto p : particles) w_old += p.w / numOfParticles;

  for (int i = 0; i < numOfParticles; i++) {
    tempP = sample_motion_model(tempP, disp, particles[i]); 
    assert(weights[i] == 0);
    totalWeight += getWeight(tempP, w_old);
    X0.push_back(tempP);
    w_avg = w_avg + 1./numOfParticles * tempP.w;
  }

  // Debug:
  // cache_.localization_mem->particles = X; // Uncomment this two line to debug motion model
  // return;                                 // Uncomment this two line to debug motion model



  w_slow = w_slow + a_slow * (w_avg - w_slow);
  w_fast = w_fast + a_fast * (w_avg - w_fast);

  // Instead of normalizing the weight, here is getting 
  // the sum of all weights
  weights[0] = X0[0].w;
  for (int i = 1; i < numOfParticles; i++) {
    assert(X0[i].w >= 0);
    weights[i] = weights[i-1] + X0[i].w;
  }

  assert (weights[numOfParticles-1] == totalWeight);
  // Second part of the algorithm

  
  int counter = 0;  // count how many particles we should resample
  for (int i = 0; i < numOfParticles; i++) {
    randNumber = Random::inst().sampleU();
    if (randNumber <= std::max(0.0, std::min(1.0 - w_fast/w_slow, 0.02))) {
      X1.push_back(ParticleFilter::randPose(tempP, w_avg));
    }
    else {
      counter += 1;
    }
  }

  // cout << "resample: " << counter << endl;
  // cout << "Wavg: " << w_avg << " , Wfast: " << w_fast << " , Wslow: " << w_slow <<  endl;

  // 1: Roulette wheel, resample according to random number
  // 2: Systematic resampling, low variance
  int resample_version = 1;  // 1: more random, 2: morn even
  int newParticleIndex = 0;
  assert(resample_version == 1 || resample_version == 2);  // must be 1 or 2

  for (int i = 0; i < counter; i++) {
    newParticleIndex = floor(i * counter / numOfParticles);  // not used if resample_version is 1
    X1.push_back(resampling(tempP, X0, weights, totalWeight, resample_version, newParticleIndex));
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
Particle& ParticleFilter::randPose(Particle& p, float w_avg) {
  p.x = Random::inst().sampleU() * X_MAX * 2 - X_MAX;
  p.y = Random::inst().sampleU() * Y_MAX * 2 - Y_MAX;
  p.t = Random::inst().sampleU() * M_2PI - M_PI;
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
    while(randNumber > weights[i]) {
      i++;
    }
  }
  else {  // version 2, Systematic resampling
    i = newParticleIndex + 1;
  }

  newP.x = particles[i].x;
  newP.y = particles[i].y;
  newP.t = particles[i].t;

  return newP;
  // std::vector<float> x_range;
  // std::vector<float> y_range;
  // std::vector<float> t_range;



  // the location of the newly sample particle falls in this range
  // float range = 2.2;
  // float tRange = 1.8;
  // x_range.push_back(std::max(X_MIN, particles[i-1].x-range));
  // x_range.push_back(std::min(X_MAX, particles[i-1].x+range)); 
  // y_range.push_back(std::max(Y_MIN, particles[i-1].y-range));
  // y_range.push_back(std::min(Y_MAX, particles[i-1].y+range));
  // t_range.push_back(std::max((float) -180, particles[i-1].t-tRange));
  // t_range.push_back(std::min((float) 180, particles[i-1].t-tRange));


  // // determine the location of new particle
  // newP.x = Random::inst().sampleU() * 
  //              (x_range[1] - x_range[0]) + x_range[0];

  // newP.y = Random::inst().sampleU() * 
  //              (y_range[1] - y_range[0]) + y_range[0];

  // newP.t = Random::inst().sampleU() * 
  //              (t_range[1] - t_range[0]) + t_range[0];

  // newP.w = 0;  // whatever, not used in the future

  // return newP;
}

/* Algorithm line 6. Get weights for each new sample. */
float ParticleFilter::getWeight(Particle & p, float w_old) {
  auto w = 1.0;

  int count = 0;

  // TODO: set variance for gaussian, and may need to debug to adjust signs.
  // Also haven't compiled because other parts not complete. May have error.
  for (const auto& beacon : beaconLocation) {
    const auto& object = cache_.world_object->objects_[beacon.first];
    if ( object.seen == false )
      continue;

    count++;
    float dist = sqrt( (beacon.second.x - p.x)*(beacon.second.x - p.x)
                + (beacon.second.y - p.y)*(beacon.second.y - p.y) );
    w *= gaussianPDF ( object.visionDistance, dist, 500); // TODO: may need to change sigma for real robot

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

    if (theta >= M_PI) theta -= M_2PI;
    if (theta <= -M_PI) theta += M_2PI;

    w *= gaussianPDF (p.t, theta, 180/RAD_T_DEG );

  }


  if (count > 0) p.w = w;
  else p.w = 1;             //w_old;
  // If no beacon seen, then everyone gets weight 1
  return p.w;
}

inline float ParticleFilter::gaussianPDF( float x, float mu, float sigma = 10) {
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
  auto dx = disp.translation.x;
  auto dy = disp.translation.y;


  newp.x = p.x + dx*cos(p.t) - dy*sin(p.t) + Random::inst().sampleN() * 15; // + distribution(generator);
  newp.y = p.y + dx*sin(p.t) + dy*cos(p.t) + Random::inst().sampleN() * 15; // + distribution(generator); 
  newp.t = p.t + disp.rotation + Random::inst().sampleN() * 5/RAD_T_DEG; // + distribution(generator); 
  // Assuming theta is between (-180, 180) degree
  while (newp.t <= -M_PI) newp.t += M_2PI;
  while (newp.t >= M_PI) newp.t -= M_2PI;
  assert(newp.t >= -M_PI && newp.t <= M_PI);

  newp.w = p.w;
  return newp;
}

const Pose2D& ParticleFilter::pose() { // TODO: Implement K-Means here
  if(dirty_) {
    // Compute the mean pose estimate
    mean_ = kMeans();
    dirty_ = false;
  }
  return mean_;
}

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