#include <localization/ParticleFilter.h>
#include <memory/FrameInfoBlock.h>
#include <memory/OdometryBlock.h>
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
  { WO_BEACON_BLUE_YELLOW,    {1500, -1000} },
  { WO_BEACON_YELLOW_BLUE,    {1500, 1000} },
  { WO_BEACON_BLUE_PINK,      {0, -1000} },
  { WO_BEACON_PINK_BLUE,      {0, 1000} },
  { WO_BEACON_PINK_YELLOW,    {-1500, -1000} },
  { WO_BEACON_YELLOW_PINK,    {-1500, 1000} }
};

/* 
 * Create an instance of class Particle Filter. 
 * Or to say create a particle filter. 
 * Only needed to be called once at the beginning. 
 */
ParticleFilter::ParticleFilter(MemoryCache& cache, TextLogger*& tlogger) 
  : cache_(cache), tlogger_(tlogger), dirty_(true),
  backToRandom(false){
    w_slow = 0;
    w_fast = 0;
    a_slow = 0.1;
    a_fast = 1000;
}

/* 
 * Initialize the particle filter. 
 * Does not make sense to me. 
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
  
  // We have a fixed number of particles.
  particles().resize(numOfParticles); 

  if (backToRandom) { // could get rid of this random part
    // Generate random particles for demonstration
    particles().resize(100);  // change the num of particles
    auto frame = cache_.frame_info->frame_id;
    for(auto& p : particles()) {
      p.x = Random::inst().sampleN() * 250 + (frame * 5); //static_cast<int>(frame * 5), 250);
      p.y = Random::inst().sampleN() * 250; // 0., 250);
      p.t = Random::inst().sampleN() * M_PI / 4;  //0., M_PI / 4);
      p.w = Random::inst().sampleU();
    }
  }
  else {  // can only keep this part
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

  int M = numOfParticles;

  // Retrieve odometry update - how do we integrate this into the filter?
  // This is the observation, z_t
  // contains x, and y and theta
  // But might need further translation
  const auto& disp = cache_.odometry->displacement;
  log(41, "Updating particles from odometry: %2.f,%2.f @ %2.2f", disp.translation.x, 
            disp.translation.y, disp.rotation * RAD_T_DEG); 

  // First part of the algorithm
  for (int i = 0; i < M; i++) {
    tempP = sample_motion_model(tempP, disp, particles[i]); 
    assert(weights[i] == 0);
    totalWeight += getWeight(tempP);
    X[i] = tempP;
    w_avg = w_avg + 1/numOfParticles * tempP.w;
  }

  w_slow = w_slow + a_slow * (w_avg - w_slow);
  w_fast = w_fast + a_fast * (w_avg - w_fast);

  // Instead of normalizing the weight, here is getting 
  // the sum of all weights
  weights[0] = X[0].w;
  for (int i = 1; i < M; i++) {
    assert(X[i].w >= 0);
    weights[i] = weights[i-1] + X[i].w;
  }

  // Second part of the algorithm
  for (int i = 0; i < M; i++) {
    randNumber = std::rand() / ((float) RAND_MAX);
    if (randNumber <= std::max(0.0, 1.0 - w_fast/w_slow)) {
      X1[i] = ParticleFilter::randPose();
    }
    else {
      X1[i] = ParticleFilter::resampling(particles, weights, totalWeight);
    }
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
Particle& ParticleFilter::randPose() {
  Particle particle;
  particle.x = ((float) std::rand()) / RAND_MAX * X_MAX * 2 - X_MAX;
  particle.y = ((float) std::rand()) / RAND_MAX * Y_MAX * 2 - Y_MAX;
  particle.t = ((float) std::rand()) / RAND_MAX * 360 - 180;
  particle.w = 0;

  return particle;
}

/*  
 * Algorithm line 9. Resample a new particle. 
 * Generate a random number to pick one existed particle p,
 * the new particle wil fall into a small range around p, 
 * and have a similar theta.
 */
Particle& ParticleFilter::resampling(std::vector<Particle>& particles, 
  float *weights, float totalWeight) {

  Particle newP;
  float randNumber = ((float) rand()) / RAND_MAX * totalWeight;

  int i = 0;
  assert(weights[0] == 0);
  assert(randNumber <= weights[numOfParticles-1]);
  
  // resampled particle should be close to particles[i-1]
  while(randNumber >= weights[i]) {
    i++;
  }
  assert(i>0);

  std::vector<float> x_range;
  std::vector<float> y_range;
  std::vector<float> t_range;

  // the location of the newly sample particle falls in this range
  x_range.push_back(std::max(X_MIN, particles[i-1].x-2));
  x_range.push_back(std::min(X_MAX, particles[i-1].x+2));
  x_range.push_back(std::max(Y_MIN, particles[i-1].y-2));
  x_range.push_back(std::min(Y_MAX, particles[i-1].y+2));
  x_range.push_back(std::max((float) -180, particles[i-1].t-2));
  x_range.push_back(std::min((float) 180, particles[i-1].t-2));


  // determine the location of new particle
  newP.x = ((float) rand()) / RAND_MAX * 
               (x_range[1] - x_range[0]) + x_range[0];

  newP.y = ((float) rand()) / RAND_MAX * 
               (y_range[1] - y_range[0]) + y_range[0];

  newP.t = ((float) rand()) / RAND_MAX * 
               (t_range[1] - t_range[0]) + t_range[0];

  newP.w = 0;  // whatever, not used in the future

  return newP;
}

/* Algorithm line 6. Get weights for each new sample. */
float ParticleFilter::getWeight(Particle & x) {
  x.w = 1;

  // TODO: set variance for gaussian, and may need to debug to adjust signs.
  // Also haven't compiled because other parts not complete. May have error.
  for (const auto& beacon : beaconLocation) {
    const auto& object =  cache_.world_object->objects_[beacon.first];
    if ( object.seen == false )
      continue;

    float dist = ( (beacon.second.x - x.x)*(beacon.second.x - x.x)
                + (beacon.second.y - x.y)*(beacon.second.y - x.y) );
    x.w *= gaussianPDF ( object.visionDistance, dist, 100 );

    /*
      We have to decide what is the best way to work with theta from
      -pi to +pi, while functions like tanh() return -pi/2 to pi/2.
      Currently setting everything to (-pi/2, pi/2], which is incorrect
      if we have error of exactly pi!!!
    */

    float world_theta = tanh((beacon.second.y - x.y) / (beacon.second.x - x.x));
    while (world_theta <= -M_PI/2) world_theta += M_PI;
    while (world_theta > M_PI/2) world_theta -= M_PI;

    // TODO: May have to debug to use correct sign.
    float relative_theta = x.t + object.visionBearing;
    while (relative_theta <= -M_PI/2) relative_theta += M_PI;
    while (relative_theta > M_PI/2) relative_theta -= M_PI;

    x.w *= gaussianPDF ( relative_theta, world_theta, 1 );
  }

  // If no beacon seen, then everyone gets weight 1
  return x.w;
}

/* 
 * Algorithm line 5. 
 * Get new estimated samples from the motion. 
 */
Particle& ParticleFilter::sample_motion_model(Particle& newp, auto& disp, Particle& p) {
  newp.x = p.x + disp.translation.x * cos(p.t * RAD_T_DEG);
  newp.y = p.y + disp.translation.y * sin(p.t * RAD_T_DEG);
  newp.t = p.t + disp.rotation * RAD_T_DEG;


  // Assuming theta is between (-180, 180) degree
  if (newp.t > 180) {
    newp.t -= 360;
  }

  if (newp.t < -180) {
    newp.t += 360;
  }

  assert(newp.t >= -180 && newp.t <= 180);

  newp.w = p.w;
  return newp;
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

inline float ParticleFilter::gaussianPDF( float x, float mu, float sigma = 100) {
  return (1 / sqrt(2*M_PI*sig*sig)) * exp(- ((x-mu)*(x-mu)) / (2*sig*sig) ) ;
}