#include <localization/ParticleFilter.h>
#include <memory/FrameInfoBlock.h>
#include <memory/OdometryBlock.h>
#include <common/Random.h>
#include <assert.h>
#include <math.h>


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
  std::vector<Particle> particles = particles();
  std::vector<Particle> X = particles;
  std::vector<Particle> X1 = particles;
  Particle xtm;
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
    xtm = sample_motion_model(xtm, disp, particles[i]); 
    assert(weights[i] == 0);
    totalWeight += getWeight(xtm);
    X[i] = xtm;
    w_avg = w_avg + 1/numOfParticles * xtm.w;
  }

  w_slow = w_slwo + a_slow * (w_avg - w_slow);
  w_fast = w_fast _ a_fast * (w_avg - w_fast);

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
      X1[i] = randPose();
    }
    else {
      X1[i] = resampling(particles, weights, totalWeights);
    }
  }

  // store the result back into memory
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
  float (&weights)[numOfParticles], float totalWeights) {

  Particle newP;
  float randNumber = ((float) rand()) / RAND_MAX * totalWeights;

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
  x_range = { std::max(X_MIN, particles[i-1].x-2),
              std::min(X_MAX, particles[i-1].x+2)};
  y_range = { std::max(Y_MIN, particles[i-1].y-2),
              std::min(Y_MAX, particles[i-1].y+2)};
  t_range = { std::max(-180.0, particles[i-1].t-2),
              std::min(180.0, particles[i-1].t-2)};

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
// TODO
float ParticleFilter::getWeight(Particle & x) {


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
  if (newp.theta > 180) {
    newp.theta -= 360;
  }

  if (newp.theta < -180) {
    newp.theta += 360;
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
