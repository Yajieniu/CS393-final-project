#include <localization/ParticleFilter.h>
#include <memory/FrameInfoBlock.h>
#include <memory/OdometryBlock.h>
#include <common/Random.h>


/* 
 * Create an instance of class Particle Filter. 
 * Or to say create a particle filter. 
 * Only needed to be called once at the beginning. 
 */
ParticleFilter::ParticleFilter(MemoryCache& cache, TextLogger*& tlogger) 
  : cache_(cache), tlogger_(tlogger), dirty_(true), 
    backToRandom(false), needToUpdate(false) {
}


/* 
 * Allow the particles to be udpated in this frame. 
 */
void ParticleFilter::NeedUpdate() {
  needToUpdate = true;
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

  // Retrieve odometry update - how do we integrate this into the filter?
  // This is the observation, z_t
  // contains x, and y and theta
  // But might need further translation
  const auto& disp = cache_.odometry->displacement;
  log(41, "Updating particles from odometry: %2.f,%2.f @ %2.2f", disp.translation.x, disp.translation.y, disp.rotation * RAD_T_DEG);
  
  // We have a fixed number of particles, for now.
  // might need to add random samples to recovery from kidnapping,
  // which Josiah said he would definitely do
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
  else if (needToUpdate) {  // can only keep this part
    PFAlgorithm();
  }
}


/* 
 * The Implementation of the Particle Filter algorithm. 
 */
void ParticleFilter::PFAlgorithm() {

  // Getting the last particle set
  std::vector<Particle> particles = particles();
  std::vector<Particle> X = particles;
  std::vector<Particle> X1 = particles;
  Particle xtm;
  float weights[numOfParticles] = {};
  float totalWeight = 0;

  int M = numOfParticles;

  // First part of the algorithm
  for (int i = 0; i < M; i++) {
    xtm = sampling(ut, particles[i]); 
    assert(weights[i] == 0);
    totalWeight += getWeight(xtm);
    X[i] = xtm;
  }

  // Instead of normalizing the weight, here is getting 
  // the sum of all weights
  weights[0] = X[0].w;
  for (int i = 1; i < M; i++) {
    weights[i] = weights[i-1] + X[i].w;
  }

  // Second part of the algorithm
  for (int i = 0; i < M; i++) {
    X1[i] = resampling();
  }

  // store the result back into memory
  cache_.localization_mem->particles = X1;
}


/* Algorithm line 9. */
Particle& ParticleFilter::resampling(std::vector<Particle>& particles, 
  float (&weights)[numOfParticles]) {
  Particle x = particles[0];
  return x;
}

/* Algorithm line 5. */
float ParticleFilter::getWeight(Particle & x) {
  return x.w;
}

/* Algorithm line 4. */
Particle& ParticleFilter::sampling(std::String& ut, Particle& xm) {
  Particle x = xm;
  return x;
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
