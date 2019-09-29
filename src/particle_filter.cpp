/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

#define EPS 0.00001

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 100;  // TODO: Set the number of particles

  std::default_random_engine gen;

  std::normal_distribution<double> N_x(x, std[0]);
  std::normal_distribution<double> N_y(y, std[1]);
  std::normal_distribution<double> N_theta(theta, std[2]);

  for (int i = 0; i < num_particles; i++)
  {
    Particle particle;
    particle.id = i;
    particle.x = N_x(gen);
    particle.y = N_y(gen);
    particle.theta = N_theta(gen);
    particle.weight = 1;

    particles.push_back(particle);
    weights.push_back(1);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

  std::default_random_engine gen;

  for (int i = 0; i < num_particles; i++)
  {
    double new_x;
    double new_y;
    double new_theta;

    // if (yaw_rate == 0)
    if (fabs(yaw_rate) < EPS)
    {
      new_x = particles[i].x + velocity * delta_t * cos(particles[i].theta);
      new_y = particles[i].y + velocity * delta_t * sin(particles[i].theta);
      new_theta = particles[i].theta;
    }
    else
    {
      new_x = particles[i].x + (velocity / yaw_rate) * (sin(particles[i].theta + (yaw_rate * delta_t)) - sin(particles[i].theta));
      new_y = particles[i].y + (velocity / yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + (yaw_rate * delta_t)));
      new_theta = particles[i].theta + (yaw_rate * delta_t);
    }

    std::normal_distribution<double> N_x(new_x, std_pos[0]);
    std::normal_distribution<double> N_y(new_y, std_pos[1]);
    std::normal_distribution<double> N_theta(new_theta, std_pos[2]);

    particles[i].x = N_x(gen);
    particles[i].y = N_y(gen);
    particles[i].theta = N_theta(gen);        
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
            
  // loop over number of landmarks and estimate pseudo ranges
  for (unsigned int i = 0; i < observations.size(); i++) {

    // current observation
    LandmarkObs observation = observations[i];

    // initialize minimum distance to maximum possible value
    double min_dist = std::numeric_limits<double>::max();

    // initialize id of landmark from map to be associated with the observation
    int map_id = -1;

    for (unsigned int j = 0; j < predicted.size(); j++) {

      // current prediction
      LandmarkObs prediction = predicted[j];

      // get distance between current observation & prediction
      double current_dist = dist(observation.x, observation.y, prediction.x, prediction.y);

      // find the predicted landmark nearest the current observed landmark
      if (current_dist < min_dist) {
        min_dist = current_dist;
        map_id = prediction.id;
      }
    }

    // set the observation's id to the nearest predicted landmark's id
    observations[i].id = map_id;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

  // Iterate through each particle
  for (int i = 0; i < num_particles; i++) {

    // get the particle x, y coordinates
    double particle_x = particles[i].x;
    double particle_y = particles[i].y;
    double particle_theta = particles[i].theta;

    // placeholder to keep map landmark locations those are predicted to be within sensor range of the particle
    vector<LandmarkObs> predictions;

    // For each landmark
    for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {
    
      // get id and x, y coordinates
      float landmark_x = map_landmarks.landmark_list[j].x_f;
      float landmark_y = map_landmarks.landmark_list[j].y_f;
      int landmark_id = map_landmarks.landmark_list[j].id_i;

      // only consider landmarks within sensor range of the particle
      if (fabs(landmark_x - particle_x) <= sensor_range && fabs(landmark_y - particle_y) <= sensor_range) {

        // add chosen landmarks to predictions
        predictions.push_back(LandmarkObs{ landmark_id, landmark_x, landmark_y });
      }
    }

    // transform observations from vehicle coordinates to map coordinates
    vector<LandmarkObs> transformed_observations;
    LandmarkObs observation;
    for (unsigned int j = 0; j < observations.size(); j++) {
      LandmarkObs transformed_observation;
      observation = observations[j];

      // perform the space transformation from vehicle to map
      transformed_observation.x = particle_x + (observation.x * cos(particle_theta) - observation.y * sin(particle_theta));
      transformed_observation.y = particle_y + (observation.x * sin(particle_theta) + observation.y * cos(particle_theta));
      transformed_observation.id = observation.id;
      transformed_observations.push_back(transformed_observation);
    }

    // perform dataAssociation for the predictions and transformed observations on current particle
    dataAssociation(predictions, transformed_observations);

    // reinitialize weight
    particles[i].weight = 1.0;

    vector<int> associations;
    vector<double> sense_x;
    vector<double> sense_y;

    for (unsigned int j = 0; j < transformed_observations.size(); j++) {
      
      // placeholders for observation (measurement) and associated prediction (mean) coordinates
      double meas_x = transformed_observations[j].x;
      double meas_y = transformed_observations[j].y;

      int association = transformed_observations[j].id;

      // get the x,y coordinates of the prediction (mean) associated with the current observation
      double mu_x, mu_y;
      for (unsigned int k = 0; k < predictions.size(); k++) {
        if (predictions[k].id == association) {
          mu_x = predictions[k].x;
          mu_y = predictions[k].y;
        }
      }

      // Saving values for debugging (optional)
      associations.push_back(association);
      sense_x.push_back(mu_x);      
      sense_y.push_back(mu_y);

      // calculate weight for this observation with multivariate Gaussian
      double sigma_x = std_landmark[0];
      double sigma_y = std_landmark[1];
      long double multiplier = ( 1/(2*M_PI*sigma_x*sigma_y)) * exp( -( pow(meas_x - mu_x,2)/(2*pow(sigma_x, 2)) + (pow(meas_y - mu_y,2)/(2*pow(sigma_y, 2))) ) );

      // combine this obersvation weight with particle's total observations weight
      if (multiplier > 0) {
        particles[i].weight *= multiplier;
      } else if (multiplier == 0) {
        particles[i].weight *= EPS;
      }

      weights[i] = particles[i].weight;
    }
    SetAssociations(particles[i], associations, sense_x, sense_y);
  }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  std::default_random_engine gen;

  // Vector for new particles
  vector<Particle> resample_particles;

  /*
  // Begins Method - 1
  // Use discrete distribution to return particles by weight
  std::discrete_distribution<int> distribution(weights.begin(), weights.end());

  for (int i = 0; i < num_particles; i++)
  {
    resample_particles.push_back(particles[distribution(gen)]);
  }
  // End of Method - 1
  */

  // Begins Method - 2
  // Use resampling wheel to return particles by weight
  // Get max weight
  double max_weight = std::numeric_limits<double>::min();
  for(int i = 0; i < num_particles; i++) {
    if ( particles[i].weight > max_weight ) {
      max_weight = particles[i].weight;
    }
  } 
  
  // Creating distribution
  std::uniform_real_distribution<double> dist_double(0.0, max_weight);
  std::uniform_int_distribution<int> dist_int(0, num_particles - 1);

  // Generating index
  int index = dist_int(gen);

  double beta = 0.0;

  // the resampling wheel
  for(int i = 0; i < num_particles; i++) {
    beta += dist_double(gen) * 2.0;
    while( beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    resample_particles.push_back(particles[index]);
  }
  // End of Method - 2

  // Replace old particles with the resampled particles
  particles = resample_particles;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates

  // Clear the previous associations
  particle.associations.clear();
  particle.sense_x.clear();
  particle.sense_y.clear();

  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
