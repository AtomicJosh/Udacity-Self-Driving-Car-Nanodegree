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

using std::string;
using std::vector;

static std::default_random_engine gen; // Declare the random engine

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */

  num_particles = 100;

  // define normal distributions for sensor noise
  std::normal_distribution<double> dist_x(0, std[0]);
  std::normal_distribution<double> dist_y(0, std[1]);
  std::normal_distribution<double> dist_theta(0, std[2]);

  // init particles
  for (int i = 0; i < num_particles; i++) {
    Particle p;
    p.id = i;
    p.x = x + dist_x(gen);
    p.y = y + dist_y(gen);
    p.theta = theta + dist_theta(gen);
    p.weight = 1.0;

    particles.push_back(p);
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

  // Define normal distributions for sensor noise
  std::normal_distribution<double> dist_x_2(0, std_pos[0]);
  std::normal_distribution<double> dist_y_2(0, std_pos[1]);
  std::normal_distribution<double> dist_theta_2(0, std_pos[2]);

  for (int i = 0; i < num_particles; i++) {

    // Calculate the new state
    if (fabs(yaw_rate) < 0.00001) {
      particles[i].x += velocity * delta_t * cos(particles[i].theta) + dist_x_2(gen);
      particles[i].y += velocity * delta_t * sin(particles[i].theta) + dist_y_2(gen);
      particles[i].theta += dist_theta_2(gen);
    } else {
      particles[i].x += velocity / yaw_rate *
                        (sin(particles[i].theta + yaw_rate*delta_t) -
                         sin(particles[i].theta)) + dist_x_2(gen);
      particles[i].y += velocity / yaw_rate *
                        (cos(particles[i].theta) - cos(particles[i].theta +
                         yaw_rate*delta_t)) + dist_y_2(gen);
      particles[i].theta += yaw_rate * delta_t;
    }
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

  for (uint i = 0; i < observations.size(); i++) {
    // Set the minimum distance to the maximum possible
    double min_dist = std::numeric_limits<double>::max();

    // Set the initial ID of the landmark from map placeholder to be associated with the observation
    int map_id = -1;

    for (unsigned int j = 0; j < predicted.size(); j++) {
      // Calculate the distance between the current and predicted landmarks
      double curr_dist = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);

      // Determine the predicted landmark closest to the current observed landmark
      if (curr_dist < min_dist) {
        min_dist = curr_dist;
        map_id = predicted[j].id; // Set the observation's ID to the closest predicted landmark's ID
      }
    }
    observations[i].id = map_id; // Set the observation's ID to the closest predicted landmark's ID
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * Update the weights of each particle using a mult-variate Gaussian 
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

  for (int i = 0; i < num_particles; i++) {
    // Vector of map landmark locations predicted to be within sensor range of the particle
    vector<LandmarkObs> predictions;

    for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {

      // Get landmark ID, x, and y coordinates
      float landmark_x = map_landmarks.landmark_list[j].x_f;
      float landmark_y = map_landmarks.landmark_list[j].y_f;
      int landmark_id = map_landmarks.landmark_list[j].id_i;

      // Verify landmarks are within sensor range
      double landmark_part_dist = sqrt(pow(particles[i].x - landmark_x, 2) +
                                       pow(particles[i].y - landmark_y, 2));
      // If so, add them to the predictions
      if (landmark_part_dist <= sensor_range) {
        predictions.push_back(LandmarkObs{landmark_id, landmark_x, landmark_y});
      }
    }

    // Calculate the observations transformed from vehicle coordinates to map coordinates
    vector<LandmarkObs> transformed_obs;
    for (unsigned int j = 0; j < observations.size(); j++) {
      double transformed_x = particles[i].x + cos(particles[i].theta) * observations[j].x -
                                              sin(particles[i].theta) * observations[j].y;
      double transformed_y = particles[i].y + sin(particles[i].theta) * observations[j].x +
                                              cos(particles[i].theta) * observations[j].y;
      transformed_obs.push_back(LandmarkObs{observations[j].id, transformed_x, transformed_y});
    }

    // Associate predictions and transformed observations on current particle
    dataAssociation(predictions, transformed_obs);

    // Reset the weight
    particles[i].weight = 1.0;

    double obs_x, obs_y, pred_x, pred_y;

    for (unsigned int j = 0; j < transformed_obs.size(); j++) {

      // Get observation coordinates
      obs_x = transformed_obs[j].x;
      obs_y = transformed_obs[j].y;

      // Get the x,y coordinates of the prediction associated with the current observation
      for (unsigned int k = 0; k < predictions.size(); k++) {
        if (predictions[k].id == transformed_obs[j].id) {
          pred_x = predictions[k].x;
          pred_y = predictions[k].y;
        }
      }

      // Calculate weight for this observation
      double sig_x = std_landmark[0];
      double sig_y = std_landmark[1];
      double weight = (1 / (2 * M_PI * sig_x * sig_y)) *
                      exp(-(pow(pred_x - obs_x, 2) / (2 * pow(sig_x, 2)) +
                           (pow(pred_y - obs_y, 2) / (2 * pow(sig_y, 2)))));

      // Multiply this observation weight with total observations weight
      particles[i].weight *= weight;
    }
  }
}

void ParticleFilter::resample() {
  /**
   * Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  vector<Particle> new_particles;

  // Get current weights
  vector<double> weights;
  for (int i = 0; i < num_particles; i++) {
    weights.push_back(particles[i].weight);
  }

  // Create random starting index for resampling wheel
  std::uniform_int_distribution<int> int_dist(0, num_particles-1);
  auto index = int_dist(gen);

  // Determine max weight
  double max_weight = *max_element(weights.begin(), weights.end());

  // Determine the uniform random distribution from 0 to max_weight
  std::uniform_real_distribution<double> real_dist(0.0, max_weight);

  double beta = 0.0;

  for (int i = 0; i < num_particles; i++) {
    beta += real_dist(gen) * 2.0;
    while (beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    new_particles.push_back(particles[index]);
  }
  particles = new_particles;
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
  particle.associations = associations;
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
