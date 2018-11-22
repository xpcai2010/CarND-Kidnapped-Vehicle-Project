/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;
#define eps 0.00001

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	if (is_initialized){
		return;
	}

	num_particles = 100;
	default_random_engine gen;

	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	for (int i=0; i<num_particles; ++i){
		Particle particle;
		particle.id = i;
		particle.x = dist_x(gen);
		particle.y = dist_y(gen);
		particle.theta = dist_theta(gen);
		particle.weight = 1;
		particles.push_back(particle);
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	
	default_random_engine gen;
	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);

	for (int i = 0; i < num_particles; ++i){
		if (fabs(yaw_rate) < eps){
			particles[i].x += velocity * delta_t * cos(particles[i].theta);
			particles[i].y += velocity * delta_t * sin(particles[i].theta);
		}
		else{
          particles[i].x += velocity / yaw_rate * (sin (particles[i].theta + yaw_rate * delta_t) - sin (particles[i].theta));
          particles[i].y += velocity / yaw_rate * (cos (particles[i].theta) - cos (particles[i].theta + yaw_rate * delta_t));
          particles[i].theta += yaw_rate * delta_t;			
		}
		particles[i].x += dist_x(gen);
		particles[i].y += dist_y(gen);
		particles[i].theta += dist_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	for (unsigned int i = 0; i < observations.size(); ++i){
		double distance_min = dist(observations[i].x, observations[i].y, predicted[0].x, predicted[0].y);
		int id_obs = predicted[0].id;
		if (predicted.size() > 1){
			for (unsigned int j = 1; j < predicted.size(); ++j){
				double distance_cur = dist(predicted[j].x, predicted[j].y, observations[i].x, observations[i].y);
				if (distance_cur < distance_min){
					distance_min = distance_cur;
					id_obs = predicted[j].id;
				}
		}
		observations[i].id = id_obs;	
		}
	}

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	for (int i = 0; i < num_particles; ++i){

		std::vector<LandmarkObs> observations_mapCoordinate;
		for (unsigned int j = 0; j < observations.size(); ++j){
			LandmarkObs obs_mapCoordinate;
			obs_mapCoordinate.x = cos(particles[i].theta) * observations[j].x - sin(particles[i].theta) * observations[j].y + particles[i].x;
			obs_mapCoordinate.y = sin(particles[i].theta) * observations[j].x + cos(particles[i].theta) * observations[j].y + particles[i].y;
			obs_mapCoordinate.id = observations[j].id;
			observations_mapCoordinate.push_back(obs_mapCoordinate);
		}

	// create a vector to keep the landmark map locations within sensor range of the particle
		std::vector<LandmarkObs> predicted;
		
		for (unsigned int k = 0; k < map_landmarks.landmark_list.size(); ++k){
			double distance_temp = dist( particles[i].x, particles[i].y, map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f);
			LandmarkObs pred;
			if (distance_temp <= sensor_range){
				pred.id = map_landmarks.landmark_list[k].id_i;
				pred.x = map_landmarks.landmark_list[k].x_f;
				pred.y = map_landmarks.landmark_list[k].y_f;
				predicted.push_back(pred);
			}
		}

		dataAssociation(predicted, observations_mapCoordinate);
		double exp_item;
		particles[i].weight = 1.0;

	
		for (unsigned int l = 0; l < observations_mapCoordinate.size(); ++l){
			double pred_x, pred_y, obs_x, obs_y;
			obs_x = observations_mapCoordinate[l].x;
			obs_y = observations_mapCoordinate[l].y;

			for (unsigned int m = 0; m < predicted.size(); ++m){
				if (observations_mapCoordinate[l].id == predicted[m].id){
					pred_x = predicted[m].x;
					pred_y = predicted[m].y;
				}
			}

			exp_item = exp(-1.0 * (pow(obs_x - pred_x, 2.0)/(2.0*pow(std_landmark[0],2.0))+ pow(obs_y - pred_y, 2.0)/(2.0*pow(std_landmark[1],2.0)))); 
			double weight = 1.0 /(2 * M_PI * std_landmark[0] * std_landmark[1]) *exp_item;

			particles[i].weight *= weight;
			
		}
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	default_random_engine gen;
	std::uniform_int_distribution<int> dist_int(0, num_particles-1);
	int index = dist_int(gen);

	vector<double> weights;

	double beta = 0.0;
	vector <Particle> particles_new;
	for (int i = 0; i < num_particles; ++i){
		weights.push_back(particles[i].weight);
	}
	
	double mw = *max_element(weights.begin(), weights.end());
	std::uniform_real_distribution<double> dist_double(0, 1);

	for (int i = 0; i < num_particles; ++i){
		
		beta += dist_double(gen) * 2.0 * mw;
		while (beta > weights[index]){
			beta -= weights[index];
			index = (index + 1) % num_particles;
		}
		particles_new.push_back(particles[index]);
	}

	particles = particles_new;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;

	return particle;
}


string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
