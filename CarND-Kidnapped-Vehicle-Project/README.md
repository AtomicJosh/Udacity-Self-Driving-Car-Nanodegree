# Kidnapped Vehicle Project
Self-Driving Car Engineer Nanodegree Program
  
# Overview
This project implements a [particle filter](https://en.wikipedia.org/wiki/Particle_filter) applied to a [kidnapped robot (car) problem](https://en.wikipedia.org/wiki/Kidnapped_robot_problem). A simulator is provided by Udacity (it could be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)). This simulator will generate noisy landmark observations from a car to the particle filter using [WebSocket](https://en.wikipedia.org/wiki/WebSocket). The particle filter uses a [uWebSockets](https://github.com/uNetworking/uWebSockets) implementation to respond to this observation with the estimated car position. Udacity provides a seed project to start from on this project ([here](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project)).
  
# Prerequisites
The project has the following dependencies (from Udacity's seed project):

* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4
* Udacity's simulator.
  
# Compiling and executing the project
In this project, Udacity's seed repo provides scripts to clean, compile and run it. These are the following commands you need to run from this repo directory

* ./clean.sh
* ./build.sh
* ./run.sh
