# Path Planning Project
Udacity Self-Driving Car Engineer Nanodegree Program

---

## Introduction
The goal of this project is to safely navigate around a virtual highway with other traffic that is driving +/- 10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data is provided as well as a sparse map list of the waypoints around the highway. The car tries to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible. The car must avoid hitting other cars and must stay inside the marked road lanes at all times, unless changing lanes in which case it has three car lengths to do so. The car must make one complete loop around the 6946m highway. The car can't experience a total acceleration over 10 m/s^2 nor a jerk that is greater than 10 m/s^3.

This was implemented by:
- Determining positions of nearby vehicles
- Determining best path to pass slower vehicles while avoiding collisions
- Determining upcoming waypoints based on determined path

## Implementation

The information from the simulator provides the locations of the vehicles, the ego car location, and waypoints along the track are provided by a text file. The locations of the vehicles and the ego car location are provided in `x` and `y` coordinates which are then transformed to Frenet coordinates, with `s` being the longitudinal coordinates and `d` being the latitudinal coordinates.

### Determine positions of nearby vehicles

A loop cycles through all vehicles in the area of the ego car. Using the `s` value of the ego car plus and minus a variable distance, all cars in the immediate vicinity can be found. If a car is immediately in front of the ego car within a predetermined distance, the ego car slows down until the car immediately in front is out of that predetermined distance. 

### Determine best path to pass slower vehicles while avoiding collisions

If the ego car slows down, it checks the adjacent lanes to see if they are occupied. Checking these lanes includes a buffer zone both in front and behind the location the ego car would transition to in order to prevent potential collisions, tailgating, and cutting off other vehicles (all of these incidents were encountered during development). The buffer zone of the adjacent lanes is made longer than that of the current lane to prevent rapid lane changes. Without this longer buffer zone, a situation can arise in which two cars are side by side in front of the ego car, the ego car changes lanes and is still going too slow so it changes back to the previous lane, continuously and rapidly oscillating between lanes. This implementation detail also adds somewhat of a cost function in that the lane with the most free space ahead is chosen. If the adjacent lane buffer zones are void of any vehicles, the decision to change lanes is made. This is done by changing the waypoint's `d` coordinate and developing a spline to make the smooth transition as discussed below.

### Determine upcoming waypoints based on determined path

The waypoints from the text file map the path along the highway. These waypoints are sparse, so they can't be used to immediately tell the vehicle where to go next. To do this, a [spline](http://kluge.in-chemnitz.de/opensource/spline/) is used to create a smooth path between the provided waypoints. Equally, when changing lanes, this spline ensures a smooth lane change, minimizing jerk. The responsiveness of the ego car can be adjusted by changing the number of waypoints generated ahead. With a higher number of waypoints, the ego car is less responsive. This caused problems during development in which the car immediately in front of the ego car would brake and the ego car would rear end the car in front.

### Potential options for future development

Due to time constraints, I didn't implement everything I wanted to. Here are future implementations I would like to make:

- Convert code to Finite State Machine (FSM) - Currently the code operates as a bunch of for-loops. I would like to convert this to a FSM to provide greater flexibility for additional functionality.

- Use a cost function to determine the best lane to use. Currently, the lane chosen to transition to is based purely on passing a car in front of the ego car and adjacent lane availability. This is a short sighted implementation. The use of a cost function would enable more advanced decision making such as using the left lane only for passing, choosing a lane based on distance to destination (don't leave the right lane if taking the exit immediately ahead), etc.

## Grading Criteria

**The code compiles correctly**: Code must compile without errors with cmake and make.

- Code compiles correctly with no errors nor warnings.

**The car is able to drive at least 4.32 miles without incident**: The top right screen of the simulator shows the current/best miles driven without incident. Incidents include exceeding acceleration/jerk/speed, collision, and driving outside of the lanes. Each incident case is also listed below in more detail.

- Car was tested to 17 miles without any incidents.

**The car drives according to the speed limit**: The car doesn't drive faster than the speed limit. Also the car isn't driving much slower than speed limit unless obstructed by traffic.

- Car drives at approximately 49 MPH unless obstructed by traffic, never exceeding the 50 MPH speed limit.

**Max Acceleration and Jerk are not Exceeded**: The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.

- Max acceleration and jerk are maintained below specifications.

**Car does not have collisions.**: The car must not come into contact with any of the other cars on the road.

- Car was tested to 17 miles without any incidents.

**The car stays in its lane, except for the time between changing lanes**: The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road.

- Car stays in lane with exception of changing lanes when behind a slower vehicle. The three car lengths transition rule is never violated.

**The car is able to change lanes**: The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic.

- Car is able to smoothly change lanes across all available lanes as needed to pass slower vehicles. Prior to changing lanes, the lane being transitioned to is checked to ensure there isn't a vehicle already in that spot and that there is sufficient space in front and behind the intended trajectory to prevent tailgating, cutting off, and potential collisions.

**There is a reflection on how to generate paths**: 

- See above "Implementation" section.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

### Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:

```shell
sudo chmod u+x {simulator_file_name}
```

### Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    
    ```shell
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

