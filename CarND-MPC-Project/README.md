# CarND-Controls-MPC
Udacity Self-Driving Car Engineer Nanodegree Program

---

## Summary

This project navigates a car around a track in a simulator. The trajectory of the car is determined by fitting a third-degree polynomial to waypoints which correspond to the center of the track. To keep computation time to a minimum, the fitted polynomial only considers a short time period worth of waypoints ahead of the vehicle. This fitted polynomial is based on the vehicle kinematics, cross-track error (distance from the lane center), orientation angle error, and actuations (to reduce erratic movements). To simulate a real-world vehicle, the model implements a 100 millisecond latency. The model communicates with a Udacity-provided simulator via websocket. Telemetry and waypoint data is sent from the simulator to the model and steering and acceleration commands are sent to the simulator.

## Grading Criteria

1. **Your code should compile:** Code must compile without errors with cmake and make.

It does, test it out!

2. **The Model:** Student describes their model in detail. This includes the state, actuators and update equations.

The model takes as inputs the vehicle's x and y coordinates, orientation angle ($\psi$), velocity ($v$), cross-track error ($cte$), and the orientation angle error ($e \psi$). The outputs (actuations) from the model are acceleration ($a$) and steering angle ($\delta$). The state and actuations for the current timestep are determined from those of the previous timestep. The equations for doing so are as shown:

$$x_{t+1} = x_t + v_t * cos(\psi_t) * dt$$
$$y_{t+1} = y_t +  v_t * sin(\psi_t) * dt$$
$$\psi_{t+1} = \psi_t + \frac{v_t}{L_f} * \delta_t * dt$$
$$v_{t+1} = v_t + a_t * dt$$
$$cte_{t+1} = f(x_t) - y_t + (v_t * sin(e \psi_t) * dt)$$
$$e \psi_{t+1} = \psi_t - \psi des_t + (\frac{v_t}{L_f} * \delta_t * dt)$$

3. **Timestep Length and Elapsed Duration (N & dt):** Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

The value chosen for the timestep length (N) is 10 and elapsed duration between timesteps (dt) is 0.1. These values were chosen by empirically testing many values, other values result in the car driving erratically. These values result in the fitted polynomial being applied to the track waypoints one second in the future.

4. **Polynomial Fitting and MPC Preprocessing:** A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

The preprocessing consists of transforming the waypoints to the vehicle's perspective, this sets the vehicle's x and y coordinates to the origin (0,0) and sets the orientation angle to zero. This, in turn, makes fitting a polynomial to the waypoints easier.

5. **Model Predictive Control with Latency:** The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

The kinematic equations presented in the class were based on one timestep prior, but a 100 millisecond latency results in the actuations being applied a timestep later. To compensate for this, the actuations have been altered to use the values from two timesteps prior (see MPC.cpp lines 119-122).

The cost function was also altered to include acceleration and steering angle to minimize excessive actuations and improve performance around corners.

6. **The vehicle must successfully drive a lap around the track:** No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle). The car can't go over the curb, but, driving on the lines before the curb is ok.

It does, test it out!

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

