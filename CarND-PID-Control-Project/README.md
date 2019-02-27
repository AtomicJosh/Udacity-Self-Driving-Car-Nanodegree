# PID Controller Project
Udacity Self-Driving Car Engineer Nanodegree Program

---

## Overview

A PID controller is commonly used in control systems to minimize the error to a target. In the case of an autonomous vehicle, the error is called the cross-track error (CTE), which is the distance between the car and the center line of the lane. The components of the PID controller are proportional (P), integral (I), and differential (D).

## PID Component Effects

* The proportional component of the PID controller tries to minimize the cross-track error (CTE) linearly. With the proportional variable alone, the car quickly overshoots the center line and goes off the road.

* The integral component of the PID controller tries to minimize the bias of the controller which could prevent the cross-track error from reaching 0. With the integral variable alone, the car goes in circles.

* The differential component of the PID controller helps minimize the overshoot of the center line which the proportional variable tends to cause. This results in a gradual approach to the center line (a gradual reduction of CTE to 0).

## Choosing Parameters

The parameters were initially chosen by trial and error, trying to minimize the oscillations. Once the oscillations were reduced to satisfactory level, the parameters were optimized by implementing the twiddle algorithm. The twiddle algorithm gradually adjusts the parameters to minimize the error. A python implementation of this is as follows:

```python
def twiddle(tol=0.2):
    p = [0, 0, 0]
    dp = [1, 1, 1]
    robot = make_robot()
    x_trajectory, y_trajectory, best_err = run(robot, p)

    iteration = 0
    while sum(dp) > tol:
        print("Iteration {}, best error = {}".format(iteration, best_err))
        for i in range(len(p)):
            p[i] += dp[i]
            robot = make_robot()
            x_trajectory, y_trajectory, err = run(robot, p)

            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                robot = make_robot()
                x_trajectory, y_trajectory, err = run(robot, p)

                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
        it += 1
    return p
```

In my implementation, if the twiddle algorithm is turned on (set by the boolean variable `twiddle_on`, the car will be run many times on the track to determine the best parameters. Left running long enough, this will reach the parameters [P:0.111051, I:0.0005641, D:1.7979]

## Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)


