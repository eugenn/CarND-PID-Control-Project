# CarND-Controls-PID

---

## Introduction
A proportional–integral–derivative controller (PID controller) is a control loop feedback mechanism (controller) commonly used in industrial control systems. In this case the PID controller is used to control a vehicle steering according to cross track error (cte) provided by udacity simulator.

```
steering = -Kp * CTE - Kd * diff_CTE - Ki * int_CTE
```

## Rubric Discussion Points

The PID controller is made up fo 3 components:

P - Proportional, this value depends on the present error. For example, if the error is large and positive, the control output will also be large and positive. 

D - Differential, prediction of future errors. This component is proportional to the rate of change. (it "accelerates" reaction)

I - Integral, the accumulation of past errors. The controller is "filling" and his reaction increases.

Finally parameters

PID parameters used for steering angles:
P: 0.2
I: 0.004
D: 3.0

PID parameters used for throttle:
P: 0.5
I: 0.0
D: 0.5

#### How to tune the parameters

The parameters are tuned manually with the order of: P, D, I. The last params settled to be zeros, and 0.2 is used for the P value. After that I start tune I and D params. The same approach I choose for 
throttle controller which is depends on two values P and D. In case when the Integral value not zero the speed of the car continuously grow and quite hard stabilize the car trajectory.

In order to automatically fine tune the parameters, an optimization algorithm the Twiddle can be used. I implemented it for tuning the steering controller but the convergence was not so fast.

Demo: speedy drive, which is targeting for driving the car as fast as possible, but as a side effect, the car starts to swing. In order to make the car drive fast as well as steady, further joint parameter tuning for both PID controllers need to be carried out.

## Summary
As video shows, the vehicle can successfully drive a many laps around the track. The project very good visualize and give "feeling" about of each component of the PID controller.
It was interesting to compare the results with a similar past project the Behavioral cloning when the car was controlled by CNN.
## Dependencies

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 