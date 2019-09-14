# **Autonomous Vehicle PID Controller for Steering**
---
**Self-Driving Car Engineer Nanodegree Program**

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## Project Goals
---
My goal in this project is to implement a PID controller in C++ to maneuver the vehicle around the track and satisfy the following conditions.

* The car must successfully drive a lap around the track. No tire may leave the drivable portion of the track surface. 
* The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle).

## Project Setup
---
The project uses a Udacity Simulator, which provides the C++ program the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle.

Details of the Simulator are provided in the later section of this README file.

## Project Implementation
---
The PID Controller is represented by the `PID class` implemented in the [PID.h](./src/pid.h) and [PID.cpp](./src/PID.cpp). The class member function `UpdateError(...)` updates the PID's proportional, integral, and differential error values and the member function `TotalError(...)` computes the total error based on these error values and their respective `control gains (coefficients)` `Kp`, `Ki`, and `Kd`.

Currently, I have the control gains as hardcoded value defined in the `main` function in [main.cpp](./src/main.cpp). The 'main' function does the message handling to and from the Simulator and provides the Simulator the value returned by the 'TotalError' function as the new 'steering_angle'.

I have implemented the code for `control gain` parameter tuning using `PID class` member functions `TuneControlGains(...)`, `SimulateTrajectory(...)`, and `SimulateMove(...)`, but the code is still experimental and disabled by default.

The PID controller executable takes in 4 optional command-line parameters in the following sequence:
1. `Kp` - (double) Proportional Control Gain
2. `Ki` - (double) Integral Control Gain 
3. `Kd` - (double) Differential Contron Gain
4. `enable_tuning` - (int) 1/0 To enable or disable parameter tuning (default 0) 

Example commands to run the program:
1. `./pid` (default and recommended  way)
2. `./pid 0.15 0.0 2.5` (Set control gain params, param tuining disabled)
3. `./pid 0.15 0.0 2.5 1` (Set control gain params, param tuining enabled)

## Reflection
---

Although the current hardcoded control gain values work to satisfy the project goals, parameter tuining is certainly required to ensure a smoother ride and increased speed of the car.

I manually tested the control gains Kp, Ki, and Kd for the P, I, and D components and found the following observations:

* Setting only the Kp (Proportional Control Gain) value, the car steers towards the intended trajectory line (lane center) but overshoots it every time and keeps zig-zaging aroung the center line.

* Setting only the Ki (Integral Control Gain), makes the car start with a wide zig-zag motion and then makes it run in circles. This compoment, which is used to correct for any bias or drift in the system, is not currently used as it is not deemed necessary at this time.

* Setting only the Kd (Differential Control Gain) tries to correct for the error but only in a small proportion as it acts on the error differential. Though, when used along with the Kp, it corrects for the overshooting by smoothing the car's approach towards the center line.

As mentioned above, I am still experimenting with the parameter tuning, so have currently chosen hardcoded vaules for the `Control Gains`, based on manual experimentation.   

## Simulator
---
You can download the Udacity Simulator from the releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Code Style

I have did my best to stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).
