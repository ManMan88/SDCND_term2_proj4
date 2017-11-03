# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

#### What's a PID controller

A proportional–integral–derivative (PID) controller is a control loop feedback mechanism widely used in industrial control systems and a variety of other applications requiring continuously modulated control. A PID controller continuously calculates an error value as the difference between a desired setpoint and a measured process variable and applies a correction based on proportional, integral, and derivative terms.

#### The influence of the PID parameters
1. **Proportional gain**, as it name implies, modifies the input proportionally to the error. If the proportional gain is too high, the system can become unstable.
2. **Integral gain** modifies the input proportionaly to the sums of the error during the operation of the system. It allows the controller to eliminate the residual steady-state error and is able to deal with systematic biases.
3. **Derivative gain** modifies the input proportionaly to the error's change rate. In other words, if the desired setpoint is getting closer (= error is decreasing) the response must be smoothed in order not to overshoot the target. Derivative component benefits the system's stability and settling time.

Here are some gif examples for the situations without a controller, with a P controller and with a PID controller:

<table style="width:100%">
  <tr>
    <th>
      <p align="center">
       <img src="./gif/control_none.gif" alt="Overview" width="80%">
      </p>
    </th>
    <th>
      <p align="center">
       <img src="./gif/control_p.gif" alt="Overview" width="80%">
      </p>
    </th>
    <th>
      <p align="center">
       <img src="./gif/control_pid.gif" alt="Overview" width="80%">
      </p>
    </th>
  </tr>
  </table>

#### How I chose the PID Parameters
First, I manually tuned the parameters in order to find a reasonable controller that can steer the car at the beginning of the track. Then, I used a modification of the [twiddle method](https://martin-thoma.com/twiddle/) in order to fine tune the paramers (can be observed in the twiddle branch). The parameter I was trying to minimize was the squared cte. I ran it for 20 epochs, 300 steps for each epoch.
The twiddle method was able to find PID parameters that steered the car safely around the track.

After achivivng this goal, I genertaed a PID controller for the velocity (where the throttle is the control input).
I used some manually tuned PID parameters for this controller. Then, I created a function that controls the target speed of the car based on the cte value, steer value of the car, the angle of the car and the last speed value. Basically, as the cte, steer value and angle values increased, the function decreased the target speed. The last speed value was used in order to apply a simple linear filter over the speed so it will not be jumpy.
Lastly, I ran the new model with the speed PID theough the twiddle process in order to finetune the sterr PID values again, and these are the final parameters I chose for the project.

---

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.
