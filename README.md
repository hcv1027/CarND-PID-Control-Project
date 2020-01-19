# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Descriptions

In this project, I implement a PID controller to control the vehicle's steer to let it drive in the center of the lane.

### P: Proportional factor

The output of the proportional factor is the product of gain `Kp` and measured error `cte` (cross track error). The higher the proportional gain `Kp` or the error `cte`, the output of proportional factor is higher, too. Setting `Kp` too high will cause the vehicle overshoot too much, but setting it too low will let vehicle drive out of the track in a big curve. [This video](https://youtu.be/PAan8YH93cE) is the result of only using the proportional factor, setting `Kp` as `0.171`. You can see that the vehicle overshoot too much and finally drive out of the track. Also, only use proportional factor will cause the effect of oscillation.

### D: Derivative factor

The derivative factor is used to overcome the overshoot problem. It considers the rate of change of `cte` with respect to time, output the product of gain `Kd` and the rate of change of `cte`. [This video]( https://youtu.be/wfIoij7OXaw) show the result of pd-controller with the `Kp = 0.171` and `Kd = 1.0305`. You can see that the vehicle can successfully drive a lap around the track now, and the oscillation has reduced a lot.

### I: Integral factor

The real world is not perfect, it always exists the system bias, so we need one more factor to handle it, the integral factor. It integrates the error over a period of time, and output the product of gain `Ki` and the total error. If the current total error is low, it outputs a low value, too. Otherwise it will have obvious effect to let vehicle drive back to center of the track quickly. [This video]( https://youtu.be/bR08o61I8nc) shows the result of fully PID-controller with the `Kp = 0.171`, `Kd = 1.0305` and `Ki = 0.000015`. Comparing with the pd-controller, you can notice that the vehicle drives more smoothly at certain big turns.

### Tuning hyperparameter: Twiddle

I also implement the twiddle algorithm to auto fine tune the hyperparameters, but sadly speaking, I don't think the result is better than the one tuned by myself. So the final hyperparameters are the one tuned by myself. I think there are two problems:<p>
1. The scale of three gains are too different, the proper range of `Kp` term is about 0.1 ~ 0.2. The range of `Kd` is about 1.0 ~ 3.0. And the range of `Ki` is about 0.00001 ~ 0.00002. So if I set the twiddle torrance to 0.001, the twiddle will have little chance to fine the best`Ki` gain. But if I set the torrance to a very small value, it will take a very long time to get the final result.
2. The initial guess of hyperparamters has a big influence on the final result. If the initial guess is bad, then twiddle will have no chance to reach the final good one.

### Throttle

I create another pid-controller to control the throttle. In the turning process, I found that the highest steady speed is around 38~39 mph. So I use a pid-controller to control the vehicle's throttle to reach the target speed 38.5 mph. But I didn't pay too much time on this pid-controller's hyperparameters. Just let it can drive  smoothly.


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

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

