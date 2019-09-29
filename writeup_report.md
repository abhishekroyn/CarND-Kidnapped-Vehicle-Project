# **Kidnapped Vehicle** 
---

**CarND-Kidnapped-Vehicle-Project**

The goals / steps of this project are the following:
* Implement a 2 dimensional particle filter in C++., while using given map and some initial localization information (analogous to what a GPS would provide), along with observation and control data at each time step.
* Initialize particle filter based on noisy position data sensed from the simulator.
* Predict the vehicle's next state from previous (noiseless control) data, and add random Gaussian noise to it.
* Update the weights of particle filters according to noisy observation data received from the simulator.
* Resample particle filters with substitution based on their individual weights.
* Calculate and output the average weighted error of the particle filter over all time steps so far.
* Ensure the particle filter localizes the vehicle to within the desired accuracy, and runs within the specified time of 100 seconds.

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/747/view) individually and describe how I addressed each point in my implementation.  

---
### Accuracy

#### 1. Does your particle filter localize the vehicle to within the desired accuracy?

The output showed "Success! Your particle filter passed!" and thus it meant my particle filter was able to localize the vehicle within the desired accuracy.

### Performance

#### 1. Does your particle run within the specified time of 100 seconds?

The output showed "Success! Your particle filter passed!" and thus it meant my particle ran within the specified time of 100 seconds.

### General

#### 1. Does your code use a particle filter to localize the robot?

The code implemented the full particle filter to localize the robot.
