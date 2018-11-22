# Overview
This repository contains my C++ code solution to the project of Kidnapped Vehicle for the Localization course in Udacity's Self-Driving Car Nanodegree. In this C++ project, a 2-dimensional particle filter is implemented to help localize a vehicle placed in an unknown place. The particle filter is initialized by less accurate GPS data. Then it predicts next location based on velocity and yaw rate with noise. After that it transforms sensor observations (measurements) into map coordinates and associates the observations with the landmarks on the map and. Each particle updates its weight (likelihood of a given particle's observation based on the landmark positions on the map) from the calculation of multi-variate Gaussian distribution. Then we resample the particles based on its weight, which would help the accuracy to localize the vehicle.


[//]: # (Image References)
[image1]: ./examples/particlefilter_steps.png "steps"
[image2]: ./examples/motion_model.png "motion model"
[image3]: ./examples/coordinate_transformation.png "coordinate transform"
[image4]: ./examples/multi-variate_gaussianDistribution.png "multi-variate_gaussianDistribution"
[image5]: ./examples/roulette_wheel_selection.png "roulette_wheel_selection"

## Brief Introduction to Particle Filter Algorithm
The particle filter algorithm is illustrated as below.

![alt text][image1]

As you can see, it includes four steps:

##### 1) Initialization Step  
The most practical way to initialize our particles and generate real time output, is to make an initial estimate using GPS input. As with all sensor based operations, this step is impacted by noise.

##### 2) Prediction Step
After we initialize our particles. It's time to predict the vehicle's position. Here we will use what we learned in the motion models lesson to predict where the vehicle will be at the next time step, by updating based on yaw rate and velocity, while accounting for Gaussian sensor noise.  
Below is the motion model. The left one is applied when yaw rate is zero. The right one is calculated when yaw rate is nonzero.

![alt text][image2]

##### 3) Update Step  
After we incorporate velocity and yaw rate measurement inputs into our filter, we must update particle weights based on LIDAR and RADAR readings of landmarks.  

First, we need to transform the observation points to map coordinates (given in vehicle coordinates). This `homogenous transformation matrix`, shown below, performs rotation and translation. x<sub>p</sub> and y<sub>p</sub> are map particle coordinates.  x<sub>c</sub> and y<sub>c</sub> are the car observation coordinates. They can be transformed into map coordinates x<sub>m</sub> and y<sub>m</sub>. Theta is the rotation angle.

![alt text][image3]

Second, after observations are transformed into the map's coordinate space, the next step is to associate each transformed observation with a land mark identifier.

Third step is the calculation of multi-variate Gaussian distribution. The particles final weight will be calculated as the product of each measurement's Multivariate-Gaussian probability density. The Multivariate-Gaussian probability density has two dimensions, x and y. The mean of the Multivariate-Gaussian is the measurement's associated landmark position and the Multivariate-Gaussian's standard deviation is described by our initial uncertainty in the x and y ranges. The Multivariate-Gaussian is evaluated at the point of the transformed measurement's position. The formula for the Multivariate-Gaussian can be seen below.

![alt text][image4]

##### 4) Resample Step     
Resample step is to resample particles, with replacement occurring based on weighting distributions. One way to do it is the roulette wheel selection. A proportion of the wheel is assigned to each of the possible selections based on their weight value. This can be achieved by dividing the weight of a selection by the total weight of all the selections, thereby normalizing them to 1. Then a random selection is made similar to how the roulette wheel is rotated.

![alt text][image5]

This course introduces *Roulette-wheel selection via stochastic acceptance* by Sebastian Thrun. I used this method for the resample step in C++ code.


The description below from original repo https://github.com/udacity/CarND-Kidnapped-Vehicle-Project.

#### Submission
All you will submit is your completed version of `particle_filter.cpp`, which is located in the `src` directory. You should probably do a `git pull` before submitting to verify that your project passes the most up-to-date version of the grading code (there are some parameters in `src/main.cpp` which govern the requirements on accuracy and run time.)

## Project Introduction
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project you will implement a 2 dimensional particle filter in C++. Your particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data.

## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Note that the programs that need to be written to accomplish the project are src/particle_filter.cpp, and particle_filter.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

// sense noisy position data from the simulator

["sense_x"]

["sense_y"]

["sense_theta"]

// get the previous velocity and yaw rate to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values

["sense_observations_x"]

["sense_observations_y"]


OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"]

//Optional message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label

["best_particle_associations"]

// for respective (x,y) sensed positions

["best_particle_sense_x"] <= list of sensed x positions

["best_particle_sense_y"] <= list of sensed y positions


Your job is to build out the methods in `particle_filter.cpp` until the simulator output says:

```
Success! Your particle filter passed!
```

# Implementing the Particle Filter
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

The only file you should modify is `particle_filter.cpp` in the `src` directory. The file contains the scaffolding of a `ParticleFilter` class and some associated methods. Read through the code, the comments, and the header file `particle_filter.h` to get a sense for what this code is expected to do.

If you are interested, take a look at `src/main.cpp` as well. This file contains the code that will actually be running your particle filter and calling the associated methods.

## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory.

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.

## Success Criteria
If your particle filter passes the current grading code in the simulator (you can make sure you have the current version at any time by doing a `git pull`), then you should pass!

The things the grading code is looking for are:


1. **Accuracy**: your particle filter should localize vehicle position and yaw to within the values specified in the parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`.

2. **Performance**: your particle filter should complete execution within the time of 100 seconds.
