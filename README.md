# Sensor-Fusion-with-Kalman-Filters

---

Overview
---
This project uses lidar measurements and radar measurements to track an object's position and velocity that travels around the vehicle. 
Sensor data is processed by an extended Kalman filter in C++.  


Data set
---
The project provides simulated lidar and radar measurements of detecting a bicycle that travels around the vehicle. 
```sh
data/sample-laser-radar-measurement-data-1.txt
data/sample-laser-radar-measurement-data-2.txt
```

src files
---

<b> main.cpp </b> - reads in data, calls a function to run the Kalman filter, calls a function to calculate RMSE

<b> FusionEKF.cpp </b>- initializes the filter, calls the predict function, calls the update function

<b> kalman_filter.cpp </b> - defines the predict function, the update function for lidar, and the update function for radar

<b> tools.cpp</b> - function to calculate RMSE and the Jacobian matrix


## Dependencies

* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
   ./ExtendedKF ../      data/sample-laser-radar-measurement-data-1.txt output1.txt

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Reference: 

* Starter code was provided by Udacity, as part of Udacity Self-Driving Car Engineer Program.

* <a href="http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/">How a Kalman filter works, in pictures </a>

* <a href="https://medium.com/towards-data-science/kalman-filter-intuition-and-discrete-case-derivation-2188f789ec3a">Vivek Yadav, Kalman filter: Intuition and discrete case derivation</a>

