# Path planning project (Omar Benzakour)

## Goals
The goal of this project is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. We are provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Building instructions

### Simulator

This project code will spawn a server that will be waiting for a connection from the simulator on port 4567. You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

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
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Some details about the provided data

**data/highway_map.csv** : there is a list of waypoints that go all the way around the track. The track contains a total of 181 waypoints, with the last waypoint mapping back around to the first. The waypoints are in the middle of the double-yellow dividing line in the center of the highway.

**highway** : The highway has 6 lanes total - 3 heading in each direction. Each lane is 4 m wide. The Frenet coordinates are in metters. Therefore being in the middle of the left lane that is the closest to the double yellow lane is transcribed by the d coordinates 2 

**getXY** : translates the Frenet coordinates into x and y coordinates

**sensor_fusion** contains all the information about the cars on the right-hand side of the road. The data format for each car is: [ id, x, y, vx (m/s), vy (m/s), s, d].

**Main car's localization Data (No Noise)** :[x, y, s, d, yaw, speed (mph)]

## System and latency

**controller**: The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.)

**Latency**: The C++ path planner will at the very least be one cycle behind the simulator because the C++ program can't receive and send data on the same cycle. As a result, any path that the simulator receives will be from the perspective of a previous cycle. This might mean that by the time a new path reaches the simulator, the vehicle has already passed the first few waypoints on that path. In this project, we don't have to worry about this too much. The simulator has built-in tools to deal with this timing difference. Indeed, the system stores the last points we have used to have a smooth transition. The system therefore extends extends the previous path to make sure the new path has a smooth transition with this last path.

## Prediction

The **sensor fusion** data provides information about the neighbouring cars at time (t). In the following section, we will predict the behaviour of the neighbouring cars for the folllowing 20 time intervalles [t, ..., t +20T] based on the supposition that the cars keep the same speed (s dot) on the road and that the curb remains almost the same for 1s

The speed vector of the cars are collinears to the road. Therefore the value of the vector is equal to: sqrt(vx^2 + vy^2). From there we can predict the position of the car for the next second.


## Staying in lane


We are provided with the coordinates of the circuit. We can therefore translate (x,y) coordinates into (s,d) and reciprocally. We need now to select wich points will constitute our trajectory. Blindly taking points in the s trajectory is not feasible because the road can have sharp turns that will not abide by the bicycle model of the car. Furthermore because of the speed and acceleration contraints, further rectrictions apply over the distance between 2 points in that trajectory. The following sections will discuss how we mitigted these issues

### Abiding by the bicycle model

Instead of taking all our points on the s line, we will select 5 points and fit a spline that will pass by these 5 points.
Since we want the polynom to be continuous with the previous trajectory, we need it to pass by the 2 previous points. We select 2 points instead of 1 in order to have a continuity for the yaw of the car as well. Initially, we don't have any previous trajectory. We will therefore create a point behind our current position so that the line formed with both these points will have a angle equal to the current yaw. The next 3 points will be respectively distant of 30, 60 and 90 m from the initial points. 
The trajectory will be smooth and have no sharp edges because the polynoms are infinitely differentiable. We will still need to check that the curvature of the polynom abides by the bicycle model of the car.

In order to fit a polynom into these 5 landmark points we will switch from the (x,y) coordinates to the car coordinates. Doing so will simplify the maths while avoiding cornercases.

### Abiding by the max speed and acceleration constraints

Between a point a t and t+TIME_PERIOD we will have the following updates:

a(t + T) = (v(t+T)-v(t))/T

v(t + T) = (distance between 2 consecutive points) / T

From the relation above, we can derive the max and min speed increase and therefore we can derive the max and min distance between 2 consecutive points.

To pick these points we can either solve equations or use the approximation given in the QA section of this project. The approximation holds because the used spline doesn't have rapid increase over a short period of time.

### Avoiding collisions

Avoiding rear ending the car in front of us is the result of the prediction step and the speed control. Indeed, we can avoid the collision by reducing the speed of the ego car to the car in front of us while keeping a safety distance 


## Changing lane

When the position of the car in front of us is less than 30 m we will try to change lane. We will try to go to the left or right lane only if we can maintain this 30m safety distance with the cars located on the adjacent lanes. If we can't we will adjust our speed to the speed of the car in front of us.
Changing line is therefore changing the lane of the desired end state. Because the safety distance is consistant between staying in lane and changing lane we know that there is a polynom that respects the kinematic constrains of the car and the start and end states. Furthermore this polynom is optimal because the end speed is maximal. This allows us to bypass the need of sampling several end states and ranking all the fitted polynoms based on a cost function. Off course this comes at a cost that will be discussed in the following paragraph



## Solution shortcomings and further discussion

The provided code that answers the goals of this project but falls short in some situations:

1. In the points selection sections, we used the approximation made in the QA section of the project. This approximation holds because the curvature of the highway is low. Let us call l the line formed by the position of the car (0,0) and (target_x, target_y). We assume that the distance between (s(a), s(b)) with a and b between [0, target_x] will be smaller to the distance between (l(a), l(b) 
2. Imagine we are in a position where we can't change lane. If the car in front of us makes a sudden speed change that violates the acceleration constrains that we have. we will rear end it because we always make sure that we always respect the constrains set by the project
3. Imagine our ego car is in the leftmost lane (lane 0) and that there are 2 cars with the same s that go at the same speed on lane(0) and lane(1). Our changing lane condition will never be met, we will therefore remain on lane(0) with the same speed as the car in front of us. A solution to this problem would have been to change the lane based on the maximum speed at which we can go. Therefore in this scenario, our car will see that there is no car on the lane 2 and will try to go there using lane 1.
4. In case there is a car at our right (lane 1) and a car in front of us (lane 0) going at the same speed, our ego car will not try to go the lane 2, decreasing its speed and letting these 2 cars go



