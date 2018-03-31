#Path Planning Project
## Objective
The goal of this project is to build a path planner that takes a set of waypoints and drives a car on a 3 lane highway along with other car traffic.  It is also required not to have any collision with other cars, not have any acceleration and jerk of 10m/s2 and 10m/s3 and also stay within the maximum permissible limits. It should not stay outside lane for more than 3s except during changing lanes when stuck behind a slow moving vehicle. 

## Approach
I first started with code for trajectory generator and then added a behavior planner using cost functions to choose the best possible successor state. 

1) Trajectory Generation

I used the splines header file discussed in the project walkthrough session as opposed to using polynomial trajectory generation as mentioned in the lectures to keep the implementation as simple as possible.Using the helper functions to convert from XY to Frenet or vice versa, I was able to generate the path coordinates. I calculated the yaw angle and either took the previous path coordinates when available or calculate two based on yaw angle and then added 


