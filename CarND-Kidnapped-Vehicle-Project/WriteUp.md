# Particle Filter Project Writeup

In this project a particle filter was implemented in order to estimate the position and motion direction of a moving object 
with the help of a map which describes its surrounding. 


---



## Algorithm

* The algorithm also loops through the predict and measurement update steps like the EKF and UKF. 
* The first step of the algorithm is to create a certain number of particles based on the GPS position and add noise to them 
according to the standard deviation of gps measurements. The total number of particles need to be big enough to represent all posible
states.
* Prediction step: all particles will be updated according to their previous state, the motion model and input velocity as well as 
yaw change rate. Additional noise are added in order to represent all posible state.
* Masurenment update step: the weight of each particle will be calculated by comparing their prediction and the real measurement.
  - Data association: Firstly, the observed landmarks will be transfered to the map coordinate based on the particle position. Then these tranfered landmarks
 will be compared with all the landmarks on the map (exclude all map landmarks outside of the sensor range of that particle) according 
 to the nearest neighbor method.
  - Weight update: The distance of associated observation and landmarks indicates the accuracy of the particle, by multipling all the gaussian probabiliy
  of each observation, the weight of that particle is determined.
  - Resample: Based on the weight of each particle, there will be the exactly same amount of new particles duplicated, the weights reture back to 1 and loop 
 back to prediction step. 



## Result
Simulator result with youtube link.
[![IMAGE ALT TEXT HERE](https://github.com/JiashengYan/CARND_TERM2/blob/master/CarND-Kidnapped-Vehicle-Project/CarND-Kidnapped-Vehicle-Project/youtubecover.jpg)](https://youtu.be/h6PUehT8cfY)
