# Unscented Kalman Filter Project Writeup

In this project a unscented kalman filter was implemented in order to estimate the state of a moving object of interest with noisy lidar and radar measurements. 
The RMSE values of estimation error were lower than the tolerance outlined in the project rubric. 


---



## Algorithm

* The algorithm also loops through the predict and measurement update steps like the EKF. The difference lies in the nonlinear 
transformation of process model and measurement model. Instead of linearization, here are the sigma points applied.
* The first step of the algorithm is to update its state according to the first set of measurement.
* Then the sigma points are created according to the number of to estimated state variables. The process noises are also integrated 
into the augment state vector as they have a nonlinear effect.
* The sigma points of the augmented state vector are converted to the sigma points representing the predicted state vector
and then to points representing predicted measuremnet vector. Corresponding mean vector and covariance matrix are cauculated from 
the sigma points.
* With the predicted mean state vector, mean measurement and covariance matrix and real measurement value, the measurement update 
step can be fullfilled.


## Result
After tuning the process noise parameters, the best result I got is [.065, .083, .327, .227].
### Using both Lidar and Radar
<p align="center">
  <img src="/CarND-Unscented-Kalman-Filter-Project/CarND-Unscented-Kalman-Filter-Project/pics/op.png" width="400" />
  <img src="/CarND-Unscented-Kalman-Filter-Project/CarND-Unscented-Kalman-Filter-Project/pics/opnis.png" width="1000" /> 
</p>
The NIS plot shows that the LIDAR measurement provide more accurate result than RADAR and this was also verified by the results in the scenarios
 of only using one of the sensors.
 
### Only using Lidar or Radar

| [![LIDAR](/CarND-Unscented-Kalman-Filter-Project/CarND-Unscented-Kalman-Filter-Project/pics/lidar.png)](/CarND-Unscented-Kalman-Filter-Project/CarND-Unscented-Kalman-Filter-Project/pics/lidar.png)  | [![RADAR](/CarND-Unscented-Kalman-Filter-Project/CarND-Unscented-Kalman-Filter-Project/pics/radar.png)](/CarND-Unscented-Kalman-Filter-Project/CarND-Unscented-Kalman-Filter-Project/pics/radar.png) |
|:---:|:---:|
| LIDAR | RADAR |

<p align="center">
  <img src="/CarND-Unscented-Kalman-Filter-Project/CarND-Unscented-Kalman-Filter-Project/pics/lidar_radar.png" width="1000" /> 
</p>
The result shows that kalmen filter with LIDAR measurment are better than RADAR, yet the result of sensor fusion is far better than both of them and by using the sigma points instead of linearization, the UKF are also superior than the EKF.
