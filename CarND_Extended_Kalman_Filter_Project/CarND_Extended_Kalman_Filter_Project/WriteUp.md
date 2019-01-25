# Extended Kalman Filter Project Writeup

In this project a extended kalman filter was implemented in order to estimate the state of a moving object of interest with noisy lidar and radar measurements. 
The RMSE values of estimation error were lower than the tolerance outlined in the project rubric. 


---

[image1]: ./CarND_Extended_Kalman_Filter_Project/CarND_Extended_Kalman_Filter_Project/Bildschirmfoto1.png "Bildschirmfoto1"
[image2]: ./CarND_Extended_Kalman_Filter_Project/CarND_Extended_Kalman_Filter_Project/Bildschirmfoto2.png "Bildschirmfoto2"
[image3]: ./CarND_Extended_Kalman_Filter_Project/CarND_Extended_Kalman_Filter_Project/dataset1.png "dataset1"
[image4]: ./CarND_Extended_Kalman_Filter_Project/CarND_Extended_Kalman_Filter_Project/dataset2.png "dataset2"

## Algorithm

* The algorithm loops through the predict and measurement update steps during running against the dataset.
* The first step of the algorithm is to update its state according to the first set of measurement.
* The algorithm record the time interval of previous measurement as timestep and update the transformation matrix F and process 
covariance Q.
* Before the meaturement update step, the measurement matrix H and measurement covariance matrix R switch based on sensor type.

## Result
As all the parameters are predefined, the result satisfied the requirement [.11, .11, 0.52, 0.52]. without any need of tunning.
### Dataset 1
<p float="left">
  <img src="/CarND_Extended_Kalman_Filter_Project/CarND_Extended_Kalman_Filter_Project/Bildschirmfoto1.png" width="400" />
  <img src="/CarND_Extended_Kalman_Filter_Project/CarND_Extended_Kalman_Filter_Project/dataset1.png" width="400" /> 
</p>

### Dataset 2
<p float="left">
  <img src="/CarND_Extended_Kalman_Filter_Project/CarND_Extended_Kalman_Filter_Project/Bildschirmfoto2.png" width="400" />
  <img src="/CarND_Extended_Kalman_Filter_Project/CarND_Extended_Kalman_Filter_Project/dataset2.png" width="400" /> 
</p>
