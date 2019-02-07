#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <fstream>


using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;
  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;
  // initial state vector
  n_x_ = 5;
  x_ = VectorXd(n_x_);
  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1;
  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.6;

  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;
  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;
  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;
  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;
  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  is_initialized_ = false;
  previous_timestamp_ = 0;
  R_lidar_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  //measurement covariance matrix - laser
  R_lidar_ << std_laspx_*std_laspx_, 0,
              0, std_laspy_*std_laspy_;
  //measurement covariance matrix - radar
  R_radar_ << std_radr_*std_radr_, 0, 0,
              0, std_radphi_*std_radphi_, 0,
              0, 0, std_radrd_*std_radrd_;
  P_aug = MatrixXd(7, 7);
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

  
  n_aug_ = n_x_ + 2;
  lambda_ = 3 - n_x_;
  
  
  x_aug = VectorXd(7);
//  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  
//  MatrixXd Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  
  // create vector for weights
  weights_ = VectorXd(2*n_aug_+1);
  // set weights
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;
  double weight = 0.5/(n_aug_+lambda_);
  for (int i=1; i<2*n_aug_+1; ++i) {  // 2n+1 weights
    weights_(i) = weight;
  }
//  std::cout << weights_ << std::endl;
  
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  
  /**
   * Initialization
   */
  if (!is_initialized_) {
    
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     */
    std::cout << "UKF: " << std::endl;
    x_ = VectorXd(5);
//    x_ << 1, 1, 1, 1, 1;
    
    
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates
      //         and initialize state.
      double rho = meas_package.raw_measurements_(0);
      double phi = meas_package.raw_measurements_(1);
      double rho_dot = meas_package.raw_measurements_(2);
      double vx = rho_dot*cos(phi);
      double vy = rho_dot*sin(phi);
      
      x_(0) = rho*cos(phi);
      x_(1) = rho*sin(phi);
      x_(2) = sqrt(vx*vx + vy*vy);
      x_(3) = 0;
      x_(4) = 0;
      
    }
    else {
      // TODO: Initialize state.

      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);
      x_(2) = 0;
      x_(3) = 0;
      x_(4) = 0;
      
    }
    
    // done initializing, no need to predict or update
    previous_timestamp_ = meas_package.timestamp_;
    is_initialized_ = true;
    std::cout << "UKF initialized" << std::endl;
    return;
    
  }
  

  
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  if (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER){
    float dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = meas_package.timestamp_;
    std::cout << "Prediction" << std::endl;
    Prediction(dt);
    std::cout << "LASER measurement update" << std::endl;
    UpdateLidar(meas_package);
  }
  else if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR){
    float dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = meas_package.timestamp_;
    std::cout << "Prediction" << std::endl;
    Prediction(dt);
    std::cout << "RADAR measurement update" << std::endl;
    UpdateRadar(meas_package);
  }
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
  
  
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;
  

//  VectorXd x_aug = VectorXd(7);
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;
  
//  std::cout << "x_aug \n" << std::endl;
//  std::cout << x_aug  << std::endl;
  
//  std::cout << "create augmented sigma points" << std::endl;
  // create square root matrix
  MatrixXd A = P_aug.llt().matrixL();
  // create augmented sigma points
  Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i++){
    Xsig_aug.col(1+i) = x_aug + sqrt(lambda_+n_aug_) * A.col(i);
    Xsig_aug.col(1+i+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * A.col(i);
  }
//  std::cout << "sigma points:" << std::endl;
//  std::cout << Xsig_aug  << std::endl;
  
  // Predict
  // predict sigma points
//  std::cout << "predict sigma points:" << std::endl;
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  for (int i = 0; i < 2*n_aug_+1; i++){
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);
    
    
    // avoid division by zero
    double px_p, py_p;
    if (fabs(yawd) > 0.001) {
      px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
      py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    } else {
      px_p = p_x + v*delta_t*cos(yaw);
      py_p = p_y + v*delta_t*sin(yaw);
    }
    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;
    
    // add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;
    
    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;
    // write predicted sigma points into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }
  
  // predict state mean
//  std::cout << "predict state mean:" << std::endl;
  

  

  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

//  x_ = Xsig_pred_ * weights_;

  // predict state covariance matrix
  P_.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    
    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;

    
  }
//  std::cout << "prediction finished" << std::endl;
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  int n_z = 2;
  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  
  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // extract values for better readability
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    
    // measurement model
    Zsig(0,i) = p_x;                       // x
    Zsig(1,i) = p_y;                                // y
  }
  
  // mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; ++i) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
  
  // innovation covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
  
  // add measurement noise covariance matrix
  S = S + R_lidar_;
  
  // Update
  double p_x = meas_package.raw_measurements_(0);
  double p_y = meas_package.raw_measurements_(1);
  VectorXd z = VectorXd(n_z);
  z << p_x, p_y;
  
  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  
  // calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  VectorXd z_diff = z - z_pred;
  
  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
//  std::cout << "LIDAR measurement update finished" << std::endl;
  //NIS Lidar Update
  std::cout << "calculation the NIS_Laser" << std::endl;
  NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
  std::ofstream myfile2;
  myfile2.open ("lidar.txt", std::ios_base::app);
  myfile2 << NIS_laser_ <<"\n";
  myfile2.close();
  
  NIS_ = NIS_laser_;
  std::ofstream myfile3;
  myfile3.open ("nis.txt", std::ios_base::app);
  myfile3 << NIS_ <<"\n";
  myfile3.close();
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  int n_z = 3;
  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  
  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // extract values for better readability
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);
    
    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;
    
    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                       // r
    Zsig(1,i) = atan2(p_y,p_x);                                // phi
    Zsig(2,i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);   // r_dot
  }
  
  // mean predicted measurement
//  std::cout << "mean predicted measurement" << std::endl;
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; ++i) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
  
  // innovation covariance matrix S
//  std::cout << "innovation covariance matrix S" << std::endl;
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    
    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
  
  // add measurement noise covariance matrix
  S = S + R_radar_;
  
  // Update
  double rho = meas_package.raw_measurements_(0);
  double phi = meas_package.raw_measurements_(1);
  double rho_dot = meas_package.raw_measurements_(2);
  VectorXd z = VectorXd(n_z);
  z << rho, phi, rho_dot;
  
  // create matrix for cross correlation Tc
//  std::cout << "create matrix for cross correlation Tc" << std::endl;
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  
  // calculate Kalman gain K;
//  std::cout << "alculate Kalman gain K" << std::endl;
  MatrixXd K = Tc * S.inverse();
  VectorXd z_diff = z - z_pred;
  
  // angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
//  std::cout << "RADAR measurement update finished" << std::endl;
  std::cout << "calculation the NIS_Radar" << std::endl;
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
  
  std::ofstream myfile1;
  myfile1.open ("radar.txt", std::ios_base::app);
  myfile1 << NIS_radar_ <<"\n";
  myfile1.close();
  
  NIS_ = NIS_radar_;
  std::ofstream myfile3;
  myfile3.open ("nis.txt", std::ios_base::app);
  myfile3 << NIS_ <<"\n";
  myfile3.close();
  
}
