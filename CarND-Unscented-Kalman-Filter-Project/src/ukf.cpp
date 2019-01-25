#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
* Initializes Unscented Kalman filter
*/
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.5;

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

  // State dimension
  n_x_ = x_.size();

  // Augmented state dimension
  n_aug_ = n_x_ + 2;

  // Number of sigma points
  n_sig_ = 2 * n_aug_ + 1;

  // Sigma point spreading parameter
  lambda_ = 3 - n_x_;

  // Predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, n_sig_);

  // Weight vector
  weights_ = VectorXd(n_sig_);

  // Set to True when measurements are initialized
  is_initialized_ = false;

  // Initialize timestamp in microseconds
  previous_timestamp_ = 0.0;
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  // Ensure only RADAR or LASER sensors are used
  if ((meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) ||
      (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)) {
    
    ///// Initialization /////
    
    if (!is_initialized_) {
      // Initialize state vector 
      x_ << 1, 1, 1, 1, 0.3;

      // Initialize covariance matrix
      P_ << 0.1, 0   , 0, 0, 0,
            0   , 0.1, 0, 0, 0,
            0   , 0   , 1, 0, 0,
            0   , 0   , 0, 1, 0,
            0   , 0   , 0, 0, 1;

      // Initialize the timestamp
      previous_timestamp_ = meas_package.timestamp_;

      if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
        x_(0) = meas_package.raw_measurements_(0); // px
        x_(1) = meas_package.raw_measurements_(1); // py
      }
      else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
        // Convert from polar to cartesian coordinates
        x_(0) = meas_package.raw_measurements_(0) * cos(meas_package.raw_measurements_(1)); // px
        x_(1) = meas_package.raw_measurements_(0) * sin(meas_package.raw_measurements_(1)); // py
      }

      // State has been initialized
      is_initialized_ = true;
      return;
    }

    // Calculate the time between measurements
    float dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = meas_package.timestamp_;

    Prediction(dt);

    //  Update state
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      UpdateLidar(meas_package);
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      UpdateRadar(meas_package);
    }
  }
}

void UKF::Prediction(double delta_t) {
  ///// Generate Sigma Points /////

  // Calculate square root of P
  MatrixXd P_sqrt_ = P_.llt().matrixL();

  // Set lambda for non-augmented sigma points
  lambda_ = 3 - n_x_;

  // Create sigma points matrix
  MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);
  Xsig.col(0) = x_;
  for (int i = 0; i < n_x_; i++)
  {
    Xsig.col(i + 1)        = x_ + sqrt(lambda_ + n_x_) * P_sqrt_.col(i);
    Xsig.col(i + 1 + n_x_) = x_ - sqrt(lambda_ + n_x_) * P_sqrt_.col(i);
  }

  ///// Augment Sigma Points /////

  // Create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  // Create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  // Create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);

  // Set lambda for augmented sigma points
  lambda_ = 3 - n_aug_;

  // Create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // Create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5, 5) = P_;
  P_aug(5, 5) = std_a_*std_a_;
  P_aug(6, 6) = std_yawdd_*std_yawdd_;

  // Create square root matrix
  MatrixXd M_sqrt_ = P_aug.llt().matrixL();

  // Create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i++)
  {
    Xsig_aug.col(i + 1)          = x_aug + sqrt(lambda_ + n_aug_) * M_sqrt_.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * M_sqrt_.col(i);
  }

  ///// Predict Sigma Points /////

  // Predict sigma points
  for (int i = 0; i < n_sig_; i++)
  {
    // Extract values for readability
    double p_x      = Xsig_aug(0, i);
    double p_y      = Xsig_aug(1, i);
    double v        = Xsig_aug(2, i);
    double yaw      = Xsig_aug(3, i);
    double yawd     = Xsig_aug(4, i);
    double nu_a     = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    // Predicted state values
    double px_p, py_p;

    // Prevent division by zero
    if (fabs(yawd) > 0.001) {
      px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    }
    else {
      px_p = p_x + v * delta_t * cos(yaw);
      py_p = p_y + v * delta_t * sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    // Add noise
    px_p += 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p += 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p += nu_a * delta_t;
    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    // Write predicted sigma point into right column
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }

  ///// Convert Predicted Sigma Points to Mean/Covariance /////

  // Set weights
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < n_sig_; i++) {
    weights_(i) = 0.5 / (lambda_ + n_aug_);
  }

  // Calculate Predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {
    x_ += weights_(i) * Xsig_pred_.col(i);
  }

  // Create predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {
    // State difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // Angle normalization
    x_diff(3) = NormalizeAngle(x_diff(3));

    P_ += weights_(i) * x_diff * x_diff.transpose();
  }

}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  // Create measurement vector
  VectorXd z = meas_package.raw_measurements_;

  // Set measurement dimension
  int n_z = 2;

  // Create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, n_sig_);

  // Transform sigma points into measurement space
  for (int i = 0; i < n_sig_; i++) {
    // Extract values for readibility
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);

    // Measurement model
    Zsig(0, i) = p_x;
    Zsig(1, i) = p_y;
  }

  // Calculate mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // Create measurement covariance matrix
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {
    // Caclulate residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // Add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_laspx_*std_laspx_, 0,
       0, std_laspy_*std_laspy_;
  S = S + R;

  // Create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  ///// UKF Update for Lidar /////

  // Calculate the cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {
    // Calculate residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // Calculate the state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Calculate the kalman gain;
  MatrixXd K = Tc * S.inverse();

  // Calculate residual
  VectorXd z_diff = z - z_pred;

  // Update the state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  // Create measurement vector
  VectorXd z = meas_package.raw_measurements_;

  // Set measurement dimensions
  int n_z = 3;

  // Create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, n_sig_);

  // Transform sigma points into measurement space
  for (int i = 0; i < n_sig_; i++) {

    // Extract values for readibility
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double v   = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);
    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // Calculate the measurement model
    Zsig(0, i) = sqrt(p_x*p_x + p_y*p_y);                     // rho
    Zsig(1, i) = atan2(p_y, p_x);                             // phi
    Zsig(2, i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y); // rho_dot
  }

  // Calculate the mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // Calculate the measurement covariance matrix
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {
    // Calculate residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // Normalize the angle
    z_diff(1) = NormalizeAngle(z_diff(1));

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // Create measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_radr_ * std_radr_, 0, 0,
       0, std_radphi_ * std_radphi_, 0,
       0, 0, std_radrd_ * std_radrd_;
  S = S + R;

  ///// UKF Update for Radar /////

  // Calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {
    // Calculate residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // Normalize angle
    z_diff(1) = NormalizeAngle(z_diff(1));

    // Calculate the state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // Normalize angle
    x_diff(3) = NormalizeAngle(x_diff(3));

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // Calculate residual
  VectorXd z_diff = z - z_pred;

  // Normalize angle
  z_diff(1) = NormalizeAngle(z_diff(1));

  // Update the state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
}

double UKF::NormalizeAngle(double angle) {
  while (angle <= -M_PI) { angle += 2. * M_PI; }
  while (angle >   M_PI) { angle -= 2. * M_PI; }
  return angle;
}
