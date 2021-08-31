#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // Initialization tag
  is_initialized_ = false;

  // Set state dimension
  n_x_ = 5;

  // Set augmented state dimension
  n_aug_ = 7;

  // Set measurement dimension for laser, lasor can measure x and y
  n_lidar_ = 2;

  // Set measurement dimension for radar, radar can measure r, phi, and r_dot
  n_radar_ = 3;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;

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

  // predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // Set Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // set vector for weights
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_.fill(0.5 / (lambda_ + n_aug_));
  weights_(0) = lambda_ / (lambda_ + n_aug_);

  // set measurement noise covariance matrix for laser measurement
  R_laser_ = MatrixXd(n_lidar_, n_lidar_);
  R_laser_.fill(0.0);
  R_laser_(0, 0) = std_laspx_ * std_laspx_;
  R_laser_(1, 1) = std_laspy_ * std_laspy_;

  // set measurement noise covariance matrix for radar measurement
  R_radar_ = MatrixXd(n_radar_, n_radar_);
  R_radar_.fill(0.0);
  R_radar_(0, 0) = std_radr_ * std_radr_;
  R_radar_(1, 1) = std_radphi_ * std_radphi_;
  R_radar_(2, 2) = std_radrd_ * std_radrd_;

  verbose_ = false;

  if (verbose_) {
    std::cout << "Init States" << std::endl;
    std::cout << "R_laser_ = " << std::endl << R_laser_ << std::endl;
    std::cout << "R_radar_ = " << std::endl << R_radar_ << std::endl;
  }
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  if (!is_initialized_) {
    x_.fill(0.0);
    P_ = MatrixXd::Identity(P_.rows(), P_.cols());
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      x_.head(n_lidar_) = meas_package.raw_measurements_.head(n_lidar_);

      P_(0, 0) = std_laspx_ * std_laspx_;
      P_(1, 1) = std_laspy_ * std_laspy_;
    } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      double rho = meas_package.raw_measurements_(0);
      double phi = meas_package.raw_measurements_(1);
      double rho_dot = meas_package.raw_measurements_(2);

      double p_x = rho * cos(phi);
      double p_y = rho * sin(phi);
      double v_x = rho_dot * cos(phi);
      double v_y = rho_dot * sin(phi);
      double vel = sqrt(v_x * v_x + v_y * v_y);

      x_.head(3) << p_x, p_y, vel;

      P_(2, 2) = 100.0;
      P_(3, 3) = 100.0;
    }

    if (verbose_) {
      std::cout << "Init States" << std::endl;
      std::cout << "x_ = " << std::endl << x_ << std::endl;
      std::cout << "P_ = " << std::endl << P_ << std::endl;
    }

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;

    return;
  }

  double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  Prediction(delta_t);

  if (verbose_) {
    std::cout << "Precidtion" << std::endl;
    std::cout << "x_ = " << std::endl << x_ << std::endl;
    std::cout << "P_ = " << std::endl << P_ << std::endl;
  }

  if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
    UpdateLidar(meas_package);
  } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR &&
             use_radar_) {
    UpdateRadar(meas_package);
  }
}

void UKF::Prediction(double delta_t) {
  /**
   * Estimate the object's location.
   */
  // Create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  // Create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  // Create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  // Set augmented mean vector
  x_aug.fill(0.0);
  x_aug.head(n_x_) = x_;

  // Set augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;

  // Create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // Create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; ++i) {
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }

  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    double vel = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yaw_rate = Xsig_aug(4, i);
    double nu_a_k = Xsig_aug(5, i);
    double nu_psi_k = Xsig_aug(6, i);

    // Predict sigma points
    VectorXd nu = VectorXd(n_x_);
    nu(0) = 0.5 * delta_t * delta_t * cos(yaw) * nu_a_k;
    nu(1) = 0.5 * delta_t * delta_t * sin(yaw) * nu_a_k;
    nu(2) = delta_t * nu_a_k;
    nu(3) = 0.5 * delta_t * delta_t * nu_psi_k;
    nu(4) = delta_t * nu_psi_k;

    VectorXd delta = VectorXd(n_x_);
    delta.fill(0.0);
    delta(3) = delta_t * yaw_rate;

    // Avoid division by zero
    if (fabs(yaw_rate) > 0.001) {
      delta(0) = vel / yaw_rate * (sin(yaw + yaw_rate * delta_t) - sin(yaw));
      delta(1) = vel / yaw_rate * (-cos(yaw + yaw_rate * delta_t) + cos(yaw));
    } else {
      delta(0) = vel * cos(yaw) * delta_t;
      delta(1) = vel * sin(yaw) * delta_t;
    }

    Xsig_pred_.col(i) = Xsig_aug.col(i).head(5) + delta + nu;
  }

  // Predict state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    x_ += Xsig_pred_.col(i) * weights_(i);
  }

  // Predict state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    while (x_diff(3) > M_PI)
      x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI)
      x_diff(3) += 2. * M_PI;
    P_ += x_diff * x_diff.transpose() * weights_(i);
  }

  if (verbose_) {
    std::cout << "Prediction" << std::endl;
    std::cout << "x_ = " << std::endl << x_ << std::endl;
    std::cout << "P_ = " << std::endl << P_ << std::endl;
  }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * Use lidar data to update the belief about the object's position.
   *  Modify the state vector, x_, and covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  // Get the laser raw measurement
  VectorXd z = meas_package.raw_measurements_.head(n_lidar_);

  // Create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_lidar_, 2 * n_aug_ + 1);

  // Mean predicted measurement
  VectorXd z_pred = VectorXd(n_lidar_);
  z_pred.fill(0.0);

  // Measurement covariance matrix S
  MatrixXd S = MatrixXd(n_lidar_, n_lidar_);
  S.fill(0.0);

  // Create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_lidar_);
  Tc.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);

    Zsig(0, i) = p_x;
    Zsig(1, i) = p_y;

    // Calculate mean predicted measurement
    z_pred += weights_(i) * Zsig.col(i);
  }

  S += R_laser_;

  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // Calculate innovation covariance matrix S
    S += weights_(i) * z_diff * z_diff.transpose();
  }

  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // State difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // Calculate cross correlation matrix
    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // residual
  VectorXd z_diff = z - z_pred;

  // update state mean and covariance matrix
  x_ += K * z_diff;
  P_ -= K * S * K.transpose();

  double nis = z_diff.transpose() * S.inverse() * z_diff;

  if (verbose_) {
    std::cout << "Update Lidar State" << std::endl;
    std::cout << "x_ = " << std::endl << x_ << std::endl;
    std::cout << "P_ = " << std::endl << P_ << std::endl;
    std::cout << "NIS = " << nis << std::endl;
  }
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * Use radar data to update the belief about the object's position.
   * Modify the state vector, x_, and covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

  // Get the radar raw measurement
  VectorXd z = meas_package.raw_measurements_.head(n_radar_);

  // Create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_radar_, 2 * n_aug_ + 1);

  // Mean predicted measurement
  VectorXd z_pred = VectorXd(n_radar_);
  z_pred.fill(0.0);

  // Measurement covariance matrix S
  MatrixXd S = MatrixXd(n_radar_, n_radar_);
  S.fill(0.0);

  // Create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_radar_);
  Tc.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double vel = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);
    Zsig(1, i) = atan2(p_y, p_x);
    Zsig(2, i) = (p_x * cos(yaw) * vel + p_y * sin(yaw) * vel) / Zsig(0, i);

    // Calculate mean predicted measurement
    z_pred += weights_(i) * Zsig.col(i);
  }

  S += R_radar_;

  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1) > M_PI)
      z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI)
      z_diff(1) += 2. * M_PI;

    // Calculate innovation covariance matrix S
    S += weights_(i) * z_diff * z_diff.transpose();
  }

  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1) > M_PI)
      z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI)
      z_diff(1) += 2. * M_PI;

    // State difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // Angle normalization
    while (x_diff(3) > M_PI)
      x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI)
      x_diff(3) += 2. * M_PI;

    // Calculate cross correlation matrix
    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // residual
  VectorXd z_diff = z - z_pred;

  // angle normalization
  while (z_diff(1) > M_PI)
    z_diff(1) -= 2. * M_PI;
  while (z_diff(1) < -M_PI)
    z_diff(1) += 2. * M_PI;

  // update state mean and covariance matrix
  x_ += K * z_diff;
  P_ -= K * S * K.transpose();

  double nis = z_diff.transpose() * S.inverse() * z_diff;

  if (verbose_) {
    std::cout << "Update Radar State" << std::endl;
    std::cout << "x_ = " << std::endl << x_ << std::endl;
    std::cout << "P_ = " << std::endl << P_ << std::endl;
    std::cout << "NIS = " << nis << std::endl;
  }
}