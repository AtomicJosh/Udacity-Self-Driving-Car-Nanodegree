#include "tools.h"
#include<iostream>

using Eigen::VectorXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // Verify estimation vector has data
  if(estimations.size() == 0) {
    std::cout << "No estimations." << std::endl;
    return rmse;
  }

  // The estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size()) {
    std::cout << "Estimations and ground truth data don't have the same dimensions." << std::endl;
    return rmse;
  }

  // Add up the squared residuals
  for(unsigned int i = 0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];

    // Coefficient-wise multiplication
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  // Calculate the RMSE mean
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();
return rmse;
}
