#pragma once
#include "hardDrone.hpp"
#include "controllerDevice.hpp"

#define DLQR_MAX_ITERATION 50000
/* number of error tolerance in Riccati solving */
#define DLQR_TOLERANCE 0.01

class Controller{
 public:
  Controller(HardDrone* _hardDrone);
  Controller(HardDrone* _hardDrone, Eigen::VectorXd q, Eigen::Vector4d r);
  std::tuple<float, float, float, float> getPropellerRPS() { return hardDrone->getPropellerRPSController(); };
  void control(float dt, float goalRoll, float goalPitch);
  void setControllerDevice(ControllerDevice* _controllerDevice );
  void controlLQR(float dt);
  void setGoalX(Vector3f goal) { goalCoord = goal; };
  Eigen::Matrix<double, 4, 12> DQLR(  Eigen::Matrix<double, 12, 12> Q, Eigen::Matrix<double, 4, 4> R, Eigen::Matrix<double, 12, 12> Ad, Eigen::Matrix<double, 12, 4> Bd);


 private:
  HardDrone* hardDrone;
  float eIntegral = 0;
  float lastEz = 0;
  bool first = true;
  ControllerDevice* controllerDevice;
  Eigen::Vector3f goalCoord;
  Eigen::Matrix<float, 4, 12> K;
  double Ts = 0.01;
  bool once = false;

  int loops = 0;
};