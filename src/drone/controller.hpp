#pragma once
#include "hardDrone.hpp"
#include "controllerDevice.hpp"


class Controller{
 public:
  Controller(HardDrone* _hardDrone);
  std::tuple<float, float, float, float> getPropellerRPS() { return hardDrone->getPropellerRPSController(); };
  void control(float dt);
  void setControllerDevice(ControllerDevice* _controllerDevice );

 private:
  HardDrone* hardDrone;
  float eIntegral = 0;
  float lastEz = 0;
  bool first = true;
  ControllerDevice* controllerDevice;
};