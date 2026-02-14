#pragma once
#include <Eigen/Dense>
#include <memory>
#include "../drone/hardDrone.hpp"
#include "../drone/controller.hpp"
#include "../drone/controllerDevice.hpp"
#include "iostream"


struct Time{
  float milliSeconds = 0;
  float seconds = 0;
  float minutes = 0;
};

struct DroneWhole{
  std::unique_ptr<HardDrone> hardDrone;
  std::unique_ptr<Controller> controller;
};

struct DroneWholeSnapshot{
  Eigen::Vector3f position;
  Eigen::Vector3f velocity;
  Eigen::Quaternionf orientation;
  std::tuple<Propeller, Propeller, Propeller, Propeller> fourPropellers;

  DroneBodyFrame bodyFrame;
};

struct SimSnapshot {
    std::vector<DroneWholeSnapshot> droneWholeSnapshots;
};


class Sim{
 public:
  Sim() = default;
  void updateSimulation(float dt);

  void pauseOn(){paused = true;};
  void pauseOff(){paused = false;};

  SimSnapshot getSnapshot() const;

  void setController(ControllerDevice* _controllerDevice){ controllerDevice = _controllerDevice; };
   
  void startSimulation();

  void endSimulation();

  void resetSimulation();

  void updateTime(float dt);


 private:
    // Congig
  std::vector<std::unique_ptr<DroneWhole>> droneWholes{};

  bool paused = false;
  bool ranOnce = false;
  ControllerDevice* controllerDevice = nullptr;

  Time time;  
};