//#pragma once

#include "raylib.h"
#include "raymath.h"
#include <Eigen/Dense>
#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include "simulation/sim.hpp"
#include "simulation/simThread.hpp"
#include <memory>
#include "raylibGraphics/raylibGraphics.hpp"
#include <fenv.h>
#include <signal.h>
#include "../src/drone/controllerDevice.hpp"
// Update the path below if the file exists elsewhere, or remove if not needed
// #include "controllerDevice.hpp"


using Eigen::Vector3f;
using Eigen::Matrix3d;

// Simulation thread function

int main(void)
{
    feenableexcept(FE_INVALID | FE_DIVBYZERO | FE_OVERFLOW);
    
    // Luo simulaatio ja thread
    std::unique_ptr<Sim> sim1 = std::make_unique<Sim>();
    // Luo ControllerDevice
    std::unique_ptr<ControllerDevice> controllerDevice1 = std::make_unique<ControllerDevice>("/dev/input/event0");
    controllerDevice1->start();

    sim1->setController(controllerDevice1.get());

    SimThread simThread1(std::move(sim1));
    simThread1.start();

    // Luo RaylibGraphics ja avaa ikkuna
    RaylibGraphics raylibGraphics1(&simThread1);
    raylibGraphics1.initRaylib();
    SetTargetFPS(60);


    while (!WindowShouldClose())
    {
        // Input simulaatiolle
        if (IsKeyPressed(KEY_SPACE)) simThread1.pauseButton();
        if (IsKeyPressed(KEY_R)) simThread1.resetSimulation();

        // Input ja kamera RaylibGraphicsille
        raylibGraphics1.updateCamera();

        // Piirto
        raylibGraphics1.cast();

        // (Voit lisätä sliderin ja muut UI-elementit RaylibGraphicsiin)
    }

    // Sulje thread ja ikkuna
    simThread1.stop();
    raylibGraphics1.closeRaylib();
    return 0;
}