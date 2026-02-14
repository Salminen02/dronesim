#pragma once

#include <string>  // <-- Lisää tämä
#include <thread>
#include <mutex>
#include <atomic>

struct ControllerState{
    float axis1;
    float axis2;
    float axis3;
    float axis4;
};

class ControllerDevice {
public:
    ControllerDevice(const std::string& devicePath);
    ~ControllerDevice();

    // Palauttaa uusimman tilan kopiona (thread-safe)
    ControllerState getControllerDeviceState();
    void update();
    void loop();
    void start();


private:
    void run(); // Metodi, joka pyörii omassa säikeessään
    
    int fd;
    std::thread thread;
    std::atomic<bool> running{false}; // <-- Alusta false:ksi
    
    ControllerState currentState; // Tänne tallennetaan arvot
    std::mutex mutex; // Estää lukemisen ja kirjoittamisen samanaikaisesti
    float timeScale = 1.0f;
    float updateFreq = 100;
};
