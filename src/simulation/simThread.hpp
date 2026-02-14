#pragma once
#include "sim.hpp"
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

class SimThread {
 public:
    SimThread(std::unique_ptr<Sim> sim);
    ~SimThread() = default;
    
    // Control
    void start();
    void stop();
    bool isRunning() const;
    void pauseButton();  // Toggle pause atomically
    bool isPaused() const { return paused.load(); };
    void resetSimulation();
    void setTimeScale(float scale);
    void runSim();
    
    // Thread-safe data access
    SimSnapshot getSimSnapshot() const;

    // loop
    void threadLoop();
    
 private:
    std::unique_ptr<Sim> sim;
    std::thread thread;
    std::atomic<bool> running;
    mutable std::mutex mutex;  // Suojaa sim:iä
    std::atomic<bool> paused{true};  // Thread-safe pause flag
    float timeScale = 1.0f;
    float updateFreq = 100;
};

