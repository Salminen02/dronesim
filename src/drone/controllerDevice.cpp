#include "controllerDevice.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <linux/input.h>

ControllerDevice::ControllerDevice(const std::string& devicePath) {
    fd = open(devicePath.c_str(), O_RDONLY | O_NONBLOCK); // NONBLOCK on tärkeä droneille!
    if (fd == -1) {
        perror("Ohjaimen avaaminen epäonnistui");
    }
    else {
        std::cout << "ohjaimen avaaminen onnistui" << std::endl;
    }
};

ControllerState ControllerDevice::getControllerDeviceState(){
    std::lock_guard<std::mutex> lock(mutex);
    return currentState;
}

float normalize(int value, int min, int max) {
    // Lasketaan keskikohta (esim. 128)
    float center = (max + min) / 2.0f;
    float range = (max - min) / 2.0f;
    
    // Muutetaan arvo välille -1.0 ... 1.0
    float normalized = (value - center) / range;
    
    // Deadzone: jos arvo on hyvin lähellä nollaa, pakotetaan se nollaksi
    if (std::abs(normalized) < 0.05f) return 0.0f;
    
    return normalized;
}

void ControllerDevice::update() {
    struct input_event ev;

    // Lue KAIKKI puskurissa olevat tapahtumat tässä syklissä
    // O_NONBLOCK varmistaa, että read palauttaa -1, kun jono on tyhjä
    while (read(fd, &ev, sizeof(ev)) > 0) {
        // Huom: Ei lukita mutexia TÄSSÄ, koska se on jo lukittu loop()-functiossa
        
        if (ev.type == EV_ABS) {
            switch (ev.code) {
                case ABS_X: 
                    currentState.axis1 = normalize(ev.value, 0, 255);
                    break;
                case ABS_Y: 
                    currentState.axis2 = normalize(ev.value, 0, 255); 
                    break;
                case ABS_Z:  // <-- Kokeile tätä ABS_RY:n sijaan
                    currentState.axis3 = normalize(ev.value, 0, 255); 
                    break;
                case ABS_RZ: // <-- Oikea Y-akseli
                    currentState.axis4 = normalize(ev.value, 0, 255); 
                    break;
            }
        }
        else if (ev.type == EV_KEY) {
            // Voit käsitellä myös napit täällä
            // if (ev.code == BTN_SOUTH) currentState.buttonA = ev.value;
        }
    }
}

void ControllerDevice::loop() {    
    while (running.load()) {
        auto startTime = std::chrono::high_resolution_clock::now();
        
        {
            std::lock_guard<std::mutex> lock(mutex);
            update(); 
            // Tulostus mutexin sisällä
        }
        
        auto endTime = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        auto sleepTime = std::chrono::milliseconds(static_cast<int>(1000.0f / updateFreq)) - elapsed;

        if (sleepTime.count() > 0) {
            std::this_thread::sleep_for(sleepTime);
        }
    }
}

void ControllerDevice::start() {
    if (!running.load()) {
        running.store(true);
        thread = std::thread(&ControllerDevice::loop, this);  // Käynnistä thread
    }
}

ControllerDevice::~ControllerDevice() {
    running.store(false);  // Pysäytä säie
    if (thread.joinable()) {
        thread.join();  // Odota säikeen päättymistä
    }
    if (fd != -1) close(fd);
}
