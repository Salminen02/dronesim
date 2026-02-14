#include "controller.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

Controller::Controller(HardDrone* _hardDrone):
hardDrone(_hardDrone)
{}

std::tuple<float, float, float> rollPitchYaw(Eigen::Quaternionf q) {
    // 1. Varmistetaan että kvaternio on normalisoitu (estää NaN-virheet)
    q.normalize();

    // 2. Roll (rotaatio X-akselin ympäri)
    // Lasketaan käyttäen Y- ja Z-akseleita
    float roll = std::atan2(2.0f * (q.w() * q.x() + q.y() * q.z()), 
                           1.0f - 2.0f * (q.x() * q.x() + q.y() * q.y()));

    // 3. Pitch (rotaatio Y-akselin ympäri)
    // Sinp on välillä [-1, 1]. Clampataan se varmuuden vuoksi.
    float sinp = 2.0f * (q.w() * q.y() - q.z() * q.x());
    float pitch;
    if (std::abs(sinp) >= 1.0f) {
        pitch = std::copysign(1.570796f, sinp); // PI / 2
    } else {
        pitch = std::asin(sinp);
    }

    // 4. Yaw (rotaatio Z-akselin ympäri)
    // Lasketaan käyttäen X- ja Y-akseleita
    float yaw = std::atan2(2.0f * (q.w() * q.z() + q.x() * q.y()), 
                          1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z()));

    return std::make_tuple(roll, pitch, yaw);
}

void Controller::control(float dt)
{

    ControllerState controllerState = controllerDevice->getControllerDeviceState();

    float goalPitch = - controllerState.axis4/1.8;
    float goalRoll = - controllerState.axis3/1.8;
    float goalYawVel = 2*controllerState.axis1;

    Vector3f vel = hardDrone->getVelocity();

    float m = hardDrone->getMass();

    Eigen::Quaternionf orientationQ = hardDrone->getOrientationQ(); 
    Vector3f ownZ = orientationQ.inverse() * Vector3f(0,0,1);
    float sinAlpha = Vector3f(0,1,0).dot(ownZ);
    
    float goalYVel = -2*controllerState.axis2;
    float yE = goalYVel - vel.z(); 

    //std::cout << goalYVel << " " << vel.z() << std::endl;
    float baseKp = 2;
    float desiredYacc = baseKp * yE;
    float desiredYF = desiredYacc * m;
    float baseForce;
    baseForce = desiredYF/(1-sinAlpha) + (hardDrone->getMass() * 9.81)/4;

    auto _rollPitchYaw = rollPitchYaw(hardDrone->getOrientationQ());
    Vector3f angVel = hardDrone->getAngularVelocityLocal();

    float eRoll = goalRoll - std::get<0>(_rollPitchYaw);   // roll on index 0
    float ePitch = goalPitch - std::get<1>(_rollPitchYaw); // pitch on index 1
    float eYawVel = goalYawVel - angVel.z();               // yaw on index 2

    float kpYawVel = 1;
    float kpPitch = -4;
    float kpRoll = 4;

    float kdRoll = 2;  // Damping termi
    float kdPitch = 2; // Damping termi


    float dRoll = angVel.x();
    float dPitch = angVel.y();

    float BLForce =   kpYawVel * eYawVel - kpRoll * eRoll + kdRoll * angVel.x() - kpPitch * ePitch - kdPitch * angVel.y() + baseForce;
    float BRForce = - kpYawVel * eYawVel - kpRoll * eRoll + kdRoll * angVel.x() + kpPitch * ePitch + kdPitch * angVel.y() + baseForce;
    float FLForce = - kpYawVel * eYawVel + kpRoll * eRoll - kdRoll * angVel.x() - kpPitch * ePitch - kdPitch * angVel.y() + baseForce;
    float FRForce =   kpYawVel * eYawVel + kpRoll * eRoll - kdRoll * angVel.x() + kpPitch * ePitch + kdPitch * angVel.y() + baseForce;

    float denumerator = hardDrone->getRho() * hardDrone->getPropellerD4() * hardDrone->getCt();
    float BLRps = sqrt(std::abs(BLForce/denumerator));
    float BRRps = sqrt(std::abs(BRForce/denumerator));
    float FLRps = sqrt(std::abs(FLForce/denumerator));
    float FRRps = sqrt(std::abs(FRForce/denumerator));

     //std::cout << BLRps << std::endl;

    auto RPSs = std::make_tuple(BLRps, BRRps, FLRps, FRRps);
    hardDrone->setFourPropellerRPS(RPSs);
}



void Controller::setControllerDevice(ControllerDevice *_controllerDevice)
{
    if (_controllerDevice == nullptr){
        std::cout << "simulation has no controllerDevice";
    }
    else{
    controllerDevice = _controllerDevice;
    }
}
