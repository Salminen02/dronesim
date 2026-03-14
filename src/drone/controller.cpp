#include "controller.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>


Controller::Controller(HardDrone* _hardDrone):
hardDrone(_hardDrone)
{
    float g = 9.81;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(12, 12);
        A(0, 1) = 1.0;  // Matlab: A(1, 2)
        A(2, 3) = 1.0;  // Matlab: A(3, 4)
        A(4, 5) = 1.0;  // Matlab: A(5, 6)
        A(6, 7) = 1.0;  // Matlab: A(7, 8)
        A(8, 9) = 1.0;  // Matlab: A(9, 10)
        A(10, 11) = 1.0; // Matlab: A(11, 12)
        A(1,8) = g;
        A(3,6) = -g;
    
    Eigen::MatrixXd Ad = Eigen::MatrixXd::Identity(A.rows(), A.cols()) + A * Ts; // freq = 100
    
    Eigen::Matrix<double, 12, 4> B = Eigen::Matrix<double, 12, 4>::Zero();
// 2. Asetetaan arvot (Matlab-indeksi - 1)
        B(5, 0)  = 1.0 / hardDrone->getMass();   // Matlab: B(6, 1)  -> Z-kiihtyvyys
        B(7, 1)  = 1.0 / hardDrone->getI();  // Matlab: B(8, 2)  -> Phi-kiihtyvyys (roll)
        B(9, 2)  = 1.0 / hardDrone->getI();  // Matlab: B(10, 3) -> Theta-kiihtyvyys (pitch)
        B(11, 3) = 1.0 / hardDrone->getI();

    Eigen::MatrixXd Bd = B*Ts;
   
    Eigen::VectorXd q_diag(12); 
    q_diag << 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2;
    Eigen::MatrixXd Q = q_diag.asDiagonal();

    // R-matriisi (4x4 yksikkömatriisi kerrottuna 0.8:lla)
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(4, 4) * 0.8;

    Eigen::Matrix<double, 4, 12> K_double = DQLR(Q,R,Ad,Bd);
    K = K_double.cast<float>();
    
}

Controller::Controller(HardDrone *_hardDrone, Eigen::VectorXd q, Eigen::Vector4d r): 
hardDrone(_hardDrone)
{
        float g = 9.81;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(12, 12);
        A(0, 1) = 1.0;  // Matlab: A(1, 2)
        A(2, 3) = 1.0;  // Matlab: A(3, 4)
        A(4, 5) = 1.0;  // Matlab: A(5, 6)
        A(6, 7) = 1.0;  // Matlab: A(7, 8)
        A(8, 9) = 1.0;  // Matlab: A(9, 10)
        A(10, 11) = 1.0; // Matlab: A(11, 12)
        A(1,8) = g;
        A(3,6) = -g;
    
    Eigen::MatrixXd Ad = Eigen::MatrixXd::Identity(A.rows(), A.cols()) + A * Ts; // freq = 100
    
    Eigen::Matrix<double, 12, 4> B = Eigen::Matrix<double, 12, 4>::Zero();
// 2. Asetetaan arvot (Matlab-indeksi - 1)
        B(5, 0)  = 1.0 / hardDrone->getMass();   // Matlab: B(6, 1)  -> Z-kiihtyvyys
        B(7, 1)  = 1.0 / hardDrone->getI();  // Matlab: B(8, 2)  -> Phi-kiihtyvyys (roll)
        B(9, 2)  = 1.0 / hardDrone->getI();  // Matlab: B(10, 3) -> Theta-kiihtyvyys (pitch)
        B(11, 3) = 1.0 / hardDrone->getI();

    Eigen::MatrixXd Bd = B*Ts;
   
    Eigen::VectorXd q_diag(12); 
    Eigen::MatrixXd Q = q.asDiagonal();

    // R-matriisi (4x4 yksikkömatriisi kerrottuna 0.8:lla)
    Eigen::MatrixXd R = r.asDiagonal();

    Eigen::Matrix<double, 4, 12> K_double = DQLR(Q,R,Ad,Bd);
    K = K_double.cast<float>();
    
}


void PrintMatrix(Eigen::MatrixXf mat)
{
    for(int i = 0 ; i < mat.rows() ; i++)
    {
        for(int j = 0 ; j < mat.cols() ; j++)
        {
            std::cout << mat(i, j) << "  ";
        }
        std::cout << std::endl;
    }
}
Eigen::Matrix<double, 4, 12> Controller::DQLR(Eigen::Matrix<double, 12, 12> Q, Eigen::Matrix<double, 4, 4> R, Eigen::Matrix<double, 12, 12> Ad, Eigen::Matrix<double, 12, 4> Bd)
{
    Eigen::MatrixXd P = Q;
    Eigen::MatrixXd P_1;
    Eigen::MatrixXd P_err;
    int iteration_num = 0;
    double err = 10.0 * DLQR_TOLERANCE;
    Eigen::MatrixXd::Index maxRow, maxCol;
    Eigen::Matrix<double, 4, 12> K_result;

    while(err > DLQR_TOLERANCE && iteration_num < DLQR_MAX_ITERATION)
    {
        iteration_num++;
        P_1 = Q + Ad.transpose() * P * Ad - Ad.transpose() * P * Bd * (R + Bd.transpose() * P * Bd).inverse() * Bd.transpose() * P * Ad;
        Eigen::MatrixXd P_err = P_1 - P;
        err = fabs(P_err.maxCoeff(&maxRow, &maxCol));
        P = P_1;
    }

    if(iteration_num < DLQR_MAX_ITERATION)
    {
        K_result = (R + Bd.transpose() * P * Bd).inverse() * Bd.transpose() * P * Ad;
        return K_result;
    }
    else{
        std::cout << "DLQR Solve failed !!!" << std::endl;
        K_result.setZero();
        return K_result;
    }
}


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

Eigen::Vector3f quaternionToEuler(Eigen::Quaternionf q) {
    // Normalize quaternion to prevent NaN errors
    q.normalize();

    // Roll (rotation around X-axis)
    float roll = std::atan2(2.0f * (q.w() * q.x() + q.y() * q.z()), 
                           1.0f - 2.0f * (q.x() * q.x() + q.y() * q.y()));

    // Pitch (rotation around Y-axis)
    float sinp = 2.0f * (q.w() * q.y() - q.z() * q.x());
    float pitch;
    if (std::abs(sinp) >= 1.0f) {
        pitch = std::copysign(1.570796f, sinp); // PI / 2
    } else {
        pitch = std::asin(sinp);
    }

    // Yaw (rotation around Z-axis)
    float yaw = std::atan2(2.0f * (q.w() * q.z() + q.x() * q.y()), 
                          1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z()));

    return Eigen::Vector3f(roll, pitch, yaw);
}

void Controller::control(float dt, float _goalRoll, float _goalPitch)
{
    ControllerState controllerState = controllerDevice->getControllerDeviceState();

    float goalPitch = _goalPitch; // - controllerState.axis4/1.8;
    float goalRoll = _goalRoll; // - controllerState.axis3/1.8;
    float goalYawVel = 0; // 2*controllerState.axis1;

    Vector3f vel = hardDrone->getVelocity();

    float m = hardDrone->getMass();

    Eigen::Quaternionf orientationQ = hardDrone->getOrientationQ(); 
    Vector3f ownZ = orientationQ.inverse() * Vector3f(0,0,1);
    float sinAlpha = Vector3f(0,1,0).dot(ownZ);
    
    float goalYVel = -2*controllerState.axis2;
    float yE = goalYVel - vel.z(); 

    //std::cout << yE << std::endl;
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

    float kpYawVel = 0;
    float kpPitch = -4;
    float kpRoll = 4;

    float kdRoll = 4;  // Damping termi
    float kdPitch = 4; // Damping termi


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

void Controller::controlLQR(float dt)
{
    Eigen::Quaternionf q1 = hardDrone->getOrientationQ();
    Eigen::Quaternionf q_extra(Eigen::AngleAxisf(-M_PI/4.0f, Eigen::Vector3f::UnitZ()));
    Eigen::Quaternionf q = q1 * q_extra; 
    Vector3f eulerAngleZ = quaternionToEuler(q);
    Eigen::Quaternionf q_offset(Eigen::AngleAxisf(eulerAngleZ.z(), Eigen::Vector3f::UnitZ()));

    // Paikka pysyy maailmankoordinaatistossa
    Vector3f pos = q_offset.inverse() * hardDrone->getPosition();

    Eigen::Quaternionf q_lqr = q_offset.inverse() * q;  
    Vector3f eulerAngles = quaternionToEuler(q_lqr);

    Vector3f vel = q_offset.inverse() * hardDrone->getVel();
    Vector3f angVel = q_offset.inverse() * hardDrone->getAngularVelocityWorld();
    Vector3f angVelLoc = hardDrone->getAngularVelocityLocal();

    Vector3f goalCoordLoc = q_offset.inverse() * goalCoord;

    Eigen::Matrix<float, 12, 1> x {
        {pos.x() - goalCoordLoc.x()},
        {vel.x()},
        {pos.y() - goalCoordLoc.y()},
        {vel.y()}, 
        {pos.z() - goalCoordLoc.z()}, 
        {vel.z()},
        {eulerAngles.x()}, 
        {angVel.x()},
        {eulerAngles.y()}, 
        {angVel.y()},
        {0}, 
        {angVelLoc.z() - 0.1f}
    };

    Eigen::Matrix<float, 4, 1> u = - K*x;

    if (!once){
        std::cout << K << std::endl;
        once = true;
    }

    float totalF = u(0,0) + hardDrone->getMass() * 9.81;
    float totalForce = totalF/(std::cos(eulerAngleZ.x()) * std::cos(eulerAngleZ.y()));
   
    Vector3f torqueXYZLQRWorld = Vector3f(u(1,0), -u(2,0), u(3,0));
    Vector3f torqueXYZWorld = q_offset.inverse() * torqueXYZLQRWorld;
    Vector3f torqueXYZLoc = q.inverse() * torqueXYZWorld;

    float torqueX = torqueXYZLoc.x();
    float torqueY = torqueXYZLoc.y();
    float torqueZ = torqueXYZLoc.z();   

    //std::cout << torqueZ << std::endl;
    
    float r = hardDrone->getR();
    float Ct = hardDrone->getCt();
    float Cq = hardDrone->getCq(); // Varmista että tämä löytyy hardDronesta
    float D  = hardDrone->getD();  // Potkurin halkaisija
    float k  = (Cq * D) / Ct;

// Lasketaan yaw-komponentti (torqueZ jaettuna neljälle moottorille ja skaalattuna k:lla)
// Tutkimuksen mukaan: Tz = k*(F2 + F4 - F1 - F3)
    float yaw_adj = torqueZ / (4.0f * k);

// Mikseri (varmista etumerkit tutkimuksen moottorijärjestyksen mukaan)
// F1 ja F3 ovat CW (negatiivinen Tz), F2 ja F4 ovat CCW (positiivinen Tz)
    float f_front = totalForce/4.0f + torqueY/(2.0f*r) - yaw_adj; // Moottori 4 (Paperin F4, pitäisi olla +yaw_adj?)
    float f_back  = totalForce/4.0f - torqueY/(2.0f*r) - yaw_adj; // Moottori 2 (Paperin F2)
    float f_left  = totalForce/4.0f + torqueX/(2.0f*r) + yaw_adj; // Moottori 1 (Paperin F1)
    float f_right = totalForce/4.0f - torqueX/(2.0f*r) + yaw_adj;

    float denumerator = hardDrone->getRho() * hardDrone->getPropellerD4() * hardDrone->getCt();
    float BLRps = sqrt(std::abs(f_left/denumerator));
    float BRRps = sqrt(std::abs(f_back/denumerator));
    float FLRps = sqrt(std::abs(f_front/denumerator));
    float FRRps = sqrt(std::abs(f_right/denumerator));

    auto RPSs = std::make_tuple(BLRps, BRRps, FLRps, FRRps);
    hardDrone->setFourPropellerRPS(RPSs);
}
