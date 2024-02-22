// TorqueVectoringController.cpp
#include "TorqueVectoringController.h"

TorqueVectoringController::TorqueVectoringController(float maxTorquePerWheel)
    : maxTorquePerWheel(maxTorquePerWheel) {
    wheelTorques.resize(4, 0.0); // Initialize all wheel torques to 0
}

void TorqueVectoringController::updateTorque(float desiredAcceleration, float actualAcceleration) {
    float error = desiredAcceleration - actualAcceleration;
    float kp = 0.1; // Proportional control factor, tunable parameter
    float torqueAdjustment = kp * error;
    
    for (float& torque : wheelTorques) {
        torque += torqueAdjustment;
        torque = std::max(0.0f, std::min(maxTorquePerWheel, torque));
    }
}

std::vector<float> TorqueVectoringController::getWheelTorques() const {
    return wheelTorques;
}
