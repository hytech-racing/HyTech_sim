// TorqueVectoringController.h
#ifndef TORQUE_VECTORING_CONTROLLER_H
#define TORQUE_VECTORING_CONTROLLER_H

#include <vector>

class TorqueVectoringController {
public:
    TorqueVectoringController(float maxTorquePerWheel);

    void updateTorque(float desiredAcceleration, float actualAcceleration);

    std::vector<float> getWheelTorques() const;

private:
    std::vector<float> wheelTorques;
    float maxTorquePerWheel;
};

#endif // TORQUE_VECTORING_CONTROLLER_H