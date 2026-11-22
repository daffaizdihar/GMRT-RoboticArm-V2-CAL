#pragma once
#include <Arduino.h>
#include "servoController.h"
#include "trajectory.h"
namespace GMRT {
class MotionController {
public:
    explicit MotionController(ServoController &servo);
    bool start(const JointAngles &target,uint32_t durationMs);
    void update(uint32_t nowMs);
    void stop();
    bool isMoving() const { return moving_; }
    const JointAngles &currentCommand() const { return current_; }
private:
    ServoController &servo_;
    JointAngles start_,target_,current_;
    uint32_t startMs_,durationMs_,lastUpdateMs_;
    bool moving_;
    JointTrajectory trajectory_;
};
}
