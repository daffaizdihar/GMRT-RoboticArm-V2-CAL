#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>
#include "kinematics.h"
#include "servoMapping.h"
namespace GMRT {
class ServoController {
public:
    ServoController();
    bool begin();                 // attaches only after explicit arm() call
    bool arm(const JointAngles &initial);
    void disarm();
    // Calibration mode: attaches exactly one servo. servoAngle is raw Servo.write() angle.
    bool calibrateOne(uint8_t jointIndex1Based, float servoAngle);
    void stopCalibration();
    bool isCalibrationActive() const { return calibrationActive_; }
    bool isArmed() const { return armed_; }
    bool validateJointTargets(const JointAngles &q) const;
    bool setJointTargets(const JointAngles &q);
    bool writeJointTargets(const JointAngles &q);
    const JointAngles &lastCommanded() const { return commanded_; }
private:
    Servo servos_[SERVO_COUNT];
    bool attached_[SERVO_COUNT];
    bool armed_;
    bool calibrationActive_;
    int8_t calibrationIndex_;
    JointAngles commanded_;
    bool convert(int index,float jointDeg,float &servoDeg) const;
};
}
