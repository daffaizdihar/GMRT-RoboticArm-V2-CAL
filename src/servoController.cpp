#include "servoController.h"
#include <math.h>
namespace GMRT {
ServoController::ServoController():attached_{false,false,false,false,false},armed_(false),
    calibrationActive_(false),calibrationIndex_(-1),commanded_{0,0,0,0,0} {}
bool ServoController::begin() { // Deliberately no attach/write at boot.
    armed_=false; calibrationActive_=false; calibrationIndex_=-1; return true;
}
bool ServoController::convert(int i,float joint,float &servo) const {
    if(i<0||i>=SERVO_COUNT||!isfinite(joint)) return false;
    return jointToServoCommand(joint,SERVO_CALIBRATION[i],servo);
}
bool ServoController::arm(const JointAngles &initial) {
    if(calibrationActive_ || !CALIBRATION_VERIFIED || !withinJointLimits(initial)) return false;
    float cmds[SERVO_COUNT]; const float q[5]={initial.base,initial.shoulder,initial.elbow,initial.wrist,initial.gripper};
    for(int i=0;i<SERVO_COUNT;i++) if(!convert(i,q[i],cmds[i])) return false;
    for(int i=0;i<SERVO_COUNT;i++) {
        servos_[i].setPeriodHertz(50);
        int ch=servos_[i].attach(SERVO_GPIO[i],SERVO_CALIBRATION[i].minPulseUs,SERVO_CALIBRATION[i].maxPulseUs);
        if(ch<0||!servos_[i].attached()) { disarm(); return false; }
        attached_[i]=true;
        // Apply the calibrated initial command immediately after this channel attaches.
        servos_[i].write(static_cast<int>(lroundf(cmds[i])));
    }
    commanded_=initial; armed_=true; return true;
}
void ServoController::stopCalibration() {
    if(calibrationIndex_ >= 0 && calibrationIndex_ < SERVO_COUNT) {
        servos_[calibrationIndex_].detach();
        attached_[calibrationIndex_] = false;
    }
    calibrationIndex_ = -1;
    calibrationActive_ = false;
}
void ServoController::disarm() {
    stopCalibration();
    for(int i=0;i<SERVO_COUNT;i++) {
        if(attached_[i]) servos_[i].detach();
        attached_[i]=false;
    }
    armed_=false;
}
bool ServoController::calibrateOne(uint8_t jointIndex1Based, float servoAngle) {
    if(armed_ || !isfinite(servoAngle) || jointIndex1Based < 1 || jointIndex1Based > SERVO_COUNT) return false;
    const int i = static_cast<int>(jointIndex1Based) - 1;

    // Narrow commissioning window around configured initial raw servo command.
    const ServoCalibration &cal = SERVO_CALIBRATION[i];
    const float baseline = cal.offsetDeg + static_cast<float>(cal.direction) * cal.initialJointDeg;
    constexpr float CAL_WINDOW_DEG = 3.0f;
    if(!isfinite(baseline) || servoAngle < cal.minCommandDeg || servoAngle > cal.maxCommandDeg ||
       fabsf(servoAngle - baseline) > CAL_WINDOW_DEG) return false;

    // Never leave more than one calibration servo attached.
    if(calibrationIndex_ != i) stopCalibration();
    if(!attached_[i]) {
        servos_[i].setPeriodHertz(50);
        const int channel = servos_[i].attach(SERVO_GPIO[i], cal.minPulseUs, cal.maxPulseUs);
        if(channel < 0 || !servos_[i].attached()) {
            servos_[i].detach();
            attached_[i] = false;
            calibrationIndex_ = -1;
            calibrationActive_ = false;
            return false;
        }
        attached_[i] = true;
        calibrationIndex_ = static_cast<int8_t>(i);
    }
    servos_[i].write(static_cast<int>(lroundf(servoAngle)));
    calibrationActive_ = true;
    return true;
}
bool ServoController::validateJointTargets(const JointAngles &q) const {
    if(!withinJointLimits(q)) return false;
    const float v[5]={q.base,q.shoulder,q.elbow,q.wrist,q.gripper}; float cmds[5];
    for(int i=0;i<SERVO_COUNT;i++) if(!convert(i,v[i],cmds[i])) return false;
    return true;
}
bool ServoController::setJointTargets(const JointAngles &q) { return writeJointTargets(q); }
bool ServoController::writeJointTargets(const JointAngles &q) {
    if(!armed_||!validateJointTargets(q)) return false;
    const float v[5]={q.base,q.shoulder,q.elbow,q.wrist,q.gripper}; float cmds[5];
    for(int i=0;i<5;i++) if(!convert(i,v[i],cmds[i])||!attached_[i]) return false;
    for(int i=0;i<5;i++) servos_[i].write(static_cast<int>(lroundf(cmds[i])));
    commanded_=q; return true;
}
}
