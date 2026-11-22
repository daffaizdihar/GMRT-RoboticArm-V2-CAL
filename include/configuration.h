#pragma once
#include <stdint.h>

namespace GMRT {
// Prefix project constants to avoid collisions with macros defined by Arduino.h.
constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kRadToDeg = 180.0f / kPi;
constexpr uint32_t SERIAL_BAUD = 115200;

// All lengths are centimetres. External coordinates: +X forward, +Y right, +Z up.
// Internal HTM frame is right-handed: +X forward, +Y left, +Z up.
struct GeometryConfig {
    float baseHeightCm = 8.0f;
    float L1Cm = 13.5f;
    float L2Cm = 7.0f;
    float L3Cm = 7.5f;
    // Tool vector expressed in wrist frame AFTER the J4 roll transform.
    // Default assumes the contact point lies on the wrist longitudinal axis.
    // Do not add an unmeasured transverse offset.
    float toolOffsetXcm = 0.0f; // additional axial offset beyond nominal L3
    float toolOffsetYcm = 0.0f; // transverse offset; must be measured
    float toolOffsetZcm = 0.0f; // vertical offset; must be measured
    // J4 axis in the wrist frame. +X is a provisional longitudinal-axis assumption.
    float wristAxisX = 1.0f;
    float wristAxisY = 0.0f;
    float wristAxisZ = 0.0f;
};

struct JointLimit { float minDeg; float maxDeg; };
struct ServoCalibration {
    int gpio;
    float offsetDeg;       // servo command = offset + direction * joint angle
    int8_t direction;      // +1 or -1
    float minCommandDeg;   // SG90 command angle, not joint angle
    float maxCommandDeg;
    int minPulseUs;
    int maxPulseUs;
    float initialJointDeg;
};

// GPIO mapping confirmed by project owner.
constexpr int SERVO_COUNT = 5;
constexpr int SERVO_GPIO[SERVO_COUNT] = {13, 14, 27, 26, 25};

// Joint convention: J1 positive turns toward external +Y (right); J2/J3 positive
// raise the arm; J4 positive is right-hand roll about the tool's local +X axis;
// J5 increases from gripper-open toward gripper-closed.
// Limits and calibration values below are PROVISIONAL, not measured safe limits.
constexpr JointLimit JOINT_LIMITS[SERVO_COUNT] = {
    {-90.0f, 90.0f}, {-80.0f, 80.0f}, {-100.0f, 100.0f}, {-90.0f, 90.0f}, {0.0f, 100.0f}
};
constexpr ServoCalibration SERVO_CALIBRATION[SERVO_COUNT] = {
    {13, 90.0f, +1, 0.0f, 180.0f, 1000, 2000, 0.0f},
    {14, 90.0f, +1, 0.0f, 180.0f, 1000, 2000, 0.0f},
    {27, 90.0f, +1, 0.0f, 180.0f, 1000, 2000, 0.0f},
    {26, 90.0f, +1, 0.0f, 180.0f, 1000, 2000, 0.0f},
    {25, 10.0f, +1, 0.0f, 180.0f, 1000, 2000, 0.0f}
};
// MUST remain false until each servo's direction, offset, limits, pulse range,
// initial alignment and mechanical clearances have been measured and updated.
constexpr bool CALIBRATION_VERIFIED = false;
constexpr uint16_t DEFAULT_MOVE_TIME_MS = 1500;
constexpr uint16_t MIN_MOVE_TIME_MS = 100;
constexpr uint16_t MAX_MOVE_TIME_MS = 30000;
constexpr uint16_t MOTION_UPDATE_MS = 20;
constexpr float IK_EPSILON = 1.0e-4f;
constexpr float SINGULARITY_EPSILON = 1.0e-5f;
}
