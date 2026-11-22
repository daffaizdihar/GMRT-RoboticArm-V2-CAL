#pragma once
#include "configuration.h"

namespace GMRT {
struct Vec3 { float x, y, z; };
struct Mat4 { float m[4][4]; };
struct JointAngles { float base, shoulder, elbow, wrist, gripper; };
struct CartesianTarget { float x, y, z, wristRoll; }; // XYZ uses external convention.
struct Pose { Vec3 positionExternalCm; Mat4 T_internal; float wristRollDeg; };
enum class IKStatus : uint8_t { Success, InvalidInput, InvalidGeometry, Unreachable, JointLimit, Singular, UnsupportedToolOffset, NumericalError, UnsupportedWristAxis };

Mat4 identity4();
Mat4 multiply(const Mat4 &a, const Mat4 &b);
Mat4 translation(float x, float y, float z);
Mat4 rotationX(float radians);
Mat4 rotationY(float radians);
Mat4 rotationZ(float radians);
Mat4 rotationAxis(const Vec3 &axis, float radians);
Vec3 externalToInternal(const Vec3 &p);
Vec3 internalToExternal(const Vec3 &p);
bool withinJointLimits(const JointAngles &q, float toleranceDeg = 0.0f);
void forwardKinematics(const GeometryConfig &g, const JointAngles &q, Pose &out);
IKStatus inverseKinematics(const GeometryConfig &g, const CartesianTarget &target,
                           bool elbowPositive, JointAngles &solution,
                           float currentGripperDeg = 0.0f);
const char *ikStatusText(IKStatus status);
}
