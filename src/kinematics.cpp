#include "kinematics.h"
#include <math.h>

namespace GMRT {
static bool finitef(float v) { return isfinite(v); }
Mat4 identity4() { Mat4 r{}; for (int i=0;i<4;i++) r.m[i][i]=1.0f; return r; }
Mat4 multiply(const Mat4 &a,const Mat4 &b) { Mat4 r{}; for(int i=0;i<4;i++) for(int j=0;j<4;j++) for(int k=0;k<4;k++) r.m[i][j]+=a.m[i][k]*b.m[k][j]; return r; }
Mat4 translation(float x,float y,float z) { Mat4 r=identity4(); r.m[0][3]=x; r.m[1][3]=y; r.m[2][3]=z; return r; }
Mat4 rotationX(float a) { Mat4 r=identity4(); float c=cosf(a),s=sinf(a); r.m[1][1]=c;r.m[1][2]=-s;r.m[2][1]=s;r.m[2][2]=c;return r; }
Mat4 rotationY(float a) { Mat4 r=identity4(); float c=cosf(a),s=sinf(a); r.m[0][0]=c;r.m[0][2]=s;r.m[2][0]=-s;r.m[2][2]=c;return r; }
Mat4 rotationZ(float a) { Mat4 r=identity4(); float c=cosf(a),s=sinf(a); r.m[0][0]=c;r.m[0][1]=-s;r.m[1][0]=s;r.m[1][1]=c;return r; }
Mat4 rotationAxis(const Vec3 &axis,float a) {
    const float n=sqrtf(axis.x*axis.x+axis.y*axis.y+axis.z*axis.z);
    if(!finitef(n)||n<SINGULARITY_EPSILON) return identity4();
    const float x=axis.x/n,y=axis.y/n,z=axis.z/n,c=cosf(a),s=sinf(a),v=1.0f-c;
    Mat4 r=identity4();
    r.m[0][0]=x*x*v+c;   r.m[0][1]=x*y*v-z*s; r.m[0][2]=x*z*v+y*s;
    r.m[1][0]=y*x*v+z*s; r.m[1][1]=y*y*v+c;   r.m[1][2]=y*z*v-x*s;
    r.m[2][0]=z*x*v-y*s; r.m[2][1]=z*y*v+x*s; r.m[2][2]=z*z*v+c;
    return r;
}
Vec3 externalToInternal(const Vec3 &p) { return {p.x,-p.y,p.z}; }
Vec3 internalToExternal(const Vec3 &p) { return {p.x,-p.y,p.z}; }
bool withinJointLimits(const JointAngles &q,float tol) {
    const float v[5]={q.base,q.shoulder,q.elbow,q.wrist,q.gripper};
    for(int i=0;i<5;i++) if(!finitef(v[i]) || v[i]<JOINT_LIMITS[i].minDeg-tol || v[i]>JOINT_LIMITS[i].maxDeg+tol) return false;
    return true;
}

void forwardKinematics(const GeometryConfig &g,const JointAngles &q,Pose &out) {
    // External +Y is right; internal +Y is left. Therefore positive J1
    // (toward external right) is a negative internal Z rotation.
    const float b=-q.base*kDegToRad, s=q.shoulder*kDegToRad, e=q.elbow*kDegToRad, w=q.wrist*kDegToRad;
    Mat4 T01=rotationZ(b);
    Mat4 T12=multiply(translation(0,0,g.baseHeightCm),multiply(rotationY(-s),translation(g.L1Cm,0,0)));
    Mat4 T23=multiply(rotationY(-e),translation(g.L2Cm,0,0));
    Mat4 T34=rotationAxis({g.wristAxisX,g.wristAxisY,g.wristAxisZ},w); // configurable independent wrist roll axis
    Mat4 T45=translation(g.L3Cm+g.toolOffsetXcm,g.toolOffsetYcm,g.toolOffsetZcm);
    Mat4 T=multiply(multiply(multiply(multiply(T01,T12),T23),T34),T45);
    out.T_internal=T;
    out.positionExternalCm=internalToExternal({T.m[0][3],T.m[1][3],T.m[2][3]});
    out.wristRollDeg=q.wrist;
}

IKStatus inverseKinematics(const GeometryConfig &g,const CartesianTarget &t,bool elbowPositive,JointAngles &sol,float gripper) {
    if(!finitef(t.x)||!finitef(t.y)||!finitef(t.z)||!finitef(t.wristRoll)||!finitef(gripper)) return IKStatus::InvalidInput;
    if(!finitef(g.baseHeightCm)||!finitef(g.L1Cm)||!finitef(g.L2Cm)||!finitef(g.L3Cm)||g.L1Cm<=0||g.L2Cm<=0||g.L3Cm<=0) return IKStatus::InvalidGeometry;
    const float axisNorm=sqrtf(g.wristAxisX*g.wristAxisX+g.wristAxisY*g.wristAxisY+g.wristAxisZ*g.wristAxisZ);
    if(!finitef(axisNorm)||axisNorm<SINGULARITY_EPSILON) return IKStatus::InvalidGeometry;
    if(fabsf(g.wristAxisX/axisNorm-1.0f)>IK_EPSILON || fabsf(g.wristAxisY/axisNorm)>IK_EPSILON || fabsf(g.wristAxisZ/axisNorm)>IK_EPSILON) return IKStatus::UnsupportedWristAxis;
    // A transverse tool offset couples yaw and planar position. FK supports a
    // full XYZ tool offset; this analytical IK intentionally rejects that
    // unmeasured/non-planar case instead of silently returning a wrong solution.
    const float w=t.wristRoll*kDegToRad;
    const float offY=g.toolOffsetYcm*cosf(w)-g.toolOffsetZcm*sinf(w);
    const float offZ=g.toolOffsetYcm*sinf(w)+g.toolOffsetZcm*cosf(w);
    if(fabsf(offY)>IK_EPSILON) return IKStatus::UnsupportedToolOffset;

    Vec3 p=externalToInternal({t.x,t.y,t.z});
    const float rho=hypotf(p.x,p.y);
    if(rho<SINGULARITY_EPSILON) return IKStatus::Singular; // base yaw undefined
    const float base=atan2f(-p.y,p.x)*kRadToDeg;
    // Tool offset in the radial/vertical plane is rotated by total arm pitch.
    // Combine L2 and tool offset as one equivalent vector in the local tool plane.
    const float vx=g.L2Cm+g.L3Cm+g.toolOffsetXcm;
    const float vz=offZ;
    const float L2eff=hypotf(vx,vz);
    if(L2eff<SINGULARITY_EPSILON) return IKStatus::InvalidGeometry;
    const float delta=atan2f(vz,vx);
    const float z=p.z-g.baseHeightCm;
    const float r2=rho*rho+z*z;
    const float D=(r2-g.L1Cm*g.L1Cm-L2eff*L2eff)/(2.0f*g.L1Cm*L2eff);
    if(!finitef(D)) return IKStatus::NumericalError;
    if(D < -1.0f-IK_EPSILON || D > 1.0f+IK_EPSILON) return IKStatus::Unreachable;
    const float Dc=fmaxf(-1.0f,fminf(1.0f,D));
    const float sinElbow=(elbowPositive?1.0f:-1.0f)*sqrtf(fmaxf(0.0f,1.0f-Dc*Dc));
    if(fabsf(sinElbow)<SINGULARITY_EPSILON) return IKStatus::Singular;
    const float beta=atan2f(sinElbow,Dc);
    const float shoulder=atan2f(z,rho)-atan2f(L2eff*sinf(beta),g.L1Cm+L2eff*cosf(beta));
    const float elbow=beta-delta;
    sol={base,shoulder*kRadToDeg,elbow*kRadToDeg,t.wristRoll,gripper};
    if(!finitef(sol.base)||!finitef(sol.shoulder)||!finitef(sol.elbow)||!finitef(sol.wrist)) return IKStatus::NumericalError;
    if(!withinJointLimits(sol)) return IKStatus::JointLimit;
    // Round-trip guard: reject solutions whose FK does not reproduce target.
    Pose check{}; forwardKinematics(g,sol,check);
    const float err=hypotf(hypotf(check.positionExternalCm.x-t.x,check.positionExternalCm.y-t.y),check.positionExternalCm.z-t.z);
    if(!finitef(err)||err>0.02f) return IKStatus::NumericalError;
    return IKStatus::Success;
}
const char *ikStatusText(IKStatus s) {
    switch(s){case IKStatus::Success:return "SUCCESS";case IKStatus::InvalidInput:return "INVALID_INPUT";case IKStatus::InvalidGeometry:return "INVALID_GEOMETRY";case IKStatus::Unreachable:return "UNREACHABLE";case IKStatus::JointLimit:return "JOINT_LIMIT";case IKStatus::Singular:return "SINGULARITY";case IKStatus::UnsupportedToolOffset:return "UNSUPPORTED_TRANSVERSE_TOOL_OFFSET_FOR_ANALYTICAL_IK";default:return "NUMERICAL_ERROR";}
}
}
