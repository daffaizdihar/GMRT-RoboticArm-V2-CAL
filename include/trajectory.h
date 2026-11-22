#pragma once
#include "kinematics.h"

namespace GMRT {
// Platform-independent synchronized cubic smoothstep trajectory.
class JointTrajectory {
public:
    JointTrajectory() : startMs_(0), durationMs_(0), active_(false) {}
    static float smoothStep(float t) {
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
        return t*t*(3.0f-2.0f*t);
    }
    void begin(const JointAngles &from, const JointAngles &to, uint32_t now, uint32_t duration) {
        from_=from;to_=to;startMs_=now;durationMs_=duration;active_=duration>0;
    }
    // Returns true if a sample was produced. active() indicates whether more
    // samples are required after this one.
    bool sample(uint32_t now, JointAngles &out) {
        if(!active_||durationMs_==0)return false;
        const float t=static_cast<float>(static_cast<uint32_t>(now-startMs_))/static_cast<float>(durationMs_);
        const float u=smoothStep(t);
        const float a[5]={from_.base,from_.shoulder,from_.elbow,from_.wrist,from_.gripper};
        const float b[5]={to_.base,to_.shoulder,to_.elbow,to_.wrist,to_.gripper};
        float v[5];for(int i=0;i<5;i++)v[i]=a[i]+(b[i]-a[i])*u;
        out={v[0],v[1],v[2],v[3],v[4]};
        if(t>=1.0f)active_=false;
        return true;
    }
    void stop() { active_=false; }
    bool active() const { return active_; }
private:
    JointAngles from_{},to_{};
    uint32_t startMs_,durationMs_;
    bool active_;
};
}
