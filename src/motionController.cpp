#include "motionController.h"
#include "configuration.h"
namespace GMRT {
MotionController::MotionController(ServoController &s):servo_(s),start_{0,0,0,0,0},target_{0,0,0,0,0},current_{0,0,0,0,0},startMs_(0),durationMs_(0),lastUpdateMs_(0),moving_(false) {}
bool MotionController::start(const JointAngles &target,uint32_t duration) {
    if(!servo_.isArmed()||!servo_.validateJointTargets(target)||duration<MIN_MOVE_TIME_MS||duration>MAX_MOVE_TIME_MS) return false;
    start_=current_=servo_.lastCommanded(); target_=target; startMs_=millis(); durationMs_=duration; lastUpdateMs_=0;
    trajectory_.begin(start_,target_,startMs_,durationMs_); moving_=true; return true;
}
void MotionController::update(uint32_t now) {
    if(!moving_||static_cast<uint32_t>(now-lastUpdateMs_)<MOTION_UPDATE_MS) return;
    lastUpdateMs_=now; JointAngles q{};
    if(!trajectory_.sample(now,q)){moving_=false;return;}
    if(!servo_.writeJointTargets(q)){trajectory_.stop();moving_=false;return;}
    current_=q; if(!trajectory_.active())moving_=false;
}
void MotionController::stop() { trajectory_.stop(); if(moving_)current_=servo_.lastCommanded();target_=current_;moving_=false; }
}
