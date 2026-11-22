#pragma once
#include <math.h>
#include "configuration.h"
namespace GMRT {
inline bool jointToServoCommand(float jointDeg,const ServoCalibration &cal,float &servoDeg) {
    if(!isfinite(jointDeg)||(cal.direction!=1&&cal.direction!=-1)) return false;
    servoDeg=cal.offsetDeg+static_cast<float>(cal.direction)*jointDeg;
    return isfinite(servoDeg)&&servoDeg>=cal.minCommandDeg&&servoDeg<=cal.maxCommandDeg;
}
}
