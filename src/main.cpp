#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include "configuration.h"
#include "kinematics.h"
#include "servoController.h"
#include "motionController.h"
using namespace GMRT;

static GeometryConfig geometry;
static ServoController servos;
static MotionController motion(servos);
static char lineBuffer[160]; static size_t lineLen=0; static bool overflowed=false;
static uint32_t moveTimeMs=DEFAULT_MOVE_TIME_MS;

static void help(){
 Serial.println(F("\nGMRT 5-DOF ARM | Serial 115200, newline terminated"));
 Serial.println(F("H | HELP | STATUS | ARM | DISARM | STOP"));
 Serial.println(F("CAL,<joint 1..5>,<servoDeg>  single-servo calibration; +/-3 deg from configured baseline"));
 Serial.println(F("CALSTOP                   detach calibration servo"));
 Serial.println(F("TIME,<ms>                 movement duration (100..30000)"));
 Serial.println(F("J,<base>,<shoulder>,<elbow>,<roll>,<gripper>"));
 Serial.println(F("IK,<x>,<y>,<z>,<roll>,<gripper>  auto branch"));
 Serial.println(F("IKU,<x>,<y>,<z>,<roll>,<gripper> positive elbow branch"));
 Serial.println(F("IKD,<x>,<y>,<z>,<roll>,<gripper> negative elbow branch"));
 Serial.println(F("XYZ: cm, external frame (+X front,+Y right,+Z up); angles: degrees"));
 Serial.println(F("J4 is roll, NOT tool pitch. J5: 0 open -> 100 closed (provisional mapping)."));
 Serial.println(F("ARM is rejected until CALIBRATION_VERIFIED=true and calibration is measured."));
}
static void status(){
 Serial.print(F("armed="));Serial.print(servos.isArmed());Serial.print(F(" moving="));Serial.print(motion.isMoving());
 Serial.print(F(" calibration_verified="));Serial.print(CALIBRATION_VERIFIED);Serial.print(F(" calibration_active="));Serial.println(servos.isCalibrationActive());
 Serial.print(F("last servo command [deg] = "));const JointAngles&q=servos.lastCommanded();
 Serial.printf("%.2f,%.2f,%.2f,%.2f,%.2f\n",q.base,q.shoulder,q.elbow,q.wrist,q.gripper);
 Serial.println(F("Position estimate is command-based; SG90 has no position feedback."));
}
static bool parseFloats(char *s,float *v,int n){
 char *p=s; for(int i=0;i<n;i++){while(isspace((unsigned char)*p))p++; if(!*p)return false; char *end=nullptr; v[i]=strtof(p,&end); if(end==p||!isfinite(v[i]))return false; p=end;while(isspace((unsigned char)*p))p++; if(i<n-1){if(*p!=',')return false;p++;}else if(*p!='\0')return false;} return true;
}
static void reportIK(IKStatus s){Serial.print(F("IK: "));Serial.println(ikStatusText(s));}
static void queueMove(const JointAngles&q){
 if(!servos.isArmed()){Serial.println(F("ERR: arm not initialized. Calibrate, set CALIBRATION_VERIFIED, then ARM."));return;}
 if(!withinJointLimits(q)){Serial.println(F("ERR: joint limit violation."));return;}
 if(!motion.start(q,moveTimeMs)){Serial.println(F("ERR: motion rejected (duration, state, or target)."));return;}
 Serial.println(F("OK: motion accepted."));
}
static void handle(char *line){
 while(isspace((unsigned char)*line)) line++;
 size_t n=strlen(line);
 while(n&&isspace((unsigned char)line[n-1])) line[--n]='\0';
 if(!n) return;
 for(size_t i=0;i<n;i++)line[i]=toupper((unsigned char)line[i]);
 if(!strcmp(line,"H")||!strcmp(line,"HELP")){help();return;}
 if(!strcmp(line,"STATUS")){status();return;}
 if(!strcmp(line,"STOP")){motion.stop();servos.stopCalibration();Serial.println(F("OK: motion stopped at last commanded position (open-loop estimate)."));return;}
 if(!strcmp(line,"DISARM")){motion.stop();servos.disarm();Serial.println(F("Servos detached."));return;}
 if(!strcmp(line,"ARM")){
  if(!CALIBRATION_VERIFIED){Serial.println(F("REFUSED: calibration is provisional. Verify each servo calibration in configuration.h first."));return;}
  JointAngles initial{SERVO_CALIBRATION[0].initialJointDeg,SERVO_CALIBRATION[1].initialJointDeg,SERVO_CALIBRATION[2].initialJointDeg,SERVO_CALIBRATION[3].initialJointDeg,SERVO_CALIBRATION[4].initialJointDeg};
  if(!servos.arm(initial)){Serial.println(F("ERR: attach/initialization failed. Check calibration and power."));return;}
Serial.println(F("ARMED at configured initial command. This is not measured physical position."));return;
 }
 char *comma=strchr(line,','); if(!comma){Serial.println(F("ERR: unknown command or missing parameters. Send H."));return;} *comma='\0';char *cmd=line;char *args=comma+1;
 if(!strcmp(cmd,"CAL")){
  float v[2];
  if(!parseFloats(args,v,2) || v[0] < 1 || v[0] > SERVO_COUNT || floorf(v[0]) != v[0]) {
   Serial.println(F("ERR: CAL format is CAL,<joint 1..5>,<servoDeg>."));
   return;
  }
  if(!servos.calibrateOne(static_cast<uint8_t>(v[0]),v[1])) {
   Serial.println(F("ERR: calibration rejected. Check DISARMED state, joint index, attach, and +/-3 deg baseline window."));
   return;
  }
  Serial.printf("OK: J%.0f calibration raw servo angle=%.1f deg; only this servo attached.\\n",v[0],v[1]);
  return;
 }
 if(!strcmp(cmd,"TIME")){float v;if(!parseFloats(args,&v,1)||v<MIN_MOVE_TIME_MS||v>MAX_MOVE_TIME_MS){Serial.println(F("ERR: TIME range 100..30000 ms."));return;}moveTimeMs=(uint32_t)v;Serial.printf("OK: duration=%lu ms\n",(unsigned long)moveTimeMs);return;}
 if(!strcmp(cmd,"J")){float v[5];if(!parseFloats(args,v,5)){Serial.println(F("ERR: J requires five numeric joint values."));return;}JointAngles q{v[0],v[1],v[2],v[3],v[4]};queueMove(q);return;}
 bool ik=!strcmp(cmd,"IK"), iku=!strcmp(cmd,"IKU"), ikd=!strcmp(cmd,"IKD");
 if(ik||iku||ikd){float v[5];if(!parseFloats(args,v,5)){Serial.println(F("ERR: IK requires x,y,z,roll,gripper."));return;}CartesianTarget t{v[0],v[1],v[2],v[3]};JointAngles a{},b{};IKStatus sa=inverseKinematics(geometry,t,true,a,v[4]);IKStatus sb=inverseKinematics(geometry,t,false,b,v[4]);
  if(iku){reportIK(sa);if(sa==IKStatus::Success)queueMove(a);return;}if(ikd){reportIK(sb);if(sb==IKStatus::Success)queueMove(b);return;}
  if(sa!=IKStatus::Success&&sb!=IKStatus::Success){Serial.print(F("IK positive: "));Serial.print(ikStatusText(sa));Serial.print(F(" | negative: "));Serial.println(ikStatusText(sb));return;}
  JointAngles chosen;if(sa==IKStatus::Success&&sb==IKStatus::Success){const JointAngles&c=servos.lastCommanded();float ca=fabsf(a.base-c.base)+fabsf(a.shoulder-c.shoulder)+fabsf(a.elbow-c.elbow);float cb=fabsf(b.base-c.base)+fabsf(b.shoulder-c.shoulder)+fabsf(b.elbow-c.elbow);chosen=ca<=cb?a:b;}else chosen=sa==IKStatus::Success?a:b;
  Serial.println(F("IK solution accepted; nearest valid elbow branch selected."));queueMove(chosen);return;
 }
 Serial.println(F("ERR: unsupported command. Send H."));
}
void setup(){Serial.begin(SERIAL_BAUD);servos.begin();Serial.println(F("\nGMRT Robotic Arm firmware booted."));Serial.println(F("SAFE BOOT: no servo attach and no motion command issued."));help();status();}
void loop(){while(Serial.available()){char c=(char)Serial.read();if(c=='\r')continue;if(c=='\n'){if(overflowed)Serial.println(F("ERR: input line too long."));else{lineBuffer[lineLen]='\0';handle(lineBuffer);}lineLen=0;overflowed=false;}else if(!overflowed){if(lineLen<sizeof(lineBuffer)-1)lineBuffer[lineLen++]=c;else overflowed=true;}}motion.update(millis());}
