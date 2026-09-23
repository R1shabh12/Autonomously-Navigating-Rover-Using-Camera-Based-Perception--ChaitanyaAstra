/*
 DART camera-first rover — low-level controller reference.
 Receives v_mps and w_rps over serial JSON, converts them to wheel targets,
 closes the wheel-speed loop using encoders, generates PWM/direction to an H-bridge,
 and returns telemetry. Replace every PIN_* constant for the actual rover.
*/
#include <Arduino.h>

const int PIN_L_PWM=5, PIN_L_DIR=4;
const int PIN_R_PWM=6, PIN_R_DIR=7;
const int PIN_L_ENC_A=2, PIN_L_ENC_B=3;
const int PIN_R_ENC_A=18, PIN_R_ENC_B=19;
const int PIN_US_TRIG=10, PIN_US_ECHO=11;

const float WHEEL_RADIUS_M=0.05f;
const float TRACK_M=0.28f;
const int TICKS_PER_REV=1024;
const int PWM_MAX=255;
const float MAX_WHEEL_RAD_S=10.0f;
float KP=0.8f, KI=0.15f; // Tune experimentally.

volatile long leftTicks=0, rightTicks=0;
float targetLeftRadS=0, targetRightRadS=0;
float leftIntegral=0, rightIntegral=0;
unsigned long lastControlMs=0, lastTelemetryMs=0;
long lastLeftTicks=0, lastRightTicks=0;

void leftEncoderISR(){ bool a=digitalRead(PIN_L_ENC_A), b=digitalRead(PIN_L_ENC_B); leftTicks += (a==b)?1:-1; }
void rightEncoderISR(){ bool a=digitalRead(PIN_R_ENC_A), b=digitalRead(PIN_R_ENC_B); rightTicks += (a==b)?1:-1; }
float clampf(float x,float lo,float hi){ return max(lo,min(hi,x)); }

float readJsonFloat(const String& s,const char* key,float fallback=0){
  String k=String("\"")+key+"\""; int p=s.indexOf(k); if(p<0)return fallback;
  p=s.indexOf(':',p); if(p<0)return fallback;
  String v=s.substring(p+1); int c=v.indexOf(','); int b=v.indexOf('}'); int e=(c>=0)?c:b;
  if(e>=0)v=v.substring(0,e); v.trim(); return v.toFloat();
}

void setMotor(int pwmPin,int dirPin,float cmd){
  cmd=clampf(cmd,-PWM_MAX,PWM_MAX);
  digitalWrite(dirPin,cmd>=0?HIGH:LOW);
  analogWrite(pwmPin,(int)fabs(cmd));
}

float wheelRate(float v,float w,bool right){
  float out=(v+(right?1:-1)*w*TRACK_M/2.0f)/WHEEL_RADIUS_M;
  return clampf(out,-MAX_WHEEL_RAD_S,MAX_WHEEL_RAD_S);
}

float ultrasonicMeters(){
  digitalWrite(PIN_US_TRIG,LOW); delayMicroseconds(2);
  digitalWrite(PIN_US_TRIG,HIGH); delayMicroseconds(10); digitalWrite(PIN_US_TRIG,LOW);
  unsigned long us=pulseIn(PIN_US_ECHO,HIGH,20000UL); if(us==0)return -1;
  return (us*1e-6f*343.0f)/2.0f;
}

void processCommand(const String& line){
  if(line.indexOf("\"type\":\"cmd\"")<0)return;
  float v=readJsonFloat(line,"v_mps",0), w=readJsonFloat(line,"w_rps",0);
  targetLeftRadS=wheelRate(v,w,false); targetRightRadS=wheelRate(v,w,true);
}

void runControl(float dt){
  long l=leftTicks,r=rightTicks;
  long dL=l-lastLeftTicks,dR=r-lastRightTicks;
  lastLeftTicks=l; lastRightTicks=r;
  float leftActual=(2*PI*dL/TICKS_PER_REV)/dt;
  float rightActual=(2*PI*dR/TICKS_PER_REV)/dt;
  float eL=targetLeftRadS-leftActual,eR=targetRightRadS-rightActual;
  leftIntegral=clampf(leftIntegral+eL*dt,-5,5);
  rightIntegral=clampf(rightIntegral+eR*dt,-5,5);
  float leftOut=(KP*eL+KI*leftIntegral)/MAX_WHEEL_RAD_S*PWM_MAX;
  float rightOut=(KP*eR+KI*rightIntegral)/MAX_WHEEL_RAD_S*PWM_MAX;
  if(fabs(targetLeftRadS)<0.05)leftIntegral=0;
  if(fabs(targetRightRadS)<0.05)rightIntegral=0;
  setMotor(PIN_L_PWM,PIN_L_DIR,clampf(leftOut,-PWM_MAX,PWM_MAX));
  setMotor(PIN_R_PWM,PIN_R_DIR,clampf(rightOut,-PWM_MAX,PWM_MAX));
}

void sendTelemetry(){
  float d=ultrasonicMeters();
  Serial.print("{\"type\":\"telemetry\",\"left_ticks\":"); Serial.print(leftTicks);
  Serial.print(",\"right_ticks\":"); Serial.print(rightTicks);
  Serial.print(",\"obstacle_distance_m\":"); if(d<0)Serial.print("null");else Serial.print(d,3);
  Serial.println("}");
}

void setup(){
  Serial.begin(115200);
  pinMode(PIN_L_PWM,OUTPUT); pinMode(PIN_L_DIR,OUTPUT); pinMode(PIN_R_PWM,OUTPUT); pinMode(PIN_R_DIR,OUTPUT);
  pinMode(PIN_L_ENC_A,INPUT_PULLUP); pinMode(PIN_L_ENC_B,INPUT_PULLUP);
  pinMode(PIN_R_ENC_A,INPUT_PULLUP); pinMode(PIN_R_ENC_B,INPUT_PULLUP);
  pinMode(PIN_US_TRIG,OUTPUT); pinMode(PIN_US_ECHO,INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_L_ENC_A),leftEncoderISR,CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_R_ENC_A),rightEncoderISR,CHANGE);
  setMotor(PIN_L_PWM,PIN_L_DIR,0); setMotor(PIN_R_PWM,PIN_R_DIR,0);
  lastControlMs=millis(); lastTelemetryMs=millis();
}

void loop(){
  while(Serial.available()){ String line=Serial.readStringUntil('\n'); line.trim(); if(line.length())processCommand(line); }
  unsigned long now=millis();
  if(now-lastControlMs>=50){ float dt=(now-lastControlMs)/1000.0f; lastControlMs=now; runControl(dt); }
  if(now-lastTelemetryMs>=200){ lastTelemetryMs=now; sendTelemetry(); }
}
