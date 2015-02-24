
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>

#include <stdlib.h>

// Definition of interrupt names
#include <avr/io.h>
// ISR interrupt service routine
#include <avr/interrupt.h>


#include "Servo.h" // Library

ros::NodeHandle  nh;


#define WHEEL_THRESH 100 // if we don't receive ctrl message in this number millis, start emergency stop

// Servo classes

Servo neckSide;
Servo neckRot;
Servo neckUp;


// gimbal bits
#define GIMBAL_S_DEC (1<<0)
#define GIMBAL_S_INC (1<<1)
#define GIMBAL_R_DEC (1<<2)
#define GIMBAL_R_INC (1<<3)
#define GIMBAL_U_DEC (1<<4)
#define GIMBAL_U_INC (1<<5)
#define GIMBAL_CENTER (1<<6)


// Servo defines
#define NECK_SIDE 0
#define NECK_SIDE_PORT 6
#define NECK_SIDE_MIN 68
#define NECK_SIDE_MID 86
#define NECK_SIDE_MAX 104

#define NECK_ROT 1
#define NECK_ROT_PORT 4
#define NECK_ROT_MIN 68
#define NECK_ROT_MID 97
#define NECK_ROT_MAX 126

#define NECK_UP 2
#define NECK_UP_PORT 5
#define NECK_UP_MIN 74
#define NECK_UP_MID 89
#define NECK_UP_MAX 104



#define wheel_LEFT_CTRL_PIN 9
#define wheel_RIGHT_CTRL_PIN 8

#define wheel_LEFT_PWM_PIN 11
#define wheel_RIGHT_PWM_PIN 10

#define wheel_DUTYcycle 100.

float pwMlH = 35.;
float pwMlL = 20.;

float pwMrH = 35.;
float pwMrL = 20.;

float minval = 15.;

uint64_t tcount;

static volatile float pwMl_current;
static volatile float pwMr_current;

float WHEEL_ACCEL = .2;
float NECK_ACCEL = .25;


// new wheel code

#define wheel_LEFT_SENSE_PIN 3
#define wheel_RIGHT_SENSE_PIN 2

static volatile int lcount, rcount;
static volatile unsigned long ltime, rtime, nowtime, wheelstime;


//

float nS = 50.,nSt=50.;
float nR = 50.,nRt=50.;
float nU = 50.,nUt=50.;


static void neck_pos(char motor, int posf) {

  char pos;
  
  if (posf > 100.) posf = 100.;
  if (posf < 0.) posf = 0.;
  
  pos = (char)posf;
  
  switch (motor) {
    case NECK_SIDE:
      pos = map(pos, 0, 100, NECK_SIDE_MIN, NECK_SIDE_MAX);
      neckSide.write(pos);

     break;

    case NECK_ROT:
      pos = map(pos, 0, 100, NECK_ROT_MIN, NECK_ROT_MAX);
      neckRot.write(pos);

      break;

    case NECK_UP:
      pos = map(pos, 0, 100, NECK_UP_MIN, NECK_UP_MAX);
      neckUp.write(pos);

      break;

   default:
   //  // do something
      break;
  }
}



static void setup_servos() { 
  // Atach the servos 
  neckSide.attach(NECK_SIDE_PORT);
  neckRot.attach(NECK_ROT_PORT);
  neckUp.attach(NECK_UP_PORT);
  
}


std_msgs::String str_msg;
ros::Publisher mstate("arduino/state", &str_msg);

char state_str[128];

static void neck_move() {
  if (nSt > nS) {
     nS += NECK_ACCEL;
     if (nS > nSt) nS = nSt;
  }
  if (nSt < nS) {
     nS -= NECK_ACCEL;
     if (nS < nSt) nS = nSt;
  }
  if (nRt > nR) {
     nR += NECK_ACCEL;
     if (nR > nRt) nR = nRt;
  }
  if (nRt < nR) {
     nR -= NECK_ACCEL;
     if (nR < nRt) nR = nRt;
  }
  if (nUt > nU) {
     nU += NECK_ACCEL;
     if (nU > nUt) nU = nUt;
  }
  if (nUt < nU) {
     nU -= NECK_ACCEL;
     if (nU < nUt) nU = nUt;
  }

  neck_pos(NECK_SIDE, nS);
  neck_pos(NECK_ROT, nR);
  neck_pos(NECK_UP, nU);
  
}





static void robot_move() {
  neck_move();

  //snprintf(state_str, 128, "%d %ld %d %ld %ld %d %d %d %d %d", pwMl_current, ltime, pwMr_current, rtime, nowtime, (int)nS, (int)nR, (int)nU);
  snprintf(state_str, 128, "%d %ld %d %ld %ld %d %d %d %d %d", lcount, ltime, rcount, rtime, nowtime, (int)nS, (int)nR, (int)nU, (int)(pwMl_current*1000.), (int)(pwMr_current*1000.));

  str_msg.data = state_str;
  mstate.publish(&str_msg);    

}


void neckHMove_cb (const std_msgs::UInt16& cmd_msg){

   //neck_pos(NECK_SIDE, cmd_msg.data);
   nS = nSt = cmd_msg.data;

}


void neck_cb( const std_msgs::UInt16& cmd_msg){

  if (cmd_msg.data & GIMBAL_CENTER) {
    nS = nR = nU = nSt = nRt = nUt = 50.;
    neck_move();    
    return;
  }

  if (cmd_msg.data == 0) {
    nSt = nS;
    nRt = nR;
    nUt = nU;
  }
  
  if (cmd_msg.data & GIMBAL_S_DEC) {
      nSt = 0.;
  }
  if (cmd_msg.data & GIMBAL_S_INC) {
      nSt = 100.;
  }
  if (cmd_msg.data & GIMBAL_R_DEC) {
      nRt = 0.;
  }
  if (cmd_msg.data & GIMBAL_R_INC) {
      nRt = 100.;
  }
  if (cmd_msg.data & GIMBAL_U_DEC) {
      nUt = 0.;
  }
  if (cmd_msg.data & GIMBAL_U_INC) {
      nUt = 100.;
  }

  
}




void wheelsPwmOut(unsigned long mill){
  static unsigned long wheelsPwmTimeCount = 0;

  if (mill - wheelsPwmTimeCount >= wheel_DUTYcycle) {
    // start new pwm cycle
    
    wheelsPwmTimeCount = mill;
    if (pwMl_current != 0.)  
      digitalWrite(wheel_LEFT_PWM_PIN, HIGH);
  
    if (pwMr_current != 0.)  
      digitalWrite(wheel_RIGHT_PWM_PIN, HIGH);

  } else {
    // go low if it is time (we passed *_current in the cycle)
    
    if (mill - wheelsPwmTimeCount >= fabs(pwMl_current) * (wheel_DUTYcycle / 100.)) {
      digitalWrite(wheel_LEFT_PWM_PIN, LOW);
    }
    if (mill - wheelsPwmTimeCount >= fabs(pwMr_current) * (wheel_DUTYcycle / 100.)) {
      digitalWrite(wheel_RIGHT_PWM_PIN, LOW);
    } 


  }

  if (pwMl_current <= 0.) {
    digitalWrite(wheel_LEFT_CTRL_PIN, HIGH);
  }
  else {
    digitalWrite(wheel_LEFT_CTRL_PIN, LOW);
  }

  if (pwMr_current <= 0.) {
    digitalWrite(wheel_RIGHT_CTRL_PIN, HIGH);
  }
  else {
    digitalWrite(wheel_RIGHT_CTRL_PIN, LOW);
  }
  


}




void wheels_cb( const std_msgs::String& cmd_msg){  
  // here we simply get a string with 2 effort values
  //  we convert these to pwm values
  
  float leffort, reffort, aleffort, areffort;
    
  int lmin, rmin, lrange, rrange;

  lmin = 0.;
  rmin = 0.;

  lrange = 100. - lmin;
  rrange = 100. - rmin;

  const char *str = (char *)cmd_msg.data;

  // split cmd_msg by " " and use strtof
  leffort = (float)strtod(str, NULL);

  int i;

  for (i=0; i < strlen(str); i++) {
     if (str[i] == ' ') {
        reffort = (float)strtod(str + i + 1, NULL);
        break;
     }
  }
  
  aleffort = leffort > 0. ? leffort : -leffort;
  areffort = reffort > 0. ? reffort : -reffort;
  
  pwMl_current = lmin + (lrange * aleffort / 100.);
  pwMr_current = rmin + (rrange * areffort / 100.);

  if (leffort < 0.) pwMl_current = - pwMl_current;
  if (reffort < 0.) pwMr_current = - pwMr_current;

  wheelstime = nowtime;  
}


void lwheelint(void) {
  nowtime = millis();

  if (nowtime - ltime < 10) return;

  int cstate = digitalRead(wheel_LEFT_SENSE_PIN);
  static int lwstate = !cstate;

  if (lwstate == cstate) return;
  lwstate = cstate;

  if (pwMl_current >= 0.) lcount++;
  else lcount--;
  
  ltime = nowtime;
}


void rwheelint(void) {
  nowtime = millis();

  if (nowtime - rtime < 10) return;

  int cstate = digitalRead(wheel_RIGHT_SENSE_PIN);
  static int rwstate = !cstate;

  if (rwstate == cstate) return;
  rwstate = cstate;

  if (pwMr_current >= 0.) rcount++;
  else rcount--;
  
  rtime = nowtime;
}







void setup_interrupts() {
   // use pin 2
  attachInterrupt(1, lwheelint, CHANGE);

   // use pin 3
  attachInterrupt(0, rwheelint, CHANGE);
}




ros::Subscriber<std_msgs::UInt16> neck("robot/neck", neck_cb);
ros::Subscriber<std_msgs::UInt16> neckHMove("robot/neckHMove", neckHMove_cb);
ros::Subscriber<std_msgs::String> wheels("/wheelcontrol", wheels_cb);




void setup(){
  Serial.begin(115200);
  
  pinMode(wheel_RIGHT_CTRL_PIN, OUTPUT);
  pinMode(wheel_LEFT_CTRL_PIN, OUTPUT);
  pinMode(wheel_RIGHT_PWM_PIN, OUTPUT);
  pinMode(wheel_LEFT_PWM_PIN, OUTPUT);

  pinMode(13, OUTPUT);

  pinMode(wheel_RIGHT_SENSE_PIN, INPUT);
  pinMode(wheel_LEFT_SENSE_PIN, INPUT);
  
  pwMl_current = pwMl_current = 0.;
  pwMr_current = pwMr_current = 0.;

  setup_servos();

  setup_interrupts();

  wheelstime = nowtime = tcount = millis();

  lcount = rcount = 0;

  nh.getHardware()->setBaud(115200);
  nh.initNode();

  nh.advertise(mstate);

  robot_move();

  nh.subscribe(neck);
  nh.subscribe(neckHMove);
  nh.subscribe(wheels);


}


void do_calibration(void) {
    if (rcount < 90) {
        pwMl_current = pwMl_current = 0.;  
        pwMr_current = pwMr_current = 100.;
    }
    else if (rcount < 100) {
        pwMl_current = pwMl_current = 0.;  
        pwMr_current = pwMr_current = 50.;
    }
    else if (rcount < 110) {
        pwMl_current = pwMl_current = 0.;  
        pwMr_current = pwMr_current = 20.;
    }
    else if (rcount < 120) {
        pwMl_current = pwMl_current = 0.;  
        pwMr_current = pwMr_current = 10.;
    }
    else {
        pwMl_current = pwMl_current = 0.;  
        pwMr_current = pwMr_current = 0.;      
    }
}



void do_emergency_stop() {
     pwMl_current *= .99;
     pwMr_current *= .99;

     if (fabs(pwMl_current) < .1) pwMl_current = 0.;
     if (fabs(pwMr_current) < .1) pwMr_current = 0.;
}




void loop(){
  
  nh.spinOnce();

  nowtime = millis();

  // if we didnt receive message from controller, start slowing quickly
  if (nowtime - wheelstime > WHEEL_THRESH) do_emergency_stop();

  wheelsPwmOut(nowtime);

  if (nowtime - tcount >= 10) {
    // update wheel/neck velocity
    robot_move();
    tcount = nowtime;
  }

//#define CALIBRATE

#ifdef CALIBRATE
  do_calibration();
#endif

}
