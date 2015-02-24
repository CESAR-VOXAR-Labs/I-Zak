/* (C) CESAR 2014, BSD license */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_broadcaster.h>

#include <sstream>
#include <unistd.h>

#include "odom.h"

static double hwsep; // (wheel - wheel distance) / 2 
static double wheel_circ; // wheel circumference

static double trv, tlv; // target velocities in m / s
static double left_vel, right_vel; // wheel velocities m / s
static double failsafe; // rcv time of last Twist message

static double el, er; // effort values (-VMAX to +VMAX)

static ros::Publisher wheel_control_pub;

#define MIN_VEL 0.005 // min vel we can register accurately

#define EMAX 100. // max effort (100 %)

#define TWO_PI 6.28318530718

#define TICKS_PER_REV 24 // odometry ticks per wheel revolution

//#define ACC .25  // acceleration, effort / s * s [per 30th second !]  (make parameter); e.g .1 will take 30 seconds to go from 0. to 100.
//#define DEC .35  // deceleration, effort / s * s * eff / 10. [per 30th second !] 

#define ACC .15  // better for followme
#define DEC .5  // deceleration, effort / s * s * eff / 10. [per 30th second !] 

#define EM_STOP .5 // emergency stop if we dont receive a Twist message in X seconds

// wheel compensations (e.g. right has to spin a little faster to keep it going straight)
#define L_COMP .85
#define R_COMP 1.0


#define HWSEP 0.312
#define WHEEL_CIRC 0.356

#define VMIN 0.05
#define VMAX 1.4

static double vmaxf = VMAX;
static double vmaxb = VMAX;


// subscribes to cmd_vel (Twist)

// subscribes to arduino/state (String) [<- arduino]

// publishes wheelctrl (String) [-> arduino]

// publishes wheelstate (String) [-> cesar_robot_state]

// publishes tf [-> tf] (world -> base_link)


static void controlCallback(const geometry_msgs::Twist& msg) {
  // set target values

  double xv = msg.linear.x;
  double angv = msg.angular.z;

  tlv = xv + angv * HWSEP;
  trv = xv - angv * HWSEP;

  // make sure in range
  if (tlv > vmaxf) tlv = vmaxf;
  if (tlv < -vmaxb) tlv = -vmaxb;

  if (fabs(tlv) < VMIN) tlv = 0.;


  if (trv > vmaxf) trv = vmaxf;
  if (trv < -vmaxb) trv = -vmaxb;

  if (fabs(trv) < VMIN) trv = 0.;

  // reset failsafe time
  failsafe = get_seconds();
}



static void send_wheel_ctrl (void) {
  // check current vels against target vels and send ctrl message

  std_msgs::String msgx;

  std::stringstream ss;

  static double el = 0., er = 0.;

  static double lel = -1000., ler = -1000.;
  double accl = ACC, accr = ACC;

#ifdef USE_FAILSAFE
  // if failsafe time exceeded, send emergency stop
  double ctime = get_seconds();
  if (ctime - failsafe > EM_STOP) {
    tlv = trv = 0.;
    el = er = 0.;
    goto send_ctl;
  }
#endif

  if (fabs(tlv) < fabs(left_vel)) accl = DEC*fabs(el)/10.;
  if (fabs(trv) < fabs(right_vel)) accr = DEC*fabs(er)/10.;

  // set real vel maxima if we reach full effort
  // not good, as we can set this by accident when stopping
  /*
  if (el * L_COMP >= EMAX && left_vel < vmaxf) vmaxf = left_vel;
  if (er * R_COMP >= EMAX && right_vel < vmaxf) vmaxf = right_vel;

  if (el * L_COMP <= -EMAX && left_vel > -vmaxb) vmaxb = -left_vel;
  if (er * R_COMP <= -EMAX && right_vel > -vmaxb) vmaxb = -right_vel;*/
  
  if (tlv > left_vel) {
    el += accl;
    if (el > EMAX) el = EMAX;
  }
  else if (tlv < left_vel) {
    el -= accl;
    if (el < -EMAX) el = -EMAX;
  }

  if (trv > right_vel) {
    er += accr;
    if (er > EMAX) er = EMAX;
  }
  else if (trv < right_vel) {
    er -= accr;
    if (er < -EMAX) er = -EMAX;
  }

  // this stops us oscillating around 0. when target is 0.
  if (fabs(tlv) < VMIN && fabs(left_vel) < VMIN) el = 0.;
  if (fabs(trv) < VMIN && fabs(right_vel) < VMIN) er = 0.;


 send_ctl:
  //  if (el != lel || er != ler) {
  if (1) {
    double xel = el * L_COMP;
    double xer = er * R_COMP;

    ss << xel << " " << xer;

    msgx.data = ss.str();

    //    ROS_INFO("wheel control: %s", msgx.data.c_str());

    wheel_control_pub.publish(msgx);

    lel = el;
    ler = er;

  }

}


static void bcast_tf(float x, float y, float theta) {
  // broadcast tf - transformation frame; "odom" -> "base_link"

  // note that "map" is the parent of "odom", but is published elsewhere (TBD...)

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(x, y, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, theta);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
}



int main(int argc, char **argv) {

  ros::init(argc, argv, "diffdrive");
  ros::NodeHandle n;

  ros::Publisher wheel_state_pub = n.advertise<std_msgs::String>("wheelstate", 1000);
  wheel_control_pub = n.advertise<std_msgs::String>("wheelcontrol", 1);

  ros::Rate loop_rate(30);

  // subscribe to control message
  ros::Subscriber sub = n.subscribe("cmd_vel", 1, controlCallback);

  int count = 0;

  float x_pos=0., y_pos=0., ang_pos = 0.;
  float ang_vel, fwd_vel;

  // init odometry, will hang until we receive feedback from wheels
  Odom odom(n);

  tlv = trv = 0.;

  double otime = get_seconds(), ctime, tdiff;

  while (ros::ok())
  {
    odom.get_wheel_counts();

    ctime = get_seconds();
    tdiff = ctime - otime;
    otime = ctime;

    // calculate each velocity 

    left_vel = (WHEEL_CIRC / TICKS_PER_REV) / odom.tdiffl * R_COMP;
    if (fabs(left_vel) < MIN_VEL) left_vel = 0.;
 
    right_vel = (WHEEL_CIRC / TICKS_PER_REV) / odom.tdiffr * L_COMP; 
    if (fabs(right_vel) < MIN_VEL) right_vel = 0.;

    //ROS_INFO("swc: %f %f %f %f %f %f", tlv, trv, left_vel, right_vel, odom.tdiffl, odom.tdiffr);

    fwd_vel = (left_vel + right_vel) / 2.;

    ang_vel = (left_vel - right_vel) / 2. / HWSEP;

    ang_pos += ang_vel * tdiff;

    //clamp to 0 to 2 * PI
    while (ang_pos < 0.) ang_pos += TWO_PI;
    while (ang_pos >= TWO_PI) ang_pos -= TWO_PI;

    x_pos += fwd_vel * tdiff * cos(ang_pos);
    y_pos += fwd_vel * tdiff * sin(ang_pos);

    std_msgs::String msg;

    std::stringstream ss;
    ss << x_pos << " " << y_pos << " " << ang_pos << " " << left_vel << " " << right_vel << " " << ang_vel << " " << count;

    msg.data = ss.str();

    wheel_state_pub.publish(msg);

    send_wheel_ctrl();

    bcast_tf(x_pos, y_pos, ang_pos);

    ros::spinOnce();

    usleep(10000.);

    ++count;

  }

  return 0;
}

