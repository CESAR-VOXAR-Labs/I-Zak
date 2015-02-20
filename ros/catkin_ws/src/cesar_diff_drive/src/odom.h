/* (C) CESAR 2014, BSD license */

#define CONTROLLER_TOPIC "arduino/state"

static volatile int lc, rc;

static bool ready;

static unsigned long lctime, rctime, nowtime;

static pthread_mutex_t mtex;

#include <sys/time.h>
#include <pthread.h>

double get_seconds() {
  struct timeval tp;
  gettimeofday(&tp, NULL);
  return ((double)tp.tv_sec + (double)tp.tv_usec / 1000000.);
}



static void odom_cb(const std_msgs::String::ConstPtr& msg) {
  // we receive a message of type string "left_tick_count timestamp_millis right_tick_count timestamp_millis nowtime_millis"
  // where timestamp is 

  char *tokens[5];
  char *str = (char *)msg->data.c_str();

  int i, j=0;

  tokens[0] = str;

  for (i=1; i<5; i++) {
    while (str[j++] != ' ');
    tokens[i] = str + j;
  }

  // mutex lock
  if (pthread_mutex_trylock(&mtex)) return;

  // parse msg
  lc = atoi(tokens[0]);
  lctime = strtoul(tokens[1], NULL, 10); // millis count
  rc = atoi(tokens[2]);
  rctime = strtoul(tokens[3], NULL, 10); // millis count

  nowtime = strtoul(tokens[4], NULL, 10); // millis count

  // mutex unlock
  pthread_mutex_unlock(&mtex);

  ready = true;
}


class Odom {
 private:

  ros::Subscriber sub;
  unsigned long last_time_l, last_time_r;

  int left_count_last;
  int left_count_curr;
  int right_count_last;
  int right_count_curr;


 public:

  double tdiffl, tdiffr;

  Odom(ros::NodeHandle n) {
    ros::Rate loop_rate(100);

    lc = rc = 0;

    last_time_l = last_time_r = 0;

    left_count_last = left_count_curr = right_count_last = right_count_curr = 0;

    // subscribe to wheel tick counter from arduino
    sub = n.subscribe(CONTROLLER_TOPIC, 1, odom_cb);

    pthread_mutex_init(&mtex, NULL);

    ready = false;
    //while (!ready) loop_rate.sleep();

    get_wheel_counts();

    tdiffl = tdiffr = 1000000000000.;
  }


  void get_wheel_counts() {
    static int lready = 0;
    static int rready = 0;

    double xdiffl, xdiffr;

    pthread_mutex_lock(&mtex);

    // left wheel

    if (lc != left_count_curr) {
      // tick count increased or decreased
      left_count_last = left_count_curr;
      left_count_curr = lc;
      if (lready) tdiffl = (double)(lctime - last_time_l) / (double)(left_count_curr - left_count_last) / 1000. * 1.024;
      last_time_l = lctime;
    }
    else {
      // otherwise the velocity may be slowing
      if (nowtime - lctime > lctime - last_time_l) {
	if (lready) {
	  xdiffl = (double)(nowtime - lctime) / 1000. * 1.024;
	  if (tdiffl < 0.) tdiffl = -xdiffl;
	  else tdiffl = xdiffl;
	}
      }
    }


    // do similar thing for right wheel

    if (rc != right_count_curr) {
      right_count_last = right_count_curr;
      right_count_curr = rc;
      if (rready) tdiffr = (double)(rctime - last_time_r) / (double)(right_count_curr - right_count_last) / 1000. * 1.024;
      last_time_r = rctime;
    }
    else {
      if (nowtime - rctime > rctime - last_time_r) {
	if (rready) {
	  xdiffr = (double)(nowtime - rctime) / 1000. * 1.024;
	  if (tdiffr < 0.) tdiffr = -xdiffr;
	  else tdiffr = xdiffr;
	}
      }
    }

    pthread_mutex_unlock(&mtex);
    
    if (left_count_last != 0 ) lready = 1;
    if (right_count_last != 0 ) rready = 1;

  }


};
