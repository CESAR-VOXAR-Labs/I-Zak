/* run with:

   $ rosrun rviz rviz &
   $ rosrun nav_rviz navrviz
*/

// grab x and y pos. from /wheelstate and send as a point to rviz

#define PI 3.1415926

#define COR_VALS

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>

#include <cmath>

static int ready = 0;

static float nav_x, nav_y, nav_ang;

static void wsCallback(const std_msgs::String::ConstPtr& msg) {

  char *tokens[3];
  char *str = (char *)msg->data.c_str();

  int i, j=0;

  tokens[0] = str;

  for (i = 1; i < 3; i++) {
    while (str[j++] != ' ');
    tokens[i] = str + j;
  }

#ifdef COR_VALS
  nav_y = strtof(tokens[0], NULL);
  nav_x = strtof(tokens[1], NULL);
  nav_ang = strtof(tokens[2], NULL);

  nav_ang += PI / 2.;
  nav_ang = -nav_ang;
#else
  nav_x = strtof(tokens[0], NULL);
  nav_y = strtof(tokens[1], NULL);
  nav_ang = strtof(tokens[2], NULL);
#endif  

  ready = 1;

}




int main( int argc, char** argv ) {
  ros::init(argc, argv, "navrviz");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Subscriber sub = n.subscribe("wheelstate", 1, wsCallback);

  ros::Rate r(4);

  float onav_x = -1000000., onav_y = -1000000., onav_ang = -1000000.;

  ROS_INFO("navrviz starting");

  visualization_msgs::Marker points;
  points.header.frame_id = "/my_frame";
  points.header.stamp = ros::Time::now();
  points.ns = "navrviz";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;

  points.id = 0;

  points.type = visualization_msgs::Marker::POINTS;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = .1;
  points.scale.y = .1;

  // Points are green
  points.color.r = 0.0f;
  points.color.g = 1.0f;
  points.color.b = 0.0f;
  points.color.a = 1.0;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "/my_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "navrviz2";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = 1;
  marker.scale.y = 1.;
  marker.scale.z = 0.;

  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;


  while (ros::ok())
    {
      if (nav_x != onav_x || nav_y != onav_y || nav_ang != onav_ang) {
	ROS_INFO("navrviz got vals %f %f %f",nav_x,nav_y,nav_ang);
  
	marker.pose.position.x = nav_x;
	marker.pose.position.y = nav_y;
	marker.pose.position.z = 0.0;

	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = sin(nav_ang/2.);
	marker.pose.orientation.w = cos(nav_ang/2.);

	marker.lifetime = ros::Duration();

	// Create the vertices for the point

	geometry_msgs::Point p;
	p.x = nav_x;
	p.y = nav_y;
	p.z = 0.0;
      
	points.points.push_back(p);

	marker_pub.publish(points);
	marker_pub.publish(marker);

	onav_x = nav_x;
	onav_y = nav_y;
	onav_ang = nav_ang;
      }

      r.sleep();
      ros::spinOnce();
    }
}


