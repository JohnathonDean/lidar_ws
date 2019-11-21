#include <cmath>
#include <cstdlib>
#include <ctime>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>

#include "straight_line_fitting/ransac.h"
#include "straight_line_fitting/ransac_fitline.h"
#include <straight_line_fitting/charactor.h>

using namespace std;

// RANSAC parameter
#define numForEstimate 8
#define successProbability (0.999f)
#define maxOutliersPercentage (0.6f)

static inline void show_ransac_line(float a, float b) {
  static ros::NodeHandle _rviz_line_nh;
  static ros::Publisher rviz_line_pub;
  static bool needInitial = true;
  if (needInitial) {
    needInitial = false;
    rviz_line_pub = _rviz_line_nh.advertise<visualization_msgs::Marker>(
        "visualization_marker", 10);
  }
  visualization_msgs::Marker line_list;
  float f = 0.0;
  line_list.header.frame_id = "scan_frame";
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "lines";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 0.1;
  line_list.id = 2;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.01;
  line_list.color.r = 1.0;
  line_list.color.a = 1.0;
  geometry_msgs::Point p;
  p.x = 0;
  p.y = b;
  p.z = 0;
  line_list.points.push_back(p);
  p.x = 1;
  p.y = b + a;
  p.z = 0;
  line_list.points.push_back(p);
  rviz_line_pub.publish(line_list);
}

void get_position_Callback(const sensor_msgs::PointCloudConstPtr &cloud) {
  
  int pt_n = (cloud->points).size();
  // ROS_INFO("pointNum: %d", pt_n);
  Point2D32f *pts_ptr = new Point2D32f[pt_n];
  for (int i = 0; i < pt_n; i++) {
    pts_ptr[i].x = cloud->points[i].x;
    pts_ptr[i].y = cloud->points[i].y;
  }

  float lines[4] = {0.0};
  Ransac(pts_ptr, pt_n, lines, numForEstimate, successProbability,
         maxOutliersPercentage);
  float a = lines[1] / lines[0];
  float b = lines[3] - a * lines[2];

  show_ransac_line(a, b);

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "get_position");
  ros::NodeHandle nh;

  ros::Subscriber pavo_subscriber = nh.subscribe<sensor_msgs::PointCloud>(
      "/pavo_scan2cloud", 50, get_position_Callback);

  ros::Rate rate(30.0);
  while (ros::ok()) {

    ros::spinOnce(); // allow data update from callback;
    rate.sleep();
  }
  return 0;
}
