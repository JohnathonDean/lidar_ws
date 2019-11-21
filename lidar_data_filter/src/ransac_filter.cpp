#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/Point32.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdio.h>
#include <string.h>
#include <thread>
#include <vector>
#include <iostream>

#include "pcl_ros/point_cloud.h"
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

using namespace std;
// 全局变量
ros::Publisher pointcloud_publisher;

class LeastSquare
{
    public:
    float a, b;
    float error;

    public:
    LeastSquare(const vector<float> &x, const vector<float> &y)
    {
        float t1 = 0, t2 = 0, t3 = 0, t4 = 0;
        for (int i = 0; i < x.size(); i++)
        {
            t1 += x[i] * x[i];
            t2 += x[i];
            t3 += x[i] * y[i];
            t4 += y[i];
        }
        a = (t3 * x.size() - t2 * t4) / (t1 * x.size() - t2 * t2);
        b = (t1 * t4 - t2 * t3) / (t1 * x.size() - t2 * t2);
        error = 0;
        for (int i = 0; i < x.size(); i++)
        {
            error += (y[i] - getY(x[i]));
        }
        error = error / x.size();
    }

    float getY(const float x) const
    {
        return a * x + b;
    }
    void print() const
    {
        cout << "y = " << a << "x + " << b << endl;
    }
};

void pointcloud_callback(
    const sensor_msgs::PointCloudConstPtr &pointcloud_msg) {

  sensor_msgs::PointCloud tmpcloud;
  sensor_msgs::PointCloud2 cloud_pub;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  tmpcloud = *pointcloud_msg;
  // cloud->points.resize(tmpcloud.points.size());
  cloud->header.frame_id = tmpcloud.header.frame_id;
  cloud->header.seq = tmpcloud.header.seq;
  cloud->header.stamp = tmpcloud.header.stamp.toNSec() / 1000ull;
  cloud->height = 1;
  cloud->width = tmpcloud.points.size();
  for (int i = 0; i < tmpcloud.points.size(); i++) {
    cloud->points.push_back(
        pcl::PointXYZ(tmpcloud.points[i].x, tmpcloud.points[i].y, 0));
    // cloud->points[i].x = tmpcloud.points[i].x;
    // cloud->points[i].y = tmpcloud.points[i].y;
    // cloud->points[i].z = 0;
  }

  // RANSAC算法滤波
  int n = 8;
  int iterations_k = 10;
  float t = 0.02;
  int d = 15;

  int iterations = 0;
  float best_lin_a = 0, best_lin_b = 0;
  vector<float> best_consensus_set_x;
  vector<float> best_consensus_set_y;
  float best_error = numeric_limits<float>::max();

  while (iterations < iterations_k) {
    vector<float> maybe_inliers_x;
    vector<float> maybe_inliers_y;
    vector<float> consensus_set_x;
    vector<float> consensus_set_y;
    vector<int> index_v;
    float maybe_lin_a;
    float maybe_lin_b;
    float better_lin_a;
    float better_lin_b;
    float this_error = 0;

    for (int i = 0; i < n; i++) {
      int index = rand() % tmpcloud.points.size();
      vector<int>::iterator result =
          find(index_v.begin(), index_v.end(), index);
      if (result != index_v.end()) {
        index = rand() % tmpcloud.points.size();
      }
      cout << "size:" << tmpcloud.points.size() << endl;
      cout << "index:" << index << endl;
      index_v.push_back(index);
      maybe_inliers_x.push_back(tmpcloud.points[index].x);
      maybe_inliers_y.push_back(tmpcloud.points[index].y);
    }
    LeastSquare ls(maybe_inliers_x, maybe_inliers_y);
    maybe_lin_a = ls.a;
    maybe_lin_b = ls.b;
    consensus_set_x = maybe_inliers_x;
    consensus_set_y = maybe_inliers_y;
    for (int i = 0; i < tmpcloud.points.size(); i++) {
      vector<int>::iterator result = find(index_v.begin(), index_v.end(), i);
      if (result == index_v.end()) //没找到
      {
        // cout<<"t:"<<fabs(ls.getY(tmpcloud.points[i].x) - tmpcloud.points[i].y)<<endl;
        if (fabs(ls.getY(tmpcloud.points[i].x) - tmpcloud.points[i].y) < t) {
          consensus_set_x.push_back(tmpcloud.points[i].x);
          consensus_set_y.push_back(tmpcloud.points[i].y);
        }
      } else {
        break;
      }
    }
    if (consensus_set_x.size() > d) {
      LeastSquare ls1(consensus_set_x, consensus_set_y);
      better_lin_a = ls1.a;
      better_lin_b = ls1.b;
      this_error = ls1.error;
      if (this_error < best_error) {
        best_lin_a = better_lin_a;
        best_lin_b = better_lin_b;
        best_consensus_set_x = consensus_set_x;
        best_consensus_set_y = consensus_set_y;
        best_error = this_error;
      }
    }
    iterations++;
  }

  sensor_msgs::PointCloud tmpcloud1;
  tmpcloud1.header = tmpcloud.header;
  tmpcloud1.points.resize(best_consensus_set_x.size());
  tmpcloud1.channels.resize(1);
  tmpcloud1.channels[0].name = tmpcloud.channels[0].name;
  tmpcloud1.channels[0].values.resize(best_consensus_set_x.size());
  for (int i = 0; i < best_consensus_set_x.size(); i++) {
    geometry_msgs::Point32 p;
    p.x = best_consensus_set_x[i];
    p.y = best_consensus_set_y[i];
    p.z = 0;
    tmpcloud1.points[i] = p;
    tmpcloud1.channels[0].values[i] = 200;
  }
  pointcloud_publisher.publish(tmpcloud1);
  cout << "a:" << best_lin_a << endl;
  cout << "b:" << best_lin_b << endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "cloud_ransac");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud>(
      "/pavo_scan2cloud", 1, pointcloud_callback);
  pointcloud_publisher =
      nh.advertise<sensor_msgs::PointCloud>("RANSAC_PointCloud", 1);
  
  double sample_rate = 20;        // HZ
  ros::Rate naptime(sample_rate); // use to regulate loop rate

  while (ros::ok()) {

    ros::spinOnce(); // allow data update from callback;
    naptime.sleep(); // wait for remainder of specified period;
  }
}
