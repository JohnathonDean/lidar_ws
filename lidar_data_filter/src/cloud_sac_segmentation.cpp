
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>

#include "pcl_ros/point_cloud.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

using namespace std;

ros::Publisher pointcloud_publisher;

static inline void show_ransac_line(std::vector<float> &a) {
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
  p.x = a[0] - 10 * a[3];
  p.y = a[1] - 10 * a[4];
  p.z = a[2] - 10 * a[5];
  line_list.points.push_back(p);
  p.x = a[0] + 10 * a[3];
  p.y = a[1] + 10 * a[4];
  p.z = a[2] + 10 * a[5];
  line_list.points.push_back(p);
  rviz_line_pub.publish(line_list);
}

void pointcloud_callback(
    const sensor_msgs::PointCloudConstPtr &pointcloud_msg) {
  sensor_msgs::PointCloud tmpcloud;
  sensor_msgs::PointCloud2 cloud_pub;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg(
      new pcl::PointCloud<pcl::PointXYZ>);

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

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_LINE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.001);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  int coefficients_num = coefficients->values.size();
  vector<float> coefficients_value;
  for (size_t i = 0; i < coefficients_num; i++) {
    coefficients_value.push_back(coefficients->values[i]);
    cout << coefficients->values[i] << "  ";
  }
  cout << endl;
  /** \brief SampleConsensusModelLine defines a model for 3D line segmentation.
  * The model coefficients are defined as:
  *   - \b point_on_line.x  : the X coordinate of a point on the line
  *   - \b point_on_line.y  : the Y coordinate of a point on the line
  *   - \b point_on_line.z  : the Z coordinate of a point on the line
  *   - \b line_direction.x : the X coordinate of a line's direction
  *   - \b line_direction.y : the Y coordinate of a line's direction
  *   - \b line_direction.z : the Z coordinate of a line's direction
  *    输出的coefficients中有6个参数*/
  show_ransac_line(coefficients_value);//显示拟合直线模型

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.filter(*cloud_seg); //提取内点

  pcl::toROSMsg(*cloud_seg, cloud_pub);
  pointcloud_publisher.publish(cloud_pub);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "cloud_sac_segmentation");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud>(
      "/pavo_scan2cloud", 1, pointcloud_callback);
  pointcloud_publisher =
      nh.advertise<sensor_msgs::PointCloud2>("SACSegmentationPointCloud", 1);

  double sample_rate = 20;        // HZ
  ros::Rate naptime(sample_rate); // use to regulate loop rate

  while (ros::ok()) {

    ros::spinOnce(); // allow data update from callback;
    naptime.sleep(); // wait for remainder of specified period;
  }
}
