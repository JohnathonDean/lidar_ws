
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>

#include "pcl_ros/point_cloud.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

using namespace std;

ros::Publisher pointcloud_publisher;
ros::Publisher pose_pub;
// Mutex: //
boost::mutex cloud_mutex;
double pose_distance;   //lidar中心到二维码中心的距离（mm）
double angel_a;
double angle_b;
int poseFlag;

static inline void show_ransac_line(std::vector<float> &a) {
  static ros::NodeHandle _rviz_line_nh;
  static ros::Publisher rviz_line_pub;
  static bool needInitial = true;
  if (needInitial) {
    needInitial = false;
    rviz_line_pub = _rviz_line_nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  }
  visualization_msgs::Marker line_list;
  float f = 0.0;
  line_list.header.frame_id = "cloud";
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "lines";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 0.1;
  line_list.id = 1;
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

static inline void show_point(float a, float b , float c) {
  static ros::NodeHandle _rviz_point_nh;
  static ros::Publisher rviz_point_pub;
  static bool needInitial = true;
  if (needInitial) {
    needInitial = false;
    rviz_point_pub = _rviz_point_nh.advertise<visualization_msgs::Marker>("visualization_point_marker", 10);
  }
  visualization_msgs::Marker points;
  points.header.frame_id = "cloud";
  points.header.stamp = ros::Time::now();
  points.ns = "points";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 0.1;
  points.id = 2;
  points.type = visualization_msgs::Marker::POINTS;

  points.scale.x = 0.01;
  points.scale.y = 0.01;
  points.color.g = 1.0f;
  points.color.a = 1.0;
  geometry_msgs::Point p;
  p.x = a;
  p.y = b;
  p.z = c;
  points.points.push_back(p);
  rviz_point_pub.publish(points);
}

void getEndpoint(std::vector<float> &a, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, std::vector<float> &b) {
  float k;
  float k_numerator, k_denominator;
  float x0, y0, z0;  //点云上的点到拟合直线垂足的坐标
  float x_min, y_min, z_min, x_max, y_max, z_max;  //标志物聚类团块两端点坐标
  bool InitialFlag = true;
  for (int i = 0; i < cloud_in->points.size(); i++) {
    k_numerator = (cloud_in->points[i].x - a[0])*a[3] + (cloud_in->points[i].y - a[1])*a[4] + (cloud_in->points[i].z - a[2])*a[5];
    k_denominator = pow(a[3], 2) + pow(a[4], 2) + pow(a[5], 2);
    k = k_numerator / k_denominator;
    x0 = a[0] + k * a[3];
    y0 = a[1] + k * a[4];
    z0 = a[2] + k * a[5];
    if(InitialFlag){
      x_min = x0; y_min = y0; z_min = z0;
      x_max = x0; y_max = y0; z_max = z0;
      InitialFlag = false;
    }
    else{
      if(x_min > x0){
        x_min = x0; y_min = y0; z_min = z0;
      }
      if(x_max < x0){
        x_max = x0; y_max = y0; z_max = z0;
      }
    }
  }
  b.push_back(x_min);
  b.push_back(y_min);
  b.push_back(z_min);
  b.push_back(x_max);
  b.push_back(y_max);
  b.push_back(z_max);

  // show_point(x_min, y_min, z_min);
  // ros::Duration(0.5).sleep();
  // show_point(x_max, y_max, z_max);
  return;
}

float getLineLength(std::vector<float> &a){
  float length;
  length = sqrt( (a[3]-a[0])*(a[3]-a[0]) + (a[4]-a[1])*(a[4]-a[1]) + (a[5]-a[2])*(a[5]-a[2]) );
  return length;
}

void pointcloud_callback(const sensor_msgs::PointCloudConstPtr &pointcloud_msg) {
  sensor_msgs::PointCloud tmpcloud;
  sensor_msgs::PointCloud2 cloud_pub;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg(new pcl::PointCloud<pcl::PointXYZ>);

  cloud_mutex.lock(); // for not overwriting the point cloud from another thread
  tmpcloud = *pointcloud_msg;
  // cloud->points.resize(tmpcloud.points.size());
  cloud->header.frame_id = tmpcloud.header.frame_id;
  cloud->header.seq = tmpcloud.header.seq;
  cloud->header.stamp = tmpcloud.header.stamp.toNSec() / 1000ull;
  cloud->height = 1;
  cloud->width = tmpcloud.points.size();
  for (int i = 0; i < tmpcloud.points.size(); i++) {
    cloud->points.push_back(pcl::PointXYZ(tmpcloud.points[i].x, tmpcloud.points[i].y, 0));
  }

  if(cloud->width > 15){
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

    // pcl::ExtractIndices<pcl::PointXYZ> extract;
    // extract.setInputCloud(cloud);
    // extract.setIndices(inliers);
    // extract.filter(*cloud_seg); //提取内点
    poseFlag = 1;  //提取到直线，则位姿有效
    int coefficients_num = coefficients->values.size();
    vector<float> coefficients_value;
    cout << "coefficients:  ";
    for (size_t i = 0; i < coefficients_num; i++) {
      coefficients_value.push_back(coefficients->values[i]);
      cout << coefficients->values[i] << "  ";
    }
    cout << endl;
    show_ransac_line(coefficients_value); //显示拟合直线模型

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec; //欧式聚类对象
    ec.setClusterTolerance(0.01); // 设置近邻搜索的搜索半径为0.0cm
    ec.setMinClusterSize(10); //设置一个聚类需要的最少的点数目为10
    ec.setMaxClusterSize(2500); //设置一个聚类需要的最大点数目为2500
    ec.setSearchMethod(tree);   //设置点云的搜索机制
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices); //从点云中提取聚类，并将点云索引保存在cluster_indices中
    //迭代访问点云索引cluster_indices,直到分割处所有聚类
    int j = 0;
    float midpoint[3];
    float Point1[3], Point2[3];  //提取标志物的两端点，以对标志物中点定位
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)

      cloud_cluster->points.push_back(cloud->points[*pit]);
      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      cout << "No: " << j+1 << "  ";
      cout << "PointCloud representing the Cluster: "
                << cloud_cluster->points.size() << " data points." << endl;
      
      vector<float> endpoint; //单端标记的两端点
      getEndpoint(coefficients_value, cloud_cluster, endpoint);
      // cout << endpoint[0] << "  " << endpoint[1] << "  " << endpoint[2] << "  ";
      // cout << endpoint[3] << "  " << endpoint[4] << "  " << endpoint[5] << "         ";
      if(j == 1){
        Point1[0] = endpoint[0]; Point1[1] = endpoint[1]; Point1[2] = endpoint[2];
        Point2[0] = endpoint[3]; Point2[1] = endpoint[4]; Point2[2] = endpoint[5];
      }
      else{
        if(Point1[0] > endpoint[0]){
          Point1[0] = endpoint[0]; Point1[1] = endpoint[1]; Point1[2] = endpoint[2];
        }
        if(Point2[0] < endpoint[5]){
          Point2[0] = endpoint[3]; Point2[1] = endpoint[4]; Point2[2] = endpoint[5];
        }
      }

      float cloud_length;
      cloud_length = getLineLength(endpoint);
      // cout << "cloud_length: " << cloud_length << endl;   //长度特征可以用于判别是否为充电标志物

      //转换回PointCloud2并发布
      pcl::toROSMsg(*cloud_cluster, cloud_pub);
      cloud_pub.header.frame_id = "cloud";
      cloud_pub.header.stamp = ros::Time::now();
      pointcloud_publisher.publish(cloud_pub);
      ros::Duration(0.5).sleep();

      j++;
    }
    if(j > 3){
      ROS_INFO("EuclideanClusterExtraction Error!"); //标志物点云团块取3个，否则提示报错
      poseFlag = 0;  //使发布的位姿无效
    }

    midpoint[0] = (Point1[0] + Point2[0]) / 2.0;
    midpoint[1] = (Point1[1] + Point2[1]) / 2.0;
    midpoint[2] = (Point1[2] + Point2[2]) / 2.0;
    show_point(midpoint[0], midpoint[1], midpoint[2]);

  //计算位姿并发布
    pose_distance = sqrt(midpoint[0]*midpoint[0]+midpoint[1]*midpoint[1]+midpoint[2]*midpoint[2]);
    angel_a = atan2(midpoint[1], midpoint[0]);  
    angle_b = atan2(coefficients_value[4], coefficients_value[3]);  
    std_msgs::Float32MultiArray msg_array;
    msg_array.data.push_back(poseFlag);
    msg_array.data.push_back(pose_distance);
    msg_array.data.push_back(angel_a);
    msg_array.data.push_back(angle_b);
    pose_pub.publish(msg_array);
  }
  
  cloud_mutex.unlock();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "cloud_sac_segmentation");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud>("/sick_scan2cloud", 1, pointcloud_callback);
  pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("ec_SegmentationPointCloud", 1);
  pose_pub = nh.advertise<std_msgs::Float32MultiArray>("charge_pose", 1000);

  double sample_rate = 20;        // HZ
  ros::Rate naptime(sample_rate); // use to regulate loop rate

  while (ros::ok()) {

    ros::spinOnce(); // allow data update from callback;
    naptime.sleep(); // wait for remainder of specified period;
  }
}
