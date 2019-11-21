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
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>

ros::Publisher pointcloud_publisher;
// Mutex: //
boost::mutex cloud_mutex;

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

void pointcloud_callback(const sensor_msgs::PointCloudConstPtr &pointcloud_msg) {

    sensor_msgs::PointCloud tmpcloud;
    sensor_msgs::PointCloud2 cloud_pub;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    tmpcloud = *pointcloud_msg;
    cloud_mutex.lock ();    // for not overwriting the point cloud from another thread
    //cloud->points.resize(tmpcloud.points.size());
    cloud->header.frame_id = tmpcloud.header.frame_id;
    cloud->header.seq = tmpcloud.header.seq;
    cloud->header.stamp = tmpcloud.header.stamp.toNSec() / 1000ull;
    cloud->height = 1;
    cloud->width = tmpcloud.points.size();
    for (int i = 0; i < tmpcloud.points.size(); i++)
    {
      cloud->points.push_back(pcl::PointXYZ(tmpcloud.points[i].x,tmpcloud.points[i].y,0));
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;   //欧式聚类对象
    ec.setClusterTolerance (0.009);                     // 设置近邻搜索的搜索半径为0.0cm
    ec.setMinClusterSize (10);                 //设置一个聚类需要的最少的点数目为10
    ec.setMaxClusterSize (2500);               //设置一个聚类需要的最大点数目为2500
    ec.setSearchMethod (tree);                    //设置点云的搜索机制
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);           //从点云中提取聚类，并将点云索引保存在cluster_indices中
    //迭代访问点云索引cluster_indices,直到分割处所有聚类
    int j = 0;
    std::vector<float> coefficients_value;
    float coefficients_tem[6];
    for(int temp=0; temp<6; temp++){
        coefficients_tem[temp] = 0.0;
    }
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        
        cloud_cluster->points.push_back (cloud->points[*pit]); 
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "No: " << j << std::endl;
        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        //使用SAC对聚类得到的点云块提取直线
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_LINE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.001);
        seg.setInputCloud(cloud_cluster);
        seg.segment(*inliers, *coefficients);

/*   /********显示提取直线模型的模型系数，发布显示拟合的直线，提取内点并发布********
        int coefficients_num = coefficients->values.size();
        std::vector<float> coefficients_value;
        for (size_t i = 0; i < coefficients_num; i++) {
            coefficients_value.push_back(coefficients->values[i]);
            std::cout << coefficients->values[i] << "  ";
        }
        std::cout << std::endl;
        show_ransac_line(coefficients_value);//显示拟合直线模型

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_cluster);
        extract.setIndices(inliers);
        extract.filter(*cloud_seg); //提取内点

        //转换回PointCloud2并发布
        pcl::toROSMsg(*cloud_seg, cloud_pub);
        cloud_pub.header.frame_id = "scan_frame";
        cloud_pub.header.stamp = ros::Time::now();
        pointcloud_publisher.publish(cloud_pub);
        ros::Duration(0.2).sleep();
*/
        int coefficients_num = coefficients->values.size();
        if(coefficients_num != 6){
            ROS_INFO("Model Error!");
            return;
        }
        for (int i = 0; i < coefficients_num; i++) {
            coefficients_tem[i] = coefficients_tem[i] + coefficients->values[i];
            std::cout << coefficients->values[i] << "  ";
        }
        std::cout << std::endl;
        j++;
    }
    for (int i = 0; i < 6; i++) {
        coefficients_tem[i] = coefficients_tem[i] / j;
        coefficients_value.push_back(coefficients_tem[i]);
        std::cout << coefficients_tem[i] << "  ";
    }
    show_ransac_line(coefficients_value);//显示拟合直线模型
    std::cout << std::endl;

    cloud_mutex.unlock ();

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "cloud_sac_segmentation");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud>("/pavo_scan2cloud", 1, pointcloud_callback);
  pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("SegmentationPointCloud", 1);

  double sample_rate = 20;         // HZ
  ros::Rate naptime(sample_rate); // use to regulate loop rate

  while (ros::ok()) {

    ros::spinOnce(); // allow data update from callback;
    naptime.sleep(); // wait for remainder of specified period;
  }
}


