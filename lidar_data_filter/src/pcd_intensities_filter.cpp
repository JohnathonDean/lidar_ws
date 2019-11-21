#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

sensor_msgs::PointCloud pub;
float intensity_min_div_max = 0.8;

float get_max_intensity(sensor_msgs::PointCloud &input_cloud) {
  int pointNum = input_cloud.points.size();
  float intensity_max = 0.0;
  float intensity;
  for (int i = 0; i < pointNum; i++) {
    if (input_cloud.points[i].x != 0 || input_cloud.points[i].y != 0) {
      intensity = input_cloud.channels[0].values[i];
      if (intensity_max < intensity) {
        intensity_max = intensity;
      }
    }
  }
  ROS_INFO("intensity_max: %f", intensity_max);
  return intensity_max;
}

void pcd_intensity_filter(sensor_msgs::PointCloud &input_cloud,
                          sensor_msgs::PointCloud &filtered_cloud,
                          float max_intensity, float intensity_min_div_max) {

    sensor_msgs::PointCloud cloud;
    int num_points;
    num_points = input_cloud.points.size();
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "pcd_frame";

    //add an intensity channel to the cloud
    cloud.channels.resize(1);
    cloud.channels[0].name = "intensities";

    for (unsigned int i = 0; i < num_points; ++i) {
        float intensity = input_cloud.channels[0].values[i];
        if (intensity > max_intensity * intensity_min_div_max &&
            intensity < max_intensity) {
            cloud.points[i].x = input_cloud.points[i].x;
            cloud.points[i].y = input_cloud.points[i].y;
            cloud.points[i].z = input_cloud.points[i].z;
            cloud.channels[0].values[i] = input_cloud.channels[0].values[i];
        }
    }
    int pointNum = cloud.points.size();
    ROS_INFO("pointNum: %d", pointNum);
    filtered_cloud = cloud;

}

void pavoData_Callback(const sensor_msgs::PointCloudConstPtr &cloud) {
  int pointNum = (cloud->points).size();
  //ROS_INFO("pointNum: %d", pointNum);
  sensor_msgs::PointCloud excloud;
  excloud = *cloud;
  for (int i = 0; i < pointNum; ++i) {
    excloud.channels[0].values[i] =
        excloud.channels[0].values[i] * 100.0; //将强度值放大100倍
    // float m;
    // m = excloud.channels[0].values[i];
    // ROS_INFO("  %f", m);
  }

  float intensity_max;
  intensity_max = get_max_intensity(excloud);
  pcd_intensity_filter(excloud, pub, intensity_max, intensity_min_div_max);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_pcd_pub");
  ros::NodeHandle nh;

  ros::Subscriber pavo_subscriber =
      nh.subscribe<sensor_msgs::PointCloud>("cloud", 50, pavoData_Callback);
  ros::Publisher pointcloud_publisher =
      nh.advertise<sensor_msgs::PointCloud>("filtered_output", 1);

  ros::Rate rate(30.0);
  while (ros::ok()) {

    pointcloud_publisher.publish(pub);

    ros::spinOnce(); // allow data update from callback;
    rate.sleep();
  }
  return 0;
}
