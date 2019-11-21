#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

sensor_msgs::PointCloud pub;

void scan_calculate(const sensor_msgs::PointCloud &input_cloud) {

  const double hist_max = 4 * 8000.0;
  const int num_buckets = 40;
  int histogram[num_buckets];
  for (int i = 0; i < num_buckets; i++)
    histogram[i] = 0;

  // Need to check ever reading in the current scan
  for (unsigned int i = 0; i < input_cloud.points.size(); i++) {
    // Calculate histogram
    // If intensity value is inf or NaN, skip voting histogram
    if (std::isinf((double)input_cloud.channels[0].values[i]) || std::isnan((double)input_cloud.channels[0].values[i]))
      continue;

    // Choose bucket to vote on histogram,
    // and check the index of bucket is in the histogram array
    int cur_bucket = (int)(input_cloud.channels[0].values[i] / hist_max * num_buckets);
    if (cur_bucket > num_buckets - 1)
      cur_bucket = num_buckets - 1;
    else if (cur_bucket < 0)
      cur_bucket = 0;
    histogram[cur_bucket]++;
  }

  // Display Histogram
  printf("********** PCD **********\n");
  for (int i = 0; i < num_buckets; i++) {
    printf("%u - %u: %u\n", (unsigned int)hist_max / num_buckets * i,
           (unsigned int)hist_max / num_buckets * (i + 1), histogram[i]);
  }
}

void pavoData_Callback(const sensor_msgs::PointCloudConstPtr &cloud) {
  int pointNum = (cloud->points).size();
  ROS_INFO("pointNum: %d", pointNum);
  pub = *cloud;
  for(int i = 0; i < pointNum; ++i){
      pub.channels[0].values[i]  = pub.channels[0].values[i] * 100.0; //将强度值放大100倍
      // float m;
      // m = pub.channels[0].values[i];
      // ROS_INFO("  %f", m);
  }
  scan_calculate(pub);
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
