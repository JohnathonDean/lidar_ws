#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

sensor_msgs::LaserScan pub;
float lower_threshold = 31000;
float upper_threshold = 38000;

void scan_calculate(sensor_msgs::LaserScan &input_scan) {

  const double hist_max = 5 * 9000.0;
  const int num_buckets = 30;
  int histogram[num_buckets];
  for (int i = 0; i < num_buckets; i++)
    histogram[i] = 0;

  // Need to check ever reading in the current scan  
  for (unsigned int i = 0; i < input_scan.ranges.size() && i < input_scan.intensities.size(); i++) {
    // Is this reading below our lower threshold?
    // Is this reading above our upper threshold?
    if (input_scan.intensities[i] <= lower_threshold ||
        input_scan.intensities[i] >= upper_threshold)
    {
      // If so, then make it an invalid value (NaN)
      input_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
    }

    // Calculate histogram
    // If intensity value is inf or NaN, skip voting histogram
    if (std::isinf((double)input_scan.intensities[i]) ||
        std::isnan((double)input_scan.intensities[i]))
      continue;

    // Choose bucket to vote on histogram,
    // and check the index of bucket is in the histogram array
    int cur_bucket = (int)(input_scan.intensities[i] / hist_max * num_buckets);
    if (cur_bucket > num_buckets - 1)
      cur_bucket = num_buckets - 1;
    else if (cur_bucket < 0)
      cur_bucket = 0;
    histogram[cur_bucket]++;
  }

  // Display Histogram
  // printf("********** SCAN **********\n");
  // for (int i = 0; i < num_buckets; i++) {
  //   printf("%u - %u: %u\n", (unsigned int)hist_max / num_buckets * i,
  //          (unsigned int)hist_max / num_buckets * (i + 1), histogram[i]);
  // }
}

void pavoData_Callback(const sensor_msgs::LaserScanConstPtr &scanMsg) {

  float pointNum = (scanMsg->intensities).size();
  pub = *scanMsg;
  for(int i = 0; i < pointNum; ++i){
      pub.intensities[i] = pub.intensities[i] * 1.0; //将强度值放大100倍
  }
  scan_calculate(pub);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_scan_pub");
  ros::NodeHandle nh;

  ros::Subscriber pavo_subscriber =
      nh.subscribe<sensor_msgs::LaserScan>("/scan", 50, pavoData_Callback);
  ros::Publisher pointcloud_publisher =
      nh.advertise<sensor_msgs::LaserScan>("filtered_output", 1);
  
  ros::Rate rate(30.0); 
  while (ros::ok()) {

    pointcloud_publisher.publish(pub);

    ros::spinOnce(); // allow data update from callback;
    rate.sleep();
  }
  return 0;
}

