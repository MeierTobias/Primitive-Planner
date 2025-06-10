#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <math.h>

using namespace std;

template <class T>
struct Range
{
  T low, high;
};

pcl::PointCloud<pcl::PointXYZ> generateRectangle(Range<double> x, Range<double> y, Range<double> z, double density)
{
  pcl::PointCloud<pcl::PointXYZ> cloudMap;
#define DRAW_2D_RECT(u, v, w)                        \
  for (double i = u.low; i < u.high; i += density)   \
    for (double j = v.low; j < v.high; j += density) \
    {                                                \
      pcl::PointXYZ pt;                              \
      pt.u = i;                                      \
      pt.v = j;                                      \
      pt.w = w.low;                                  \
      cloudMap.points.push_back(pt);                 \
      pt.w = w.high;                                 \
      cloudMap.points.push_back(pt);                 \
    }

  // For the first two dimensions the order doesn't matter, but the last dimension (over which we don't iterate) must be passed once as x, y, and z
  DRAW_2D_RECT(x, y, z);
  DRAW_2D_RECT(y, z, x);
  DRAW_2D_RECT(z, x, y);

  cloudMap.width = cloudMap.points.size();
  ROS_WARN("size: %ld", cloudMap.points.size());
  cloudMap.height = 1;
  cloudMap.is_dense = true;
  return cloudMap;
}

template <class T>
inline T getParam(ros::NodeHandle &n, const char *key)
{
  T value;
  ROS_ASSERT(n.getParam(key, value) == true);
  return value;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "random_map_sensing");
  ros::NodeHandle n("~");

  ros::Publisher map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);

  Range<double> x, y, z;
  double resolution, pub_rate;

  x.low = getParam<double>(n, "map/x_low");
  x.high = getParam<double>(n, "map/x_high");
  y.low = getParam<double>(n, "map/y_low");
  y.high = getParam<double>(n, "map/y_high");
  z.low = getParam<double>(n, "map/z_low");
  z.high = getParam<double>(n, "map/z_high");
  n.param("map/resolution", resolution, 0.1);
  n.param("pub_rate", pub_rate, 10.0);

  ros::Duration(0.5).sleep();

  auto cloudMap = generateRectangle(x, y, z, resolution);

  ros::Rate loop_rate(pub_rate);

  while (ros::ok())
  {
    sensor_msgs::PointCloud2 globalMap_pcd;
    pcl::toROSMsg(cloudMap, globalMap_pcd);
    globalMap_pcd.header.frame_id = "world";
    map_pub.publish(globalMap_pcd);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
