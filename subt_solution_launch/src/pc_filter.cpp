#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <cmath>

float flat_height;
float slope_threshold;

std::string name;
std::string flat_frame;
std::string new_frame;

ros::Publisher filtered_pc_pub;
tf2_ros::Buffer tf_buffer;

// The callback method for the subscriber.
// Receives the original point cloud data, filters it, and then
// publishes this filtered point cloud to a new topic
void filter_pc(const sensor_msgs::PointCloud2::ConstPtr & pc);

int main(int argc, char *argv[])
{
  // Start node
  ros::init(argc, argv, "pc_filter");
  ros::NodeHandle nh;

  // Set up parameters.
  // The slope parameter is the upper bound of a height to distance ratio that determines
  // whether a point in a point cloud at a certain location is an obstacle or not
  ros::NodeHandle private_nh("~");
  private_nh.param<float>("flat_height", flat_height, -0.25);
  private_nh.param<float>("slope", slope_threshold, 1.0);
  private_nh.param<std::string>("name", name, "X1");

  // Define transform frames
  flat_frame = name + "/map";
  new_frame = name + "/fake";

  // Set up a publisher for the filtered point cloud
  // and initialize the buffer so we can lookup transforms
  filtered_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("points_filtered", 1);
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // Subscribe to the original point cloud topic
  ros::Subscriber original_pc_sub = nh.subscribe("points", 1, &filter_pc);

  // Start listening for data and trigger the callback when possible
  ros::spin();
}

void filter_pc(const sensor_msgs::PointCloud2::ConstPtr & pc)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr body(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr publishBody(new pcl::PointCloud<pcl::PointXYZ>);
  sensor_msgs::PointCloud2 result;

  geometry_msgs::PointStamped origin;
  geometry_msgs::TransformStamped transformStamped;
  try
  {
    // Find the transform between the real and fake sensor frame,
    // and then apply this transformation to the original point cloud
    // so that we have point cloud data in the fake sensor frame
    transformStamped = tf_buffer.lookupTransform(pc->header.frame_id,
      flat_frame, ros::Time(0), ros::Duration(5.0));
    geometry_msgs::PointStamped tempOrigin;
    tempOrigin.point.x = 0;
    tempOrigin.point.y = 0;
    tempOrigin.point.z = flat_height;
    tempOrigin.header = transformStamped.header;
    tf2::doTransform(tempOrigin, origin, transformStamped);
    tf2::doTransform(*pc, result, transformStamped);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s",ex.what());
    return;
  }

  // Filter the point cloud data.
  // If a point's slope in the point cloud is less than the slope_threshold
  // (the slope is calculated relative to a point's neighbor in the point cloud),
  // we "delete" this point so that move_base doesn't interpret it as an obstacle.
  // We need to do this since move_base is 2D and our world is 3D
  // with varying terrain height
  pcl::fromROSMsg(result, *body);
  pcl::fromROSMsg(*pc, *publishBody);
  for (size_t x = 0; x < body->width; ++x)
  {
    geometry_msgs::PointStamped last_point;
    last_point.point.x = 0;
    last_point.point.y = 0;
    last_point.point.z = 0;
    for (size_t y = 0; y < body->height; ++y)
    {
      geometry_msgs::PointStamped transformed;
      pcl::PointXYZ actual_point = body->at(x,y);
      float diff_z = actual_point.z - last_point.point.z;
      float diff_position = sqrt( pow(actual_point.x - last_point.point.x, 2) +
        pow(actual_point.y - last_point.point.y, 2) );
      if (abs(diff_position) < .0001)
      {
        continue;
      }

      float slope = diff_z / diff_position;
      last_point.point.x = actual_point.x;
      last_point.point.y = actual_point.y;
      last_point.point.z = actual_point.z;
      if (slope == NAN || slope == INFINITY)
      {
        continue;
      }
      else if (slope < slope_threshold)
      {
        // Setting the point's x and y to infinity makes move_base think
        // there's no obstacle at this location
        actual_point.x = INFINITY;
        actual_point.y = INFINITY;
        actual_point.z = origin.point.z;
        publishBody->at(x,y) = actual_point;
      }
      else
      {
        break;
      }
    }
  }

  // Publish the filtered point cloud so move_base can use it
  pcl::toROSMsg(*publishBody, result);
  result.header.frame_id = new_frame + "/sensor_fake";
  filtered_pc_pub.publish(result);
};
