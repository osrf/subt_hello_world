#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include <string>

geometry_msgs::TransformStamped ToTransformStamped(
  const geometry_msgs::PoseStamped & offset,
  const std::string& parent_frame,
  const std::string& child_frame)
{
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = offset.header.stamp;
  transformStamped.header.frame_id = parent_frame;
  transformStamped.child_frame_id = child_frame;
  transformStamped.transform.translation.x = offset.pose.position.x;
  transformStamped.transform.translation.y = offset.pose.position.y;
  transformStamped.transform.translation.z = offset.pose.position.z;
  transformStamped.transform.rotation.x = offset.pose.orientation.x;
  transformStamped.transform.rotation.y = offset.pose.orientation.y;
  transformStamped.transform.rotation.z = offset.pose.orientation.z;
  transformStamped.transform.rotation.w = offset.pose.orientation.w;
  return transformStamped;
}