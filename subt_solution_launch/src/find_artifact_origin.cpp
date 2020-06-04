#include "ignition/math/Pose3.hh"
#include "ros/ros.h"
#include "subt_msgs/PoseFromArtifact.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "ToTransformStamped.h"

std::string robot_name;
std::string base_link_frame;
std::string map_frame;
std::string artifact_origin_frame;
std::string service_name = "/subt/pose_from_artifact_origin";

void InvertTransform(subt_msgs::PoseFromArtifact & service);

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "artifact_origin_finder");
  ros::NodeHandle nh;

  ros::NodeHandle private_nh("~");
  private_nh.param("robot_name", robot_name, std::string("X1"));
  private_nh.param("base_link_frame", base_link_frame, robot_name);
  private_nh.param("map_frame", map_frame, robot_name + "/map");
  private_nh.param("artifact_origin_frame", artifact_origin_frame, std::string("artifact_origin"));
  ROS_INFO_STREAM(
    "artifact_origin_finder values..." << std::endl <<
    "robot_name: " << robot_name << std::endl <<
    "base_link_frame: " << base_link_frame << std::endl <<
    "map_frame: " << map_frame << std::endl <<
    "artifact_origin_frame: " << artifact_origin_frame << std::endl);

  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // set up service call
  ros::ServiceClient client = nh.serviceClient<subt_msgs::PoseFromArtifact>(service_name);
  subt_msgs::PoseFromArtifact srv;
  srv.request.robot_name.data = robot_name;

  // ensure that transform between map->base_link is available before calling service
  if (!tf_buffer.canTransform(map_frame, base_link_frame, ros::Time(0), ros::Duration(5.0)))
  {
    ROS_FATAL("could not find transform between map and base_link, exiting now");
    ros::shutdown();
  }

  // call service and save transforms
  if (client.call(srv))
  {
    // need to convert the transform from artifact_origin->base_link to base_link->artifact_origin
    // (this will allow us to get the map->artifact_origin transform)
    InvertTransform(srv);

    // save static transfrom between map->artifact_origin
    try
    {
      auto tf_info = tf_buffer.transform<geometry_msgs::PoseStamped>(srv.response.pose,
        map_frame, ros::Duration(3.0));
      ROS_INFO_STREAM(map_frame << "->" << artifact_origin_frame << ": " <<
        tf_info.pose.position.x << "," << tf_info.pose.position.y << "," << tf_info.pose.position.z);

      auto transformStamped = ToTransformStamped(tf_info, map_frame,
        artifact_origin_frame);

      // periodically broadcast the transform between map->artifact_origin
      // in case other nodes start later that need it
      ros::Rate r(1); // 1 Hz
      while (ros::ok())
      {
        static_tf_broadcaster.sendTransform(transformStamped);
        r.sleep();
      }
    }
    catch(tf2::TransformException &ex)
    {
      ROS_FATAL("%s", ex.what());
      ros::shutdown();
    }
  }
  else
  {
    ROS_FATAL_STREAM("service call failed: " << service_name);
    ros::shutdown();
  }

  // shouldn't ever get here
  return -1;
}

void InvertTransform(subt_msgs::PoseFromArtifact & service)
{
  ignition::math::Vector3 translation(
    service.response.pose.pose.position.x,
    service.response.pose.pose.position.y,
    service.response.pose.pose.position.z
    );
  ignition::math::Quaternion rotation(
    service.response.pose.pose.orientation.w,
    service.response.pose.pose.orientation.x,
    service.response.pose.pose.orientation.y,
    service.response.pose.pose.orientation.z
    );

  ignition::math::Pose3 originalTF(translation, rotation);
  auto invertedTF = originalTF.Inverse();

  service.response.pose.pose.position.x = invertedTF.Pos().X();
  service.response.pose.pose.position.y = invertedTF.Pos().Y();
  service.response.pose.pose.position.z = invertedTF.Pos().Z();
  service.response.pose.pose.orientation.w = invertedTF.Rot().W();
  service.response.pose.pose.orientation.x = invertedTF.Rot().X();
  service.response.pose.pose.orientation.y = invertedTF.Rot().Y();
  service.response.pose.pose.orientation.z = invertedTF.Rot().Z();
  service.response.pose.header.frame_id = base_link_frame;
}