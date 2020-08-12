#include "ros/ros.h"
#include "subt_communication_broker/subt_communication_client.h"
#include "subt_ign/CommonTypes.hh"

std::string robot_name;
std::string other_robot;
subt::CommsClient* commsClient;

void IncomingMsgCallback(
  const std::string & srcAddress,
  const std::string & dstAddress,
  const uint32_t dstPort,
  const std::string & data);

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "artifact_report_relay");
  ros::NodeHandle nh;

  ros::NodeHandle private_nh("~");
  private_nh.param("robot_name", robot_name, std::string("X1"));
  private_nh.param("other_robot", other_robot, std::string("X2"));

  // set up communications for artifact reporting
  commsClient = new subt::CommsClient(robot_name);
  commsClient->Bind(&IncomingMsgCallback, robot_name);

  ros::spin();
}

void IncomingMsgCallback(
  const std::string & srcAddress,
  const std::string & dstAddress,
  const uint32_t dstPort,
  const std::string & data)
{
  if (srcAddress == other_robot)
  {
    // we received an artifact from the other robot,
    // so we need to send this artifact message to the base station
    commsClient->SendTo(data, subt::kBaseStationName);
  }
  else if (srcAddress == subt::kBaseStationName);
  {
    // the base station has received the artifact we relayed from the other robot,
    // so we need to let the other robot know that the relay succeeded
    commsClient->SendTo(data, other_robot);
  }
}