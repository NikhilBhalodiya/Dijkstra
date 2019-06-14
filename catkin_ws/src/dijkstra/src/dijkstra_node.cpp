#include <ros/ros.h>
#include <dijkstra.hpp>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "dijkstra_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh;

  DJ::Dijkstra dijkstra(nh);

  while(ros::ok)
  {
     ros::spin();

  }
  ROS_INFO("Hello world!");

  return 0;
}
