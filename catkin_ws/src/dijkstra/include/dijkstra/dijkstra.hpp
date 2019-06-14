#pragma once
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <queue>


namespace DJ {
using namespace std;

class Dijkstra
{
public:
    Dijkstra(ros::NodeHandle &nh);
    ~Dijkstra();
private:
    ros::Subscriber map_sub;
    ros::Subscriber goal_sub;
    ros::Publisher map_pub;

    nav_msgs::OccupancyGrid::ConstPtr map;
    geometry_msgs::PoseStamped::ConstPtr goal;

    geometry_msgs::PoseStamped::ConstPtr start;

    nav_msgs::OccupancyGrid updated_map;

    bool once = false;
    bool m_have_goal = false;

    double start_grid_x;
    double start_grid_y;
    double goal_grid_x;
    double goal_grid_y;

    void planPathDijkstra();
    void drawStartAndGoal();
    void addStarToQueue();

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void StartEndGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void removeCurrGoal();
};



}
