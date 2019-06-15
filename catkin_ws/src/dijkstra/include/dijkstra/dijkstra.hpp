#pragma once
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <queue>
#include <GraphNode.hpp>
#include <algorithm>
#include <vector>

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
    nav_msgs::OccupancyGrid updated_map;
    geometry_msgs::PoseStamped::ConstPtr goal;
    geometry_msgs::PoseStamped::ConstPtr start;

    bool once = false;
    bool first_itr = true;
    bool m_have_goal = false;

    int start_grid_x;
    int start_grid_y;
    int goal_grid_x;
    int goal_grid_y;

    priority_queue<GraphNode, std::vector<GraphNode*>, LessThanByCost> frontier;

    void planPathDijkstra();
    void createStartNode();
    void createGoalNode();
    void drawStartAndGoal();
    void addStarToFrontier();
    void expandCurrentNode();
    void ShowCurrentNeighbour();
    void getNeighbours();

    GraphNode *start_node;
    GraphNode *goal_node;
    GraphNode *current_node;
    GraphNode *new_node ;

    vector<GraphNode*> neighbours;
    vector<GraphNode*> open_set;
    vector<GraphNode*> closed_set;

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void StartEndGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void removeCurrGoal();
    void eraseFromOpenSet(GraphNode *node);
    bool isInClosedSet(GraphNode *node);
    bool isNotInOpenSet(GraphNode *node);
    double distBetween(GraphNode *node1,GraphNode *node2);
};



}
