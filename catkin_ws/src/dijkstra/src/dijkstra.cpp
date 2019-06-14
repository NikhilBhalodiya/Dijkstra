#include <dijkstra.hpp>

namespace DJ {

Dijkstra::Dijkstra(ros::NodeHandle &nh)
{
    map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map",10,&Dijkstra::mapCallback,this);
    goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal",10,&Dijkstra::StartEndGoalCallback,this);
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map_new",10);
}
Dijkstra::~Dijkstra()
{

}
void Dijkstra::createGoalNode()
{
    goal_node = new DJ::GraphNode(goal_grid_x,goal_grid_y);
}
void Dijkstra::createStartNode()
{
    start_node = new DJ::GraphNode(start_grid_x,start_grid_y);
    start_node->updateParent(*start_node);
}


void Dijkstra::planPathDijkstra()
{
    if(first_itr)
    {
        createStartNode();
        createGoalNode();
        first_itr = false;
    }
    else
    {
      delete goal_node;
      createGoalNode();
      ROS_INFO("Goal has been updated");
    }
    drawStartAndGoal();
    addStarToFrontier();

    while(!frontier.empty())
    {
        current_node = frontier.top();
        frontier.pop();

        ROS_INFO("first node %d %d %lf",current_node->grid_x,current_node->grid_y,current_node->g_cost);

    }


}

void Dijkstra::drawStartAndGoal()
{
    updated_map.data[map->info.width*start_grid_y + start_grid_x] = 100;
    updated_map.data[map->info.width*goal_grid_y + goal_grid_x] = 100;
    map_pub.publish(updated_map);
}

void Dijkstra::addStarToFrontier()
{
    frontier.push(start_node);
}

void Dijkstra::removeCurrGoal()
{
updated_map.data[map->info.width*goal_grid_y + goal_grid_x] = 0;
map_pub.publish(updated_map);
}

void Dijkstra::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    map = msg;
    updated_map = *map;
    ROS_INFO("map has been imported");
}

void Dijkstra::StartEndGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    int map_rviz_offset = 200;  //calculated by try and error
    if(once == false)
    {
    start = msg;
    start_grid_x = int(start->pose.position.x/map->info.resolution)+ map_rviz_offset;
    start_grid_y = int(start->pose.position.y/map->info.resolution)+ map_rviz_offset;
    ROS_INFO("Source position has been taken");
    once = true;
    }
    else
    {
       if(m_have_goal == true)
       {
        removeCurrGoal();
       }
       goal = msg;
       m_have_goal = true;
       goal_grid_x = int(goal->pose.position.x/map->info.resolution) + map_rviz_offset;
       goal_grid_y = int(goal->pose.position.y/map->info.resolution) + map_rviz_offset;

       planPathDijkstra();
    }
}


}
