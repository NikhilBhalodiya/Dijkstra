#include <dijkstra.hpp>

namespace DJ {

Dijkstra::Dijkstra(ros::NodeHandle &nh)
{
    map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map",10,&Dijkstra::mapCallback,this);
    goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal",10,&Dijkstra::StartEndGoalCallback,this);
}

void Dijkstra::addStarToQueue()
{
    priority_queue<GraphNode> frontier;

}

void Dijkstra::planPathDijkstra()
{
    drawStartAndGoal();
    addStarToQueue();



}

void Dijkstra::drawStartAndGoal()
{
    updated_map.data[map->info.width*start_grid_y + start_grid_x] = 100;
    updated_map.data[map->info.width*goal_grid_y + goal_grid_x] = 100;
}


void Dijkstra::removeCurrGoal()
{
    updated_map.data[map->info.width*goal_grid_y + goal_grid_x] = 0;
}

void Dijkstra::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    map = msg;
}

void Dijkstra::StartEndGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{

    if(once == false)
    {
    start = msg;
    start_grid_x = (double)start->pose.position.x/map->info.resolution;
    start_grid_y = (double)start->pose.position.y/map->info.resolution;

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
       goal_grid_x = (double)goal->pose.position.x/map->info.resolution;
       goal_grid_y = (double)goal->pose.position.y/map->info.resolution;
       planPathDijkstra();
    }
}


}
