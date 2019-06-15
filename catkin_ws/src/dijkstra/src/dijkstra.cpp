#include <dijkstra.hpp>

namespace DJ {

Dijkstra::Dijkstra(ros::NodeHandle &nh)
{
    map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map",10,&Dijkstra::mapCallback,this);
    goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal",10,&Dijkstra::StartEndGoalCallback,this);
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map_new",10);
}
Dijkstra::~Dijkstra()
{}


void Dijkstra::createStartNode()
{
    start_node = new GraphNode(start_grid_x,start_grid_y);
    start_node->updateParent(*start_node);
}

void Dijkstra::createGoalNode()
{
    goal_node = new GraphNode(goal_grid_x,goal_grid_y);
}

void Dijkstra::addStarToFrontier()
{
    frontier.push(start_node);
    open_set.push_back(start_node);
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
      createGoalNode();
    }
    drawStartAndGoal();
    addStarToFrontier();
    while(!frontier.empty())
    {
        current_node = frontier.top();
        frontier.pop();

        ROS_INFO("current node %d %d",current_node->grid_x,current_node->grid_y);   ;
        eraseFromOpenSet(current_node);
        closed_set.push_back(current_node);
//      expandCurrentNode();

        getNeighbours();
        for(auto neighbor: neighbours)
        {
//            ROS_INFO("neighbour %d %d",neighbor->grid_x,neighbor->grid_y);
            if(isInClosedSet(neighbor))
            {
                continue;
            }
            double tent_g_cost = current_node->g_cost + distBetween(current_node,neighbor);

//            if(isNotInOpenSet(&neighbor))
//            {
//                frontier.push(&neighbor);
//                open_set.push_back(neighbor);
//            }
//            else if (tent_g_cost >= neighbor.g_cost)
//            {
//                continue;
//            }
//            neighbor.g_cost = tent_g_cost;
        }

    }

}

//bool Dijkstra::isNotInOpenSet(GraphNode *node)
//{
//    auto x = find(closed_set.begin(),closed_set.end(),node);
//    if(x != open_set.end())
//    {
//       return false;
//    }
//    return true;
//}

double Dijkstra::distBetween(GraphNode *node1,GraphNode *node2)
{
    return sqrt(pow(node1->grid_x - node2->grid_x, 2) +
                pow(node1->grid_y - node2->grid_y, 2)*1.0);


}

void Dijkstra::eraseFromOpenSet(GraphNode *node)
{
    ROS_INFO(" node delete ");
    auto x = find(open_set.begin(),open_set.end(),node);
    if(x != open_set.end())
    {
        open_set.erase(x);
    }
}

bool Dijkstra::isInClosedSet(GraphNode *node)
{
    auto x = find(closed_set.begin(),closed_set.end(),node);
    if(x != open_set.end())
    {
       return true;
    }
    return false;
}

void Dijkstra::getNeighbours()
{
    int step_length = 3;  //in pixels chosen by Try and Error. Generally it depends of Velocity and dimention of the bot
    int prev_grid_x,prev_grid_y,new_grid_x,new_grid_y;
    neighbours.clear();
    for(int theta = 0; theta<360; theta+=5)
    {
        new_grid_x = current_node->grid_x + step_length*cos(theta);
        new_grid_y = current_node->grid_y + step_length*sin(theta);

        if(new_grid_x == prev_grid_x && new_grid_y == prev_grid_y )
        {
            continue;
        }
        else {
            prev_grid_x = new_grid_x;
            prev_grid_y = new_grid_y;
            new_node = new GraphNode(new_grid_x,new_grid_y);
            new_node->updateParent(*current_node);
            neighbours.push_back(new_node);
        }
    }
}

//void Dijkstra::ShowCurrentNeighbour()
//{
//    while(!frontier.empty())
//    {
//        GraphNode *temp = frontier.top();
//        updated_map.data[map->info.width*temp->grid_y + temp->grid_x] = 100;
//        frontier.pop();
//    }
//    map_pub.publish(updated_map);

//}
//void Dijkstra::expandCurrentNode()
//{
//    int step_length = 3;  //in pixels chosen by Try and Error. Generally it depends of Velocity and dimention of the bot
//    int prev_grid_x,prev_grid_y,new_grid_x,new_grid_y;
//    for(int theta = 0; theta<360; theta+=5)
//    {
//        new_grid_x = current_node->grid_x + step_length*cos(theta);
//        new_grid_y = current_node->grid_y + step_length*sin(theta);

//        if(new_grid_x == prev_grid_x && new_grid_y == prev_grid_y )
//        {
//            continue;
//        }
//        else {
//            prev_grid_x = new_grid_x;
//            prev_grid_y = new_grid_y;
//            new_node = new GraphNode(new_grid_x,new_grid_y);
//            frontier.push(new_node);
//        }
//    }
////    ShowCurrentNeighbour();   // uncomment it just to check if this function works properly

//}


void Dijkstra::drawStartAndGoal()
{
    updated_map.data[map->info.width*start_grid_y + start_grid_x] = 100;
    updated_map.data[map->info.width*goal_grid_y + goal_grid_x] = 100;
    map_pub.publish(updated_map);
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
