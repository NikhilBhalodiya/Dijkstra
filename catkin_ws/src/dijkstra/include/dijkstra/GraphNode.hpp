#include <ros/ros.h>
#include <list>
#include <math.h>

using namespace  std;

namespace DJ
{
static int COUNT = 0;

class GraphNode
{
protected:
    int node_id;
    static int COUNT;

    GraphNode *parent_node = nullptr;
public:
    int grid_x;
    int grid_y;

    double g_cost = 0;
    GraphNode(const int &x_,const int &y_):grid_x(x_),grid_y(y_)
    {
      DJ::COUNT++;
      node_id = DJ::COUNT;
    }

    ~GraphNode()
    {
        delete parent_node;
    }

    void updateParent(GraphNode &parent)
    {
        parent_node = &parent;
        g_cost = parent_node->g_cost + sqrt(pow(parent_node->grid_x - grid_x, 2) +
                                                  pow(parent_node->grid_y - grid_y, 2)*1.0);

    }
};

struct LessThanByCost
{
  bool operator()(const GraphNode *lhs, const GraphNode *rhs) const
  {
    return lhs->g_cost < rhs->g_cost;
  }
};

}

