#include "a_star_planner/a_star_solver.h"

/* Vector2Int */

bool AStarSolver::Vector2Int::operator==(const Vector2Int& v)
{
    return (this->x == v.x && this->y == v.y);
}

AStarSolver::Vector2Int AStarSolver::Vector2Int::operator+(const Vector2Int& v)
{
    return {this->x + v.x, this->y + v.y};
}

/* Node */

AStarSolver::Node::Node(Vector2Int coordinates, Node* parent)
{
    this->parent = parent;
    this->coordinates = coordinates;
    this->G = 0;
    this->H = 0;
}

unsigned int AStarSolver::Node::getScore()
{
    return G + H;
}

/* A* Solver */

AStarSolver::AStarSolver()
{

}

void AStarSolver::SetMap(nav_msgs::OccupancyGridConstPtr map)
{
    _map = *map;
}

void AStarSolver::SetSource(Vector2Int coordinates)
{
    _source = coordinates;
}

void AStarSolver::SetTarget(Vector2Int coordinates)
{
    _target = coordinates;
}

bool AStarSolver::Solve(int cost_threshold)
{
    int mapIndex = coordinates2mapIndex(_source);
    if(_map.data[mapIndex] > cost_threshold) return false;
    mapIndex = coordinates2mapIndex(_target);
    if(_map.data[mapIndex] > cost_threshold) return false;
    
    Node *node_current = nullptr;
    std::vector<Node*> nodes_opened;
    std::vector<Node*> nodes_closed;

    nodes_opened.push_back(new Node(_source));

    bool solved = false;

    while(!nodes_opened.empty())
    {
        auto it_current = nodes_opened.begin();
        node_current = *it_current;

        for(auto it_node = nodes_opened.begin(); it_node != nodes_opened.end(); it_node++)
        {
            const auto node = *it_node;
            if(node->getScore() <= node_current->getScore())
            {
                node_current = node;
                it_current = it_node;
            }
        }

        if(node_current->coordinates == _target)
        {
            solved = true;
            break;
        }
        nodes_closed.push_back(node_current);
        nodes_opened.erase(it_current);

        for(int i = 0; i < 8; i++)
        {
            Vector2Int coordinates_new(node_current->coordinates + _directions[i]);
            int mapIndex = coordinates2mapIndex(coordinates_new);
            if(mapIndex == -1 || _map.data[mapIndex] > cost_threshold || findNode(nodes_closed, coordinates_new))
            {
                continue;
            }
            unsigned int cost = node_current->G + (i < 4 ? 10 : 14);
            Node *node_new = findNode(nodes_opened, coordinates_new);
            if(node_new == nullptr)
            {
                node_new = new Node(coordinates_new, node_current);
                node_new->G = cost;
                node_new->H = calcHeuristic(coordinates_new, _target);
                nodes_opened.push_back(node_new);
            }
            else if(cost < node_new->G)
            {
                node_new->parent = node_current;
                node_new->G = cost;
            }
        }
    }

    path.clear();
    while(node_current != nullptr)
    {
        path.push_back(node_current->coordinates);
        node_current = node_current->parent;
    }
    std::reverse(path.begin(), path.end());

    for(auto it = nodes_opened.begin(); it != nodes_opened.end();)
    {
        delete *it;
        it = nodes_opened.erase(it);
    }

    for(auto it = nodes_closed.begin(); it != nodes_closed.end();)
    {
        delete *it;
        it = nodes_closed.erase(it);
    }

    return solved;
}

bool AStarSolver::Solve(int cost_threshold, Vector2Int source, Vector2Int target)
{
    SetSource(source);
    SetTarget(target);
    return Solve(cost_threshold);
}

AStarSolver::Node* AStarSolver::findNode(std::vector<Node*> nodes, Vector2Int coordinates)
{
    for(auto node : nodes)
    {
        if(node->coordinates == coordinates) return node;
    }
    return nullptr;
}

int AStarSolver::coordinates2mapIndex(Vector2Int coordinates)
{
    int x = coordinates.x;
    int y = coordinates.y;
    if(x < 0 || y < 0 || x >= _map.info.width || y >= _map.info.height) return -1;
    return y * _map.info.width + x;
}

unsigned int AStarSolver::calcHeuristic(Vector2Int source, Vector2Int target)
{
    int dx = target.x - source.x;
    int dy = target.y - source.y;
    return static_cast<unsigned int>(10 * sqrt(dx*dx + dy*dy));
}