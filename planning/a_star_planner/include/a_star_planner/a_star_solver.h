#ifndef A_STAR_SOLVER_H
#define A_STAR_SOLVER_H

#include <vector>
#include <nav_msgs/OccupancyGrid.h>

class AStarSolver
{
    public:
        
        struct Vector2Int
        {
            bool operator == (const Vector2Int& v);
            Vector2Int operator + (const Vector2Int& v);
            int x, y;
        };

        AStarSolver();
        void SetMap(nav_msgs::OccupancyGridConstPtr map);
        void SetSource(Vector2Int coordinates);
        void SetTarget(Vector2Int coordinates);
        bool Solve();
        bool Solve(Vector2Int source, Vector2Int target);
    
    private:
        
        struct Node
        {
            Node(Vector2Int coordinates, Node *parent = nullptr);
            unsigned int getScore();

            unsigned int G, H;
            Vector2Int coordinates;
            Node* parent;
        };

        nav_msgs::OccupancyGrid _map;
        Vector2Int _source;
        Vector2Int _target;

        const std::vector<Vector2Int> _directions = {
                                                        {-1, 1}, {0, 1}, {1, 1}, 
                                                        {-1, 0}, {0, 0}, {1, 0},
                                                        {-1, -1}, {0, -1}, {1, -1}
                                                    };
};

#endif