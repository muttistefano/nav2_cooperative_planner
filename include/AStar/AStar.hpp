/*
    Copyright (c) 2015, Damian Barczynski <daan.net@wp.eu>
    Following tool is licensed under the terms and conditions of the ISC license.
    For more information visit https://opensource.org/licenses/ISC.
*/
#ifndef __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
#define __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__

#include <vector>
#include <functional>
#include <set>
#include <iostream>
#include <chrono>
#include <memory>



namespace AStar
{
    struct Vec2i
    {
        int x, y, z;

        bool operator == (const Vec2i& coordinates_);
        friend std::ostream& operator << (std::ostream& os, const Vec2i& coordinates_)
        {   
            os << coordinates_.x << ' ' << coordinates_.y << ' ' << coordinates_.z;
            return os;
        };
    };

    using uint = unsigned int;
    using HeuristicFunction = std::function<uint(Vec2i, Vec2i)>;
    using CoordinateList = std::vector<Vec2i>;

    struct Node
    {
        uint G, H;
        Vec2i coordinates;
        std::shared_ptr<Node> parent;

        Node(Vec2i coord_, std::shared_ptr<Node> parent_ = nullptr);
        uint getScore() const;
    };

    using NodeSet = std::vector<std::shared_ptr<Node>>;

    class Generator
    {
        bool detectCollision(Vec2i coordinates_);
        std::shared_ptr<Node> findNodeOnList(NodeSet& nodes_, Vec2i coordinates_);
        void releaseNodes(NodeSet& nodes_);

    public:
        Generator();
        void setWorldSize(Vec2i worldSize_);
        Vec2i getWorldSize();
        void setHeuristic(HeuristicFunction heuristic_);
        CoordinateList findPath();
        void setStart(Vec2i source_);
        Vec2i getStart() const {return source_pt;};
        void setGoal(Vec2i target_);
        Vec2i getGoal() const {return goal_pt;};
        void removeCollision(Vec2i coordinates_);
        void clearCollisions();
        void reset();
        bool setCostmap(std::vector<int> costmap);

    private:
        HeuristicFunction heuristic;
        CoordinateList direction;
        CoordinateList walls;
        Vec2i worldSize;
        std::vector<uint> direction_costs;
        Vec2i source_pt;
        Vec2i goal_pt;


    };

    class Heuristic
    {
        static Vec2i getDelta(Vec2i source_, Vec2i target_);

    public:
        static uint euclidean(Vec2i source_, Vec2i target_);
        static uint octagonal(Vec2i source_, Vec2i target_);
        static uint manhattan(Vec2i source_, Vec2i target_);
    };
}

#endif // __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
