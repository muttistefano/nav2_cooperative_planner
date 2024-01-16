#include "AStar/AStar.hpp"
#include <algorithm>
#include <math.h>

using namespace std::placeholders;

bool AStar::Vec2i::operator == (const Vec2i& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y && z == coordinates_.z);
}

AStar::Vec2i operator + (const AStar::Vec2i& left_, const AStar::Vec2i& right_)
{
    return{ left_.x + right_.x, left_.y + right_.y,left_.z + right_.z };
}

AStar::Node::Node(Vec2i coordinates_, std::shared_ptr<Node> parent_)
{
    parent = parent_;
    coordinates = coordinates_;
    G = 0;
    H = 0;
}

AStar::uint AStar::Node::getScore() const
{
    return G + H;
}

void AStar::Generator::reset(){};

AStar::Generator::Generator()
{
    setHeuristic(&Heuristic::manhattan);
    direction = {
        { 1, 1, 0 }, { 1, 0, 0 }, { 1,-1, 0 }, { 0,-1, 0 },{-1,-1, 0 }, {-1, 0, 0 }, {-1, 1, 0 }, { 0, 1, 0 },
        { 1, 1, 1 }, { 1, 0, 1 }, { 1,-1, 1 }, { 0,-1, 1 },{-1,-1, 1 }, {-1, 0, 1 }, {-1, 1, 1 }, { 0, 1, 1 }, { 0, 0, 1 },
        { 1, 1,-1 }, { 1, 0,-1 }, { 1,-1,-1 }, { 0,-1,-1 },{-1,-1,-1 }, {-1, 0,-1 }, {-1, 1,-1 }, { 0, 1,-1 }, { 0, 0,-1 }
    };
    direction_costs = {
        14,10,14,10,14,10,14,10,
        14,10,14,10,14,10,14,10,10,
        14,10,14,10,14,10,14,10,10
    };
    // direction_costs = {
    //     14,10,14,10,14,10,14,10,
    //     17,14,17,14,17,14,17,14,10,
    //     17,14,17,14,17,14,17,14,10
    // };
}

void AStar::Generator::setWorldSize(Vec2i worldSize_)
{
    worldSize = worldSize_;
    //TODO is ok ?
    auto maxcoll = static_cast<std::size_t>(worldSize_.x * worldSize_.y * worldSize_.z);
    walls.reserve(maxcoll);
}

AStar::Vec2i AStar::Generator::getWorldSize()
{
    return worldSize;
}

void AStar::Generator::setHeuristic(HeuristicFunction heuristic_)
{
    heuristic = std::bind(heuristic_, _1, _2);
}

void AStar::Generator::removeCollision(Vec2i coordinates_)
{
    auto it = std::find(walls.begin(), walls.end(), coordinates_);
    if (it != walls.end()) {
        walls.erase(it);
    }
}

void AStar::Generator::clearCollisions()
{
    walls.clear();
}

AStar::CoordinateList AStar::Generator::findPath()
{
    std::shared_ptr<Node> current = nullptr;
    NodeSet openSet, closedSet;
    // openSet.reserve(10000);
    // closedSet.reserve(10000);
    openSet.push_back(std::make_shared<Node>(this->source_pt));

    while (!openSet.empty()) {
        auto current_it = openSet.begin();
        current = *current_it;

        for (auto it = openSet.begin(); it != openSet.end(); it++) {
            auto node = *it;
            if (node->getScore() <= current->getScore()) {
                current = node;
                current_it = it;
            }
        }

        if (current->coordinates == goal_pt) {
            break;
        }

        closedSet.push_back(current);
        openSet.erase(current_it);

        for (uint i = 0; i < 27; i++) {
            Vec2i newCoordinates(current->coordinates + direction[i]);
            if (detectCollision(newCoordinates) ||
                findNodeOnList(closedSet, newCoordinates)) {
                // std::cout << "cll " << i << " " << std::flush;

                continue;
            }
            
            uint totalCost = current->G + direction_costs[i];

            std::shared_ptr<Node> successor = findNodeOnList(openSet, newCoordinates);
            if (successor == nullptr) {
                successor = std::make_shared<Node>(newCoordinates, current);
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, goal_pt);
                openSet.push_back(successor);
            }
            else if (totalCost < successor->G) {
                successor->parent = current;
                successor->G = totalCost;
            }
        }
    }

    CoordinateList path;
    while (current != nullptr) {
        path.push_back(current->coordinates);
        current = current->parent;
    }

    releaseNodes(openSet);
    releaseNodes(closedSet);

    return path;
}

std::shared_ptr<AStar::Node> AStar::Generator::findNodeOnList(NodeSet& nodes_, Vec2i coordinates_)
{
    for (auto node : nodes_) {
        if (node->coordinates == coordinates_) {
            return node;
        }
    }
    return nullptr;
}

void AStar::Generator::releaseNodes(NodeSet& nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        // delete *it;
        it = nodes_.erase(it);
    }
}

bool AStar::Generator::detectCollision(Vec2i coordinates_)
{
    if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
        coordinates_.y < 0 || coordinates_.y >= worldSize.y ||
        coordinates_.z < 0 || coordinates_.z >= worldSize.z ||
        std::find(walls.begin(), walls.end(), coordinates_) != walls.end()) {
        return true;
    }
    return false;
}

AStar::Vec2i AStar::Heuristic::getDelta(Vec2i source_, Vec2i target_)
{
    return{ abs(source_.x - target_.x),  abs(source_.y - target_.y), abs(source_.z - target_.z) };
}

AStar::uint AStar::Heuristic::euclidean(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    //TODO fix
    return static_cast<uint>(14 * sqrt(pow(delta.x, 2) + pow(delta.y, 2) + pow(delta.z, 2)));
}

AStar::uint AStar::Heuristic::octagonal(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
    //TODO fix this
}

AStar::uint AStar::Heuristic::manhattan(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(14 * (delta.x + delta.y + delta.z));
}

void AStar::Generator::setStart(Vec2i source_)
{
    this->source_pt = source_;
}

void AStar::Generator::setGoal(Vec2i target_)
{
    this->goal_pt = target_;
}

bool AStar::Generator::setCostmap(std::vector<int> costmap)
{
    for(int z=0;z<this->worldSize.z;z++)
    {
        for(int x=0;x<this->worldSize.x;x++)
        {
            for(int y=0;y<this->worldSize.y;y++)
            {
                if(costmap[z * (this->worldSize.x * this->worldSize.y) + y * this->worldSize.x + x] > 10)
                {
                    this->walls.push_back({x,y,z});
                }
            }
        }
    }
    return true;
}

