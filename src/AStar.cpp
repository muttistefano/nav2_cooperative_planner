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
    setHeuristic(&Heuristic::octagonal);
    direction = {{ 1, 1, 0 }, { 1, 0, 0 }, { 1,-1, 0 }, { 0,-1, 0 },{-1,-1, 0 }, {-1, 0, 0 }, {-1, 1, 0 }, { 0, 1, 0 }, { 0, 0, 1 },{ 0, 0, -1 }};
    direction_costs = {14,10,14,10,14,10,14,10,2,2};
}

void AStar::Generator::setWorldSize(Vec2i worldSize_)
{
    worldSize = worldSize_;
}

AStar::Vec2i AStar::Generator::getWorldSize()
{
    return worldSize;
}

void AStar::Generator::setHeuristic(HeuristicFunction heuristic_)
{
    heuristic = std::bind(heuristic_, _1, _2);
}


AStar::CoordinateList AStar::Generator::findPath()
{
    std::shared_ptr<Node> current = nullptr;
    NodeSet openSet, closedSet;
    openSet.reserve(10000);
    closedSet.reserve(10000);
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
        // RCLCPP_INFO(rclcpp::get_logger("nav2_coop"), "\n\n\n\n\n");
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("nav2_coop"), openSet.size());
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("nav2_coop"), "node -> " << current->coordinates.x << " " << current->coordinates.y << " " << current->coordinates.z);

        if (current->coordinates == goal_pt) {
            break;
        }

        closedSet.push_back(current);
        openSet.erase(current_it);

        
        // #pragma omp parallel for
        for (uint i = 0; i < 10; i++) {
            Vec2i newCoordinates(current->coordinates + direction[i]);
            auto start_time2 = high_resolution_clock::now();
            if (detectCollision(newCoordinates) || findNodeOnList(closedSet, newCoordinates)) {
                continue;
            }
            auto stop_time2 = high_resolution_clock::now();
            auto duration2 = duration_cast<microseconds>(stop_time2 - start_time2);

            uint totalCost = current->G + direction_costs[i] 
            + (*costmap_a_)[current->coordinates.z * (this->worldSize.x * this->worldSize.y) + current->coordinates.y * this->worldSize.x + current->coordinates.x];

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

    if (current->coordinates == this->goal_pt)
    {
        while (current != nullptr) {
            path.push_back(current->coordinates);
            current = current->parent;
        }
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
        coordinates_.z < 0 || coordinates_.z >= worldSize.z) 
        {
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
    return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2) + pow(delta.z, 2)));
}

AStar::uint AStar::Heuristic::octagonal(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return 10 * (delta.x + delta.y + delta.z) + (-6) * std::min({delta.x, delta.y});
    //TODO fix this
}

AStar::uint AStar::Heuristic::manhattan(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * (delta.x + delta.y));
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

    this->costmap_a_ = std::make_shared<std::vector<int>>(costmap);
    return true;
}

