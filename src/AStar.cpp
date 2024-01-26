#include "AStar/AStar.hpp"
#include <algorithm>
#include <math.h>

using namespace std::placeholders;

bool AStar::Vec2i::operator == (const Vec2i& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y);
}

AStar::Vec2i operator + (const AStar::Vec2i& left_, const AStar::Vec2i& right_)
{
    return{ left_.x + right_.x, left_.y + right_.y};
}

AStar::ANode::ANode(Vec2i coordinates_, std::shared_ptr<ANode> parent_)
{
    parent = parent_;
    coordinates = coordinates_;
    G = 0;
    H = 0;
}

AStar::uint AStar::ANode::getScore() const
{
    return G + H;
}

void AStar::Generator::reset(){};

AStar::Generator::Generator(): Node("A_star")
{
    setHeuristic(&Heuristic::euclidean);
    direction = {
        { 1, 1}, { 1, 0}, { 1,-1}, { 0,-1},{-1,-1}, {-1, 0}, {-1, 1}, { 0, 1}};
    direction_costs = {14,10,14,10,14,10,14,10};
    rclcpp::Clock ros_clock(RCL_ROS_TIME);
    publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("point", 1);
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
    std::shared_ptr<ANode> current = nullptr;
    NodeSet openSet, closedSet;
    // openSet.reserve(10000);
    // closedSet.reserve(10000);
    openSet.push_back(std::make_shared<ANode>(this->source_pt));

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

        // RCLCPP_INFO_STREAM(rclcpp::get_logger("nav2_coop"), "node -> " << current->coordinates.x << " " << current->coordinates.y);
        // geometry_msgs::msg::PointStamped mmsg;
        // mmsg.header.frame_id = "map";
        // mmsg.header.stamp = this->get_clock()->now();
        // mmsg.point.x = current->coordinates.x * 0.05;
        // mmsg.point.y = current->coordinates.y * 0.05;
        // this->publisher_->publish(mmsg);

        if (current->coordinates == goal_pt) {
            break;
        }

        closedSet.push_back(current);
        openSet.erase(current_it);

        for (uint i = 0; i < 8; i++) {
            Vec2i newCoordinates(current->coordinates + direction[i]);
            if((*costmap_a_)[newCoordinates.y * this->worldSize.x + newCoordinates.x] > 90) continue;
            if (detectCollision(newCoordinates) ||
                findNodeOnList(closedSet, newCoordinates)) {
                // std::cout << "cll " << i << " " << std::flush;

                continue;
            }
            
            uint totalCost = current->G + direction_costs[i]
            + static_cast<uint>((*costmap_a_)[current->coordinates.y * this->worldSize.x + current->coordinates.x]/10);
            // std::cout << "MEGAPD " << direction_costs[i] << "  " << i << "\n" << std::flush;
            // std::cout << totalCost << "\n";

            std::shared_ptr<ANode> successor = findNodeOnList(openSet, newCoordinates);
            if (successor == nullptr) {
                successor = std::make_shared<ANode>(newCoordinates, current);
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

std::shared_ptr<AStar::ANode> AStar::Generator::findNodeOnList(NodeSet& nodes_, Vec2i coordinates_)
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
        coordinates_.y < 0 || coordinates_.y >= worldSize.y) 
    {
        return true;
    }
    return false;
}

AStar::Vec2i AStar::Heuristic::getDelta(Vec2i source_, Vec2i target_)
{
    return{ abs(source_.x - target_.x),  abs(source_.y - target_.y)};
}

AStar::uint AStar::Heuristic::euclidean(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    //TODO fix
    return static_cast<uint>(14 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
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
    return static_cast<uint>(14 * (delta.x + delta.y));
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

