// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2018 Simbe Robotics
// Copyright (c) 2019 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Navigation Strategy based on:
// Brock, O. and Oussama K. (1999). High-Speed Navigation Using
// the Global Dynamic Window Approach. IEEE.
// https://cs.stanford.edu/group/manips/publications/pdfs/Brock_1999_ICRA.pdf

// #define BENCHMARK_TESTING

#include "nav2_cooperative_planner/cooperative_planner.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;  // NOLINT
using nav2_util::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

namespace nav2_cooperative_planner
{

CoopPlanner::CoopPlanner()
: tf_(nullptr), costmap_(nullptr)
{
}

CoopPlanner::~CoopPlanner()
{
  RCLCPP_INFO(
    logger_, "Destroying plugin %s of type CoopPlanner",
    name_.c_str());
}

void
CoopPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  tf_ = tf;
  name_ = name;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  node_ = parent;
  auto node = parent.lock();
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  RCLCPP_INFO(
    logger_, "Configuring plugin %s of type CoopPlanner",
    name_.c_str());

  // Initialize parameters
  // Declare this plugin's parameters
  declare_parameter_if_not_declared(node, name + ".tolerance", rclcpp::ParameterValue(0.5));
  node->get_parameter(name + ".tolerance", tolerance_);
  declare_parameter_if_not_declared(node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".allow_unknown", allow_unknown_);
    declare_parameter_if_not_declared(node, name + ".transportation", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".transportation", transportation_);
  declare_parameter_if_not_declared(
    node, name + ".use_final_approach_orientation", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".use_final_approach_orientation", use_final_approach_orientation_);
  declare_parameter_if_not_declared(node, name + ".follower_costmap", rclcpp::ParameterValue(""));
  node->get_parameter(name + ".follower_costmap", follower_costmap_);
    declare_parameter_if_not_declared(node, name + ".follower_frame", rclcpp::ParameterValue(""));
  node->get_parameter(name + ".follower_frame", follower_frame_);
      declare_parameter_if_not_declared(node, name + ".leader_frame", rclcpp::ParameterValue(""));
  node->get_parameter(name + ".leader_frame", leader_frame_);
  

  costmap_follower_ = costmap_ros->getCostmap();

  costmap_follower_subscription_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      follower_costmap_, 1, std::bind(&CoopPlanner::costmap_follower_call, this, _1));

  map_publisher_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>("merged", 1);

  _tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

  // Create a planner based on the new costmap size
  planner_ = std::make_unique<AStar::Generator>();
}

void CoopPlanner::costmap_follower_call(const nav_msgs::msg::OccupancyGrid & msg)
{
  this->costmap_follower_ = new nav2_costmap_2d::Costmap2D(msg);
}

void
CoopPlanner::activate()
{
  RCLCPP_INFO(
    logger_, "Activating plugin %s of type CoopPlanner",
    name_.c_str());
  // Add callback for dynamic parameters
  auto node = node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&CoopPlanner::dynamicParametersCallback, this, _1));
}

void
CoopPlanner::deactivate()
{
  RCLCPP_INFO(
    logger_, "Deactivating plugin %s of type CoopPlanner",
    name_.c_str());
  dyn_params_handler_.reset();
}

void
CoopPlanner::cleanup()
{
  RCLCPP_INFO(
    logger_, "Cleaning up plugin %s of type CoopPlanner",
    name_.c_str());
  planner_.reset();
}

nav_msgs::msg::Path CoopPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{

  unsigned int mx_start, my_start, mx_goal, my_goal;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start)) {
    throw nav2_core::StartOutsideMapBounds(
            "Start Coordinates of(" + std::to_string(start.pose.position.x) + ", " +
            std::to_string(start.pose.position.y) + ") was outside bounds");
  }

  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal)) {
    throw nav2_core::GoalOutsideMapBounds(
            "Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
            std::to_string(goal.pose.position.y) + ") was outside bounds");
  }

  if (costmap_->getCost(mx_start, my_start) == nav2_costmap_2d::LETHAL_OBSTACLE) {
    throw nav2_core::StartOccupied(
            "Start Coordinates of(" + std::to_string(start.pose.position.x) + ", " +
            std::to_string(start.pose.position.y) + ") was in lethal cost");
  }

  if (tolerance_ == 0 && costmap_->getCost(mx_goal, my_goal) == nav2_costmap_2d::LETHAL_OBSTACLE) {
    throw nav2_core::GoalOccupied(
            "Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
            std::to_string(goal.pose.position.y) + ") was in lethal cost");
  }

  nav_msgs::msg::Path path;

  // Corner case of the start(x,y) = goal(x,y)
  if (start.pose.position.x == goal.pose.position.x &&
    start.pose.position.y == goal.pose.position.y)
  {
    path.header.stamp = clock_->now();
    path.header.frame_id = global_frame_;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.z = 0.0;

    pose.pose = start.pose;
    // if we have a different start and goal orientation, set the unique path pose to the goal
    // orientation, unless use_final_approach_orientation=true where we need it to be the start
    // orientation to avoid movement from the local planner
    if (start.pose.orientation != goal.pose.orientation && !use_final_approach_orientation_) {
      pose.pose.orientation = goal.pose.orientation;
    }
    path.poses.push_back(pose);
    return path;
  }

  char * cost_translation_table_ = new char[256];

  // special values:
  cost_translation_table_[0] = 0;  // NO obstacle
  cost_translation_table_[253] = 99;  // INSCRIBED obstacle
  cost_translation_table_[254] = 100;  // LETHAL obstacle
  cost_translation_table_[255] = -1;  // UNKNOWN

  // regular cost values scale the range 1 to 252 (inclusive) to fit
  // into 1 to 98 (inclusive).
  for (int i = 1; i < 253; i++) {
    cost_translation_table_[i] = static_cast<char>(1 + (97 * (i - 1)) / 251);
  }

  try {
      _tf_map_foll_ = _tf_buffer->lookupTransform(
       global_frame_,follower_frame_,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
      this->logger_, "Could not transform %s to %s: %s",
      global_frame_.c_str(), follower_frame_.c_str(), ex.what());
  }

  try {
      _tf_map_lead_ = _tf_buffer->lookupTransform(
      global_frame_,leader_frame_,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
      this->logger_, "Could not transform %s to %s: %s",
      global_frame_.c_str(), leader_frame_.c_str(), ex.what());
  }

  tf2::Stamped<tf2::Transform> map_to_foll,map_to_lead;

  tf2::fromMsg(_tf_map_foll_, map_to_foll);
  tf2::fromMsg(_tf_map_lead_, map_to_lead);


  //Get map size
  int lead_x = costmap_->getSizeInCellsX(); 
  int lead_y = costmap_->getSizeInCellsY();
  int foll_x = costmap_follower_->getSizeInCellsX(); 
  int foll_y = costmap_follower_->getSizeInCellsY();

  // Update planner based on the new costmap size
  if (isPlannerOutOfDate()) {
    planner_->setWorldSize({
      static_cast<int>(costmap_->getSizeInCellsX()),
      static_cast<int>(costmap_->getSizeInCellsY())});
      RCLCPP_WARN_STREAM(this->logger_,"Planner out of date \n");

  }

  RCLCPP_INFO_STREAM(this->logger_,"Bounds " << lead_x << " "<< lead_y << " "<< foll_x << " "<< foll_y << "\n");

  RCLCPP_INFO_STREAM(this->logger_,"leader frame " << leader_frame_);
  RCLCPP_INFO_STREAM(this->logger_,"follower frame " << follower_frame_);

  //TODO Up to now they are the same costmap cause we don't publish the second one
  const unsigned char * lead_map =  costmap_->getCharMap();
  const unsigned char * foll_map =  costmap_follower_->getCharMap();

  auto start_time = high_resolution_clock::now();

  merged_map.resize(static_cast<std::size_t>(lead_x*lead_y));
  std::fill(merged_map.begin(), merged_map.end(), 0);
  
  RCLCPP_INFO_STREAM(this->logger_,"Merged map size ->  " << merged_map.size() << ":"<< lead_x*lead_y << "\n");

  double resolution = costmap_->getResolution();
  auto shift_x = int((map_to_lead.getOrigin().x() - map_to_foll.getOrigin().x())/resolution);
  auto shift_y = int((map_to_lead.getOrigin().y() - map_to_foll.getOrigin().y())/resolution);

  int xxyylen = std::max(std::abs(shift_x),std::abs(shift_y));
  std::vector<int> x_vec;
  x_vec.reserve(xxyylen);
  std::vector<int> y_vec;
  y_vec.reserve(xxyylen);

  for(int pd = 0 ; pd < xxyylen ; pd++)
  {
    x_vec.push_back(static_cast<int>( (shift_x/xxyylen) * pd ));
    y_vec.push_back(static_cast<int>( (shift_y/xxyylen) * pd ));
  }


  if(this->transportation_)
  {
    for(int x=0;x<lead_x;x++)
    {
      for(int y=0;y<lead_y;y++)
      {
        int idx         = y * lead_x + x;
        merged_map[idx] = static_cast<int>(lead_map[idx]/2);

        std::vector<int> vals ;
        for(int pd = 0 ; pd < xxyylen ; pd++)
        {
          int idx_2_xx     = ((x-x_vec[pd]) > 0) && ((x-x_vec[pd]) < lead_x) ?  (x-x_vec[pd])  : 0 ;
          int idx_2_yy     = ((y-y_vec[pd]) > 0) && ((y-y_vec[pd]) < lead_y) ?  (y-y_vec[pd])  : 0 ;
          int idx_22       = idx_2_yy * lead_x + idx_2_xx;
          vals.push_back(foll_map[idx_22]);

        }

        // int idx_2 = idx_2_y * lead_x + idx_2_x;
        // merged_map[idx] = std::max(merged_map[idx],(foll_map[idx_2]/2));
        merged_map[idx] = *std::max_element(vals.begin(), vals.end());

      }
    }
  }
  else
  {
    for(int x=0;x<lead_x;x++)
    {
      for(int y=0;y<lead_y;y++)
      {
        int idx = y * lead_x + x;
        merged_map[idx] = static_cast<int>(lead_map[idx]);
        int idx_2_x = ((x-shift_x) > 0) && ((x-shift_x) < lead_x) ?  (x-shift_x)  : 0 ;
        int idx_2_y = ((y-shift_y) > 0) && ((y-shift_y) < lead_y) ?  (y-shift_y)  : 0 ;
        // //TODO fix is not 0
        if ((idx_2_x==0) || (idx_2_y==0))
        {
          continue;
        }

        int idx_2 = idx_2_y * lead_x + idx_2_x;
        merged_map[idx] = std::max(merged_map[idx],(static_cast<int>(foll_map[idx_2])));

      }
    }
  }

  costmap_raw_ = std::make_unique<nav_msgs::msg::OccupancyGrid>();
  costmap_raw_->info.resolution = (float)resolution;
  costmap_raw_->info.width = lead_x;
  costmap_raw_->info.height = lead_y;
  costmap_raw_->info.origin.position.x = costmap_->getOriginX();
  costmap_raw_->info.origin.position.y = costmap_->getOriginY();
  costmap_raw_->header.frame_id = global_frame_;

  costmap_raw_->data.resize(lead_x*lead_y);
  for(int ix = 0;ix <lead_x*lead_y;ix++)
  {
    costmap_raw_->data.at(ix) = (cost_translation_table_[merged_map[ix]]);
  }
  
  map_publisher_->publish(*costmap_raw_);

  auto stop_time = high_resolution_clock::now();
  auto duration = duration_cast<microseconds>(stop_time - start_time);
  RCLCPP_INFO_STREAM(this->logger_ ,"Time taken to merge maps: " << duration.count());

  if (!makePlan(start.pose, goal.pose, tolerance_, path)) {
    throw nav2_core::NoValidPathCouldBeFound(
            "Failed to create plan with tolerance of: " + std::to_string(tolerance_) );
  }

  RCLCPP_INFO_STREAM(this->logger_,"plan lenght : " << path.poses.size());
  RCLCPP_INFO_STREAM(this->logger_,"returning \n \n \n \n \n \n ");

  return path;
}

bool
CoopPlanner::isPlannerOutOfDate()
{
  if (!planner_.get() ||
    planner_->getWorldSize().x != (int)costmap_->getSizeInCellsX() ||
    planner_->getWorldSize().y != (int)costmap_->getSizeInCellsY())
  {
    return true;
  }
  return false;
}

bool
CoopPlanner::makePlan(
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal, double tolerance,
  nav_msgs::msg::Path & plan)
{
  // clear the plan, just in case
  RCLCPP_INFO_STREAM(this->logger_,"making plan");
  plan.poses.clear();

  plan.header.stamp = clock_->now();
  plan.header.frame_id = global_frame_;

  double wx = start.position.x;
  double wy = start.position.y;

  RCLCPP_DEBUG(
    logger_, "Making plan from (%.2f,%.2f) to (%.2f,%.2f)",
    start.position.x, start.position.y, goal.position.x, goal.position.y);

  unsigned int mx, my;
  worldToMap(wx, wy, mx, my);
  // clear the starting cell within the costmap because we know it can't be an obstacle
  clearRobotCell(mx, my);

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  planner_->setCostmap(merged_map);

  lock.unlock();

  planner_->setStart({(int)mx,(int)my});
  RCLCPP_INFO_STREAM(this->logger_,"plan start : " << (int)mx << " " << (int)my);

  wx = goal.position.x;
  wy = goal.position.y;

  worldToMap(wx, wy, mx, my);
  planner_->setGoal({(int)mx,(int)my});
  RCLCPP_INFO_STREAM(this->logger_,"plan goal : " << (int)mx << " " << (int)my);
  
  auto saa = high_resolution_clock::now();
  AStar::CoordinateList plan_out = planner_->findPath();
  auto sae = high_resolution_clock::now();
  auto drr = duration_cast<microseconds>(sae - saa);

  RCLCPP_INFO_STREAM(this->logger_ ,"Time taken to plan: " << drr.count());

  //TODO remove or use tolerance
  double pd = 0.5 *tolerance ;
  tolerance = pd;

  int len = plan_out.size();

  for (int i = len - 1; i >= 0; --i) {
    // convert the plan to world coordinates
    double world_x, world_y;
    mapToWorld(plan_out[i].x, plan_out[i].y, world_x, world_y);

    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = world_x;
    pose.pose.position.y = world_y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.poses.push_back(pose);
  }

  if (use_final_approach_orientation_) {
    size_t plan_size = plan.poses.size();
    if (plan_size == 1) {
      plan.poses.back().pose.orientation = start.orientation;
    } else if (plan_size > 1) {
      double dx, dy, theta;
      auto last_pose = plan.poses.back().pose.position;
      auto approach_pose = plan.poses[plan_size - 2].pose.position;
      // Deal with the case of NavFn producing a path with two equal last poses
      if (std::abs(last_pose.x - approach_pose.x) < 0.0001 &&
        std::abs(last_pose.y - approach_pose.y) < 0.0001 && plan_size > 2)
      {
        approach_pose = plan.poses[plan_size - 3].pose.position;
      }
      dx = last_pose.x - approach_pose.x;
      dy = last_pose.y - approach_pose.y;
      theta = atan2(dy, dx);
      plan.poses.back().pose.orientation =
        nav2_util::geometry_utils::orientationAroundZAxis(theta);
    }
  }


  RCLCPP_INFO_STREAM(this->logger_,"plan lenght: " << plan.poses.size());

  RCLCPP_INFO_STREAM(this->logger_,"returning from making plan");
  return !plan.poses.empty();
}

bool
CoopPlanner::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
{
  if (wx < costmap_->getOriginX() || wy < costmap_->getOriginY()) {
    return false;
  }

  mx = static_cast<int>(
    std::round((wx - costmap_->getOriginX()) / costmap_->getResolution()));
  my = static_cast<int>(
    std::round((wy - costmap_->getOriginY()) / costmap_->getResolution()));

  if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY()) {
    return true;
  }

  RCLCPP_ERROR(
    logger_,
    "worldToMap failed: mx,my: %d,%d, size_x,size_y: %d,%d", mx, my,
    costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

  return false;
}

void
CoopPlanner::mapToWorld(double mx, double my, double & wx, double & wy)
{
  wx = costmap_->getOriginX() + mx * costmap_->getResolution();
  wy = costmap_->getOriginY() + my * costmap_->getResolution();
}

void
CoopPlanner::clearRobotCell(unsigned int mx, unsigned int my)
{
  // TODO(orduno): check usage of this function, might instead be a request to
  //               world_model / map server
  costmap_->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
}

rcl_interfaces::msg::SetParametersResult
CoopPlanner::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == name_ + ".tolerance") {
        tolerance_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == name_ + ".use_final_approach_orientation") {
        use_final_approach_orientation_ = parameter.as_bool();
      }
      if (name == name_ + ".transportation") {
        transportation_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_cooperative_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_cooperative_planner::CoopPlanner, nav2_core::GlobalPlanner)
