// Copyright 2021 Apex.AI, Inc.
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
#ifndef REFERENCE_SYSTEM__SYSTEM__TYPE__RCLCPP_SYSTEM_HPP_
#define REFERENCE_SYSTEM__SYSTEM__TYPE__RCLCPP_SYSTEM_HPP_
#include "reference_system/nodes/rclcpp/command.hpp"
#include "reference_system/nodes/rclcpp/fusion.hpp"
#include "reference_system/nodes/rclcpp/transform.hpp"
#include "reference_system/nodes/rclcpp/cyclic.hpp"
#include "reference_system/nodes/rclcpp/sensor.hpp"
#include "reference_system/nodes/rclcpp/intersection.hpp"

// Sensor nodes
#include "reference_system/nodes/rclcpp/front_lidar_driver.hpp"
#include "reference_system/nodes/rclcpp/rear_lidar_driver.hpp"
#include "reference_system/nodes/rclcpp/point_cloud_map.hpp"
#include "reference_system/nodes/rclcpp/visualizer.hpp"
#include "reference_system/nodes/rclcpp/lanelet2_map.hpp"
#include "reference_system/nodes/rclcpp/euclidean_cluster_settings.hpp"

// Transform nodes
#include "reference_system/nodes/rclcpp/points_transformer_front.hpp"
#include "reference_system/nodes/rclcpp/points_transformer_rear.hpp"
#include "reference_system/nodes/rclcpp/voxel_grid_downsampler.hpp"
#include "reference_system/nodes/rclcpp/point_cloud_map_loader.hpp"
#include "reference_system/nodes/rclcpp/ray_ground_filter.hpp"
#include "reference_system/nodes/rclcpp/object_collision_estimator.hpp"
#include "reference_system/nodes/rclcpp/mpc_controller.hpp"
#include "reference_system/nodes/rclcpp/parking_planner.hpp"
#include "reference_system/nodes/rclcpp/lane_planner.hpp"

// Fusion nodes
#include "reference_system/nodes/rclcpp/point_cloud_fusion.hpp"
#include "reference_system/nodes/rclcpp/ndt_localizer.hpp"
#include "reference_system/nodes/rclcpp/vehicle_interface.hpp"
#include "reference_system/nodes/rclcpp/lanelet2_global_planner.hpp"
#include "reference_system/nodes/rclcpp/lanelet2_map_loader.hpp"

// Cyclic nodes
#include "reference_system/nodes/rclcpp/behavior_planner.hpp"

// Intersection nodes
#include "reference_system/nodes/rclcpp/euclidean_cluster_detector.hpp"

// Command nodes
#include "reference_system/nodes/rclcpp/vehicle_dbw_system.hpp"
#include "reference_system/nodes/rclcpp/intersection_output.hpp"

struct RclcppSystem
{
  using NodeBaseType = rclcpp::Node;

  // Base types (for backward compatibility)
  using Command = nodes::rclcpp_system::CommandBase;
  using Cyclic = nodes::rclcpp_system::CyclicBase;
  using Fusion = nodes::rclcpp_system::FusionBase;
  using Intersection = nodes::rclcpp_system::IntersectionBase;
  using Sensor = nodes::rclcpp_system::SensorBase;
  using Transform = nodes::rclcpp_system::TransformBase;

  // Specific sensor node types
  using FrontLidarDriver = nodes::rclcpp_system::FrontLidarDriver;
  using RearLidarDriver = nodes::rclcpp_system::RearLidarDriver;
  using PointCloudMap = nodes::rclcpp_system::PointCloudMap;
  using Visualizer = nodes::rclcpp_system::Visualizer;
  using Lanelet2Map = nodes::rclcpp_system::Lanelet2Map;
  using EuclideanClusterSettings = nodes::rclcpp_system::EuclideanClusterSettings;

  // Specific transform node types
  using PointsTransformerFront = nodes::rclcpp_system::PointsTransformerFront;
  using PointsTransformerRear = nodes::rclcpp_system::PointsTransformerRear;
  using VoxelGridDownsampler = nodes::rclcpp_system::VoxelGridDownsampler;
  using PointCloudMapLoader = nodes::rclcpp_system::PointCloudMapLoader;
  using RayGroundFilter = nodes::rclcpp_system::RayGroundFilter;
  using ObjectCollisionEstimator = nodes::rclcpp_system::ObjectCollisionEstimator;
  using MPCController = nodes::rclcpp_system::MPCController;
  using ParkingPlanner = nodes::rclcpp_system::ParkingPlanner;
  using LanePlanner = nodes::rclcpp_system::LanePlanner;

  // Specific fusion node types
  using PointCloudFusion = nodes::rclcpp_system::PointCloudFusion;
  using NDTLocalizer = nodes::rclcpp_system::NDTLocalizer;
  using VehicleInterface = nodes::rclcpp_system::VehicleInterface;
  using Lanelet2GlobalPlanner = nodes::rclcpp_system::Lanelet2GlobalPlanner;
  using Lanelet2MapLoader = nodes::rclcpp_system::Lanelet2MapLoader;

  // Specific cyclic node types
  using BehaviorPlanner = nodes::rclcpp_system::BehaviorPlanner;

  // Specific intersection node types
  using EuclideanClusterDetector = nodes::rclcpp_system::EuclideanClusterDetector;

  // Specific command node types
  using VehicleDBWSystem = nodes::rclcpp_system::VehicleDBWSystem;
  using IntersectionOutput = nodes::rclcpp_system::IntersectionOutput;
};

#endif  // REFERENCE_SYSTEM__SYSTEM__TYPE__RCLCPP_SYSTEM_HPP_
