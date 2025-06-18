#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <traj_utils/planning_visualization.h>
#include <traj_utils/plan_container.hpp>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <vector>
#include <map>
#include <string>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/filters/random_sample.h>

namespace primitive_planner
{
// Primitive Planner Manager
// Key algorithms of mapping and planning are called

class PPPlannerManager
{
public:
  PPPlannerManager(){};
  ~PPPlannerManager(){};

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis = NULL);
  void odomCallback(const nav_msgs::OdometryConstPtr &odom);
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &img);
  bool labelObsCollisionPaths(const Eigen::Vector3d &start_pt, const Eigen::Matrix3d &rotVW);
  bool labelAgentCollisionPaths(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_v, const double &start_time, const Eigen::Matrix3d &rotVW);
  vector<int> scorePaths(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &global_goal, const Eigen::Matrix3d &rotWV, const primitive_planner::LocalTrajData &current_traj, const Eigen::Vector3d &virtual_vel);
  void visAllPaths(const Eigen::Vector3d &start_pt, const Eigen::Matrix3d &rotWV);
  bool trajReplan(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_v, const double &start_time, const Eigen::Matrix3d &RWV, const Eigen::Vector3d &global_goal, vector<int> &select_path_id, const primitive_planner::LocalTrajData &current_traj, const Eigen::Vector3d &virtual_vel);

  void readCorrespondences();
  void readAgentCorrespondences();
  // int readPlyHeader(FILE *filePtr);
  int readPathList();
  void readPathAll();
  void determineEndDirection();

  int flight_type_;
  int drone_id;
  double max_vel_;
  bool sim_dist_com_;
  double drone_com_r_;
  double base_com_r_;
  double arc_length_;
  double swarm_clearance_;
  std::string primitiveFolder_;
  SwarmTrajData swarm_traj;

  double voxelSize_;

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> depthCloudStack_;

  int depthCloudStackNum_, depthCloudCount_;

  bool has_odom_, has_cloud_;

  // velocity vector of the virtual leader
  Eigen::Vector3d virtual_vel_;

private:
  PlanningVisualization::Ptr visualization_;

  ros::Subscriber dep_odom_sub_, dep_cloud_sub_;

  Eigen::Vector3d robot_pos_;
  Eigen::Quaterniond robot_q_;

  // pcl::VoxelGrid<pcl::PointXYZ> downSizeFilter_;// VoxelGrid filter
  pcl::RandomSample<pcl::PointXYZ> rs_; // RandomSample

  double boxX_, boxY_, boxZ_;
  double voxelX_, voxelY_, voxelZ_;
  int voxelNumX_, voxelNumY_, voxelNumZ_, voxelNumAll_, voxelNum_swarm_clearance_;

  int pathNum_, sampleSize_;

  std::vector<std::vector<int>> correspondences_;
  std::vector<std::vector<std::vector<int>>> allVelCorrespondences_;
  std::vector<int> clearPathList_;
  std::vector<Eigen::Vector3d> pathEndList_;               // body frame
  std::vector<std::optional<Eigen::Vector3d>> pathEndDir_; // body frame
  std::vector<std::vector<Eigen::Vector3d>> pathAll_, pathAllWorld_;
  std::vector<double> pathLengthList_;
  double pathLengthMax_;

  double lambda_l_, lambda_b_, lambda_d_, lambda_heading_virtual_, lambda_heading_neighbors_end_, lambda_heading_neighbors_start_, lambda_speed_virtual_, lambda_speed_neighbors_, lambda_contraction_;

  double x_size_, y_size_, z_size_;

  // path parameters

  // point cloud

public:
  typedef std::unique_ptr<PPPlannerManager> Ptr;
};
} // namespace primitive_planner

#endif