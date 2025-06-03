#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <eigen3/Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <stdlib.h>

namespace primitive_planner
{
class PlanningVisualization
{
private:
  ros::NodeHandle node;

  ros::Publisher goal_point_pub;
  ros::Publisher global_list_pub;
  ros::Publisher init_list_pub;
  ros::Publisher optimal_list_pub;
  ros::Publisher failed_list_pub;
  ros::Publisher a_star_list_pub;
  ros::Publisher guide_vector_pub;

  ros::Publisher intermediate_pt0_pub;
  ros::Publisher intermediate_pt1_pub;
  ros::Publisher intermediate_grad0_pub;
  ros::Publisher intermediate_grad1_pub;
  ros::Publisher intermediate_grad_smoo_pub;
  ros::Publisher intermediate_grad_dist_pub;
  ros::Publisher intermediate_grad_feas_pub;
  ros::Publisher intermediate_grad_swarm_pub;

  std::vector<Eigen::Vector3d> transformToWorld(const std::vector<std::vector<Eigen::Vector3d>> &trajs, const int id, const Eigen::Vector3d &startPt, const Eigen::Matrix3d &rotWV);

public:
  PlanningVisualization(/* args */)
  {
  }
  ~PlanningVisualization()
  {
  }
  PlanningVisualization(ros::NodeHandle &nh);

  typedef std::shared_ptr<PlanningVisualization> Ptr;

  void displayMarkerList(ros::Publisher &pub, const std::vector<Eigen::Vector3d> &list, double scale,
                         Eigen::Vector4d color, int id, bool show_sphere = true);
  void generatePathDisplayArray(visualization_msgs::MarkerArray &array,
                                const std::vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
  void generateArrowDisplayArray(visualization_msgs::MarkerArray &array,
                                 const std::vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
  void displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id);
  void displayGlobalPathList(std::vector<Eigen::Vector3d> &global_pts, const double scale, int id);
  void displayInitPathList(const std::vector<Eigen::Vector3d> &init_pts, const double scale, Eigen::Vector4d color, int id);
  void displayMultiInitPathList(std::vector<std::vector<Eigen::Vector3d>> &init_trajs, const double scale);
  void displayPathSelection(const std::vector<int> &collisionPaths, const std::vector<int> &validPaths, const int selectedPath, const std::vector<std::vector<Eigen::Vector3d>> &trajs, const Eigen::Vector3d &startPt, const Eigen::Matrix3d &rotWV);
  void displayMultiOptimalPathList(std::vector<std::vector<Eigen::Vector3d>> &optimal_trajs, const double scale);
  void displayOptimalList(std::vector<Eigen::Vector3d> &optimal_pts, int id);
  void displayFailedList(Eigen::MatrixXd failed_pts, int id);
  void displayAStarList(std::vector<std::vector<Eigen::Vector3d>> &a_star_paths, int id);
  void displayArrowList(ros::Publisher &pub, const std::vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);

  void displayIntermediatePt(std::string type, Eigen::MatrixXd &pts, int id, Eigen::Vector4d color);
  void displayIntermediateGrad(std::string type, Eigen::MatrixXd &pts, Eigen::MatrixXd &grad, int id, Eigen::Vector4d color);
  // void displayNewArrow(ros::Publisher& guide_vector_pub, ego_planner::PolyTrajOptimizer::Ptr optimizer);
};
} // namespace primitive_planner
#endif