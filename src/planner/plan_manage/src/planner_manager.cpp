#include <plan_manage/planner_manager.h>
#include <thread>
#include "visualization_msgs/Marker.h"

using namespace std;

namespace primitive_planner
{

void PPPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
{
  nh.param<std::string>("manager/primitiveFolder", primitiveFolder_, "none");
  nh.param("manager/depthCloudStackNum", depthCloudStackNum_, 4); // local map 12Hz - save 1s pointcloud
  nh.param("manager/boxX", boxX_, 8.0);
  nh.param("manager/boxY", boxY_, 9.0);
  nh.param("manager/boxZ", boxZ_, 9.0);
  nh.param("manager/sim_distributed_communication", sim_dist_com_, false); // simulate that the drones are only able to communicate to neighbors within a certain range.
  nh.param("manager/drone_com_r", drone_com_r_, 15.0);                     // drone communication radius
  nh.param("manager/base_com_r", base_com_r_, 30.0);                       // base station communication radius
  nh.param("manager/arc_length", arc_length_, 5.0);
  nh.param("manager/voxelSize", voxelSize_, 0.1);
  // nh.param("manager/pathNum", pathNum_, 61);
  nh.param("manager/sampleSize", sampleSize_, 2000);
  // nh.param("manager/lamda_c", lamda_c_, 12.0);
  nh.param("manager/lamda_l", lamda_l_, 12.0);
  nh.param("manager/lamda_b", lamda_b_, 12.0);
  nh.param("manager/map_size_x", x_size_, -1.0);
  nh.param("manager/map_size_y", y_size_, -1.0);
  nh.param("manager/map_size_z", z_size_, -1.0);
  nh.param("manager/max_vel", max_vel_, -1.0);
  nh.param("manager/drone_id", drone_id, -1);
  nh.param("manager/swarm_clearence", swarm_clearence_, -1.0);

  voxelNumX_ = int(boxX_ / voxelSize_);
  voxelNumY_ = int(boxY_ / voxelSize_);
  voxelNumZ_ = int(boxZ_ / voxelSize_);
  voxelNumAll_ = voxelNumX_ * voxelNumY_ * voxelNumZ_;
  voxelX_ = boxX_;
  voxelY_ = boxY_ / 2.0;
  voxelZ_ = boxZ_ / 2.0;

  correspondences_.resize(voxelNumAll_);
  for (int i = 0; i < voxelNumAll_; i++)
  {
    correspondences_[i].resize(0);
  }

  pathNum_ = readPathList();
  readPathAll();
  readCorrespondences();
  readAgentCorrespondences();

  depthCloudCount_ = 0;
  dep_odom_sub_ = nh.subscribe<nav_msgs::Odometry>("plan_manage/odom", 10, &PPPlannerManager::odomCallback, this);

  // 订阅局部点云(world) 转换为局部robot系下 标记path
  dep_cloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("plan_manage/cloud", 10, &PPPlannerManager::cloudCallback, this);

  // init depthCloudStack_
  depthCloudStack_.resize(depthCloudStackNum_);

  // set leaf size in downsample
  // downSizeFilter_.setLeafSize(voxelSize_, voxelSize_, voxelSize_);

  has_odom_ = false;
  has_cloud_ = false;

  visualization_ = vis;
}

void PPPlannerManager::odomCallback(const nav_msgs::OdometryConstPtr &odom)
{

  robot_pos_(0) = odom->pose.pose.position.x;
  robot_pos_(1) = odom->pose.pose.position.y;
  robot_pos_(2) = odom->pose.pose.position.z;

  has_odom_ = true;
}

void PPPlannerManager::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &img)
{
  // *latestCloud in world frame
  pcl::PointCloud<pcl::PointXYZ>::Ptr latestCloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*img, *latestCloud);

  if (!has_odom_)
  {
    ROS_DEBUG("no odom!");
    return;
  }

  if (latestCloud->points.size() == 0)
    return;

  depthCloudStack_[depthCloudCount_] = latestCloud;
  depthCloudCount_ = (depthCloudCount_ + 1) % depthCloudStackNum_;

  static int cloud_num = 0;
  cloud_num++;
  if (cloud_num == depthCloudStackNum_)
  {
    cloud_num = 0;
    has_cloud_ = true;
  }
}

// 点云下采样 转换坐标系 按照correspondence voxel大小筛选点云 标记path
bool PPPlannerManager::labelObsCollisionPaths(const Eigen::Vector3d &start_pt, const Eigen::Matrix3d &rotVW)
{
  clearPathList_.clear();
  clearPathList_.resize(pathNum_, 0);

  // label collision paths
  if (has_cloud_)
  {
    // downsample
    pcl::PointCloud<pcl::PointXYZ>::Ptr plannerCloudStack(new pcl::PointCloud<pcl::PointXYZ>());
    for (int i = 0; i < depthCloudStackNum_; i++)
    {
      *plannerCloudStack += *(depthCloudStack_[i]);
    }

    // if cloud size is too large, consider setting up random sample(fixed size)
    pcl::PointCloud<pcl::PointXYZ>::Ptr plannerCloud(new pcl::PointCloud<pcl::PointXYZ>());

    // [VoxelGrid --- high fidelity, but low]
    // downSizeFilter_.setInputCloud(plannerCloudStack);
    // downSizeFilter_.filter(*plannerCloud);

    // [RandomSample --- low fidelity, but fast]
    rs_.setInputCloud(plannerCloudStack);
    rs_.setSample(sampleSize_);
    rs_.filter(*plannerCloud);

    // plannerCloud = plannerCloudStack;

    // TODO:[real robot pointcloud noise] use raycast to avoid

    // plannerCloudStack transform to Body Frame
    Eigen::Vector3d pos_v, pos_w;
    int plannerCloudSize = plannerCloud->points.size();

    int indX, indY, indZ, ind, occPathByVoxelNum;

    // TODO: [???] GPU parallel computation
    for (int i = 0; i < plannerCloudSize; i++)
    {
      // pointcloud: world frame in simulation; body frame in experiment;
      pos_w(0) = plannerCloud->points[i].x - start_pt(0);
      pos_w(1) = plannerCloud->points[i].y - start_pt(1);
      pos_w(2) = plannerCloud->points[i].z - start_pt(2);

      pos_v = rotVW * pos_w;

      // select pointcloud in box
      if ((pos_v(0) >= 1e-4 && pos_v(0) <= voxelX_ - 1e-4) &&
          (pos_v(1) >= -voxelY_ + 1e-4 && pos_v(1) <= voxelY_ - 1e-4) &&
          (pos_v(2) >= -voxelZ_ + 1e-4 && pos_v(2) <= voxelZ_ - 1e-4))
      {
        indX = floor((voxelX_ - pos_v(0)) / voxelSize_);
        indY = floor((voxelY_ - pos_v(1)) / voxelSize_);
        indZ = floor((voxelZ_ - pos_v(2)) / voxelSize_);

        ind = voxelNumY_ * voxelNumZ_ * indX + voxelNumZ_ * indY + indZ;
        occPathByVoxelNum = correspondences_[ind].size();

        for (int j = 0; j < occPathByVoxelNum; j++)
        {
          clearPathList_[correspondences_[ind][j]]++;
        }
      }
    }
  }

  return true;
}

vector<int> PPPlannerManager::scorePaths(const Eigen::Vector3d &start_pt,
                                         const Eigen::Vector3d &global_goal, const Eigen::Matrix3d &rotWV)
{
  /* score: 1. collision;
                    2. close to global goal; or delta_pitch/yaw(between endpoint and goal in body frame)
                    || p_0 - p_goal || - || p(t) - p_goal || (p_0 --- odom) 大小始终维持在一定范围内
                    3. low delta_w (smoothness) ;
  TODO: 暂定倾向于靠近中心线的primitive(后续根据实验效果调整)
  */
  double cost, goal_dist;

  std::map<double, int> mapCost;
  std::vector<int> select_path_id;
  Eigen::Vector3d endPoint;
  // Eigen::Vector4d collision_color(1, 0, 0, 1);
  for (int i = 0; i < pathNum_; i++)
  {

    // collision paths
    if (clearPathList_[i] > 0)
    {
      // TODO:[lable red]
      // visualization_->displayInitPathList(pathAllWorld_[i], 0.05, collision_color, 0);
      continue;
    }

    // cost: lower -> better
    // rotWV * pathEndList_ + start_pt: body -> world;
    endPoint = rotWV * pathEndList_[i] + start_pt;
    if ((start_pt - global_goal).norm() > pathLengthMax_ || rotWV.col(0).dot(global_goal - start_pt) <= 0)
    {
      goal_dist = (endPoint - global_goal).norm() - (start_pt - global_goal).norm();
    }
    else
    {
      goal_dist = 99999;
      for (size_t j = 0; j < pathAll_[i].size(); ++j)
      {
        Eigen::Vector3d traj_pt = rotWV * pathAll_[i][j] + start_pt;
        double dist = (traj_pt - global_goal).norm();
        ;
        goal_dist = goal_dist > dist ? dist : goal_dist;
      }
    }

    double bound_cost = 0;
    if (endPoint(0) < -x_size_ / 2 || endPoint(0) > x_size_ / 2 || endPoint(1) < -y_size_ / 2 || endPoint(1) > y_size_ / 2 || endPoint(2) < 0 || endPoint(2) > z_size_)
    {
      bound_cost = 10;
    }

    cost = lamda_l_ * goal_dist + lamda_b_ * bound_cost;

    mapCost.insert({cost, i});
  }

  // TODO: map 数据结构需要修改 我们并不需要key 去查询 value
  if (!mapCost.empty())
  {
    std::map<double, int>::iterator it;
    for (it = mapCost.begin(); it != mapCost.end(); it++)
    {
      select_path_id.push_back((*it).second);
    }
    // TODO:[label green]
    // Eigen::Vector4d select_color(0, 1, 0, 1);
    // visualization_->displayInitPathList(pathAllWorld_[select_path_id.front()], 0.05, select_color, 0);
  }
  else
  {
    ROS_WARN("====[id:%d] All primitives are infeasible!====", drone_id);
  }

  return select_path_id;
}

void PPPlannerManager::readCorrespondences()
{
  std::string fileName = primitiveFolder_ + "/obs_correspondence/obs_correspondence.txt";

  FILE *filePtr = fopen(fileName.c_str(), "rb");
  if (filePtr == NULL)
  {
    ROS_ERROR("Cannot read input [correspondence files], exit.");
    exit(1);
  }

  int val1, voxelID, pathID;
  for (int i = 0; i < voxelNumAll_; i++)
  {
    val1 = fread(&voxelID, 4, 1, filePtr);
    if (val1 != 1)
    {
      ROS_ERROR("Error reading [voxelID] input files, exit.");
      exit(1);
    }

    while (1)
    {
      val1 = fread(&pathID, 4, 1, filePtr);
      if (val1 != 1)
      {
        ROS_ERROR("Error reading [pathID] input files, exit.");
        exit(1);
      }

      if (pathID != -1)
      {
        if (voxelID >= 0 && voxelID < voxelNumAll_ && pathID >= 0 && pathID < pathNum_)
        {
          correspondences_[voxelID].push_back(pathID);
        }
      }
      else
      {
        break;
      }
    }
  }

  fclose(filePtr);
}

void PPPlannerManager::readAgentCorrespondences()
{
  std::string a = primitiveFolder_ + "/agent_correspondence/";
  std::string b = "_correspondence.txt";
  for (int i = 0; i <= int(max_vel_ * 10); i++)
  {
    std::stringstream ss;
    ss << a << i << b;
    std::string fileName = ss.str();

    FILE *filePtr = fopen(fileName.c_str(), "rb");
    if (filePtr == NULL)
    {
      ROS_ERROR("Cannot read input [agentcorrespondence files], i=%d, exit.", i);
      exit(1);
    }

    std::vector<std::vector<int>> velCorrespondences(voxelNumAll_, std::vector<int>(0, 0));

    int voxelID, pathID, tStart, tEnd;
    for (int j = 0; j < voxelNumAll_; j++)
    {

      int val1 = fread(&voxelID, 4, 1, filePtr);
      if (val1 != 1)
      {
        ROS_ERROR("Error reading [voxelID] input files, exit.");
        exit(1);
      }

      while (1)
      {
        val1 = fread(&pathID, 4, 1, filePtr);
        if (val1 != 1)
        {
          ROS_ERROR("Error reading [pathID] input files, exit.");
          exit(1);
        }

        if (pathID != -1)
        {
          int val2 = fread(&tStart, 4, 1, filePtr);
          int val3 = fread(&tEnd, 4, 1, filePtr);

          if (val2 != 1 || val3 != 1)
          {
            ROS_ERROR("Error reading [tStart] or [tEnd] input files, exit.");
            exit(1);
          }

          if (voxelID >= 0 && voxelID < voxelNumAll_ && pathID >= 0 && pathID < pathNum_)
          {
            velCorrespondences[voxelID].push_back(pathID);
            velCorrespondences[voxelID].push_back(tStart);
            velCorrespondences[voxelID].push_back(tEnd);
          }
        }
        else
        {
          break;
        }
      }
    }
    fclose(filePtr);

    allVelCorrespondences_.push_back(velCorrespondences);
  }
}

int PPPlannerManager::readPathList()
{
  std::string fileName = primitiveFolder_ + "/obs_correspondence/path_end.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL)
  {
    ROS_ERROR("Cannot read input [path_end files], exit.");
    exit(1);
  }

  double endX, endY, endZ;
  int pathID;
  int totalLines = 0;

  while (fscanf(filePtr, "%lf %lf %lf %d", &endX, &endY, &endZ, &pathID) == 4)
  {
    if (pathID >= 0)
    {
      pathEndList_.push_back(Eigen::Vector3d(endX, endY, endZ));
    }
    totalLines++;
  }

  fclose(filePtr);
  return totalLines;
}

void PPPlannerManager::readPathAll()
{
  pathAll_.resize(pathNum_);
  pathLengthList_.resize(pathNum_);
  pathLengthMax_ = 0;

  std::string fileName = primitiveFolder_ + "/obs_correspondence/path_all.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL)
  {
    ROS_ERROR("Cannot read [path_all] input files, exit.");
    exit(1);
  }

  int pointNum, val_num, val1, val2, val3, val4, pathID, pathID_last = -666;
  Eigen::Vector3d pos, pos_last;
  double length = 0;
  val_num = fscanf(filePtr, "%d", &pointNum);
  if (val_num != 1)
  {
    ROS_ERROR("Error reading [pointNum] input files, exit.");
    exit(1);
  }

  for (int i = 0; i < pointNum; i++)
  {
    val1 = fscanf(filePtr, "%lf", &pos(0));
    val2 = fscanf(filePtr, "%lf", &pos(1));
    val3 = fscanf(filePtr, "%lf", &pos(2));
    val4 = fscanf(filePtr, "%d", &pathID);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1)
    {
      ROS_ERROR("Error reading [path_all] input files, exit.");
      exit(1);
    }

    if (pathID == pathID_last)
    {
      length += (pos - pos_last).norm();
      if (pathLengthMax_ < length)
        pathLengthMax_ = length;
    }

    // TODO: path上间隔0.1m保存一次；
    if (pathID >= 0 && pathID < pathNum_)
    {
      if (pathAll_[pathID].empty() || (pos - pathAll_[pathID].back()).norm() > 0.1)
      {
        pathAll_[pathID].push_back(pos);
      }
    }
    else
    {
      ROS_ERROR("Illegal pathID=%d", pathID);
      exit(0);
    }

    if (i == pointNum - 1)
    {
      pathAll_[pathID].push_back(pos_last);
      pathLengthList_[pathID] = length;
    }

    if (pathID_last != -666 && pathID != pathID_last)
    {
      pathAll_[pathID_last].push_back(pos_last);
      pathLengthList_[pathID_last] = length;
      length = 0;
    }

    pos_last = pos;
    pathID_last = pathID;
  }

  fclose(filePtr);
}

void PPPlannerManager::visAllPaths(const Eigen::Vector3d &start_pt, const Eigen::Matrix3d &rotWV)
{
  pathAllWorld_.clear();
  pathAllWorld_.resize(pathNum_);
  Eigen::Vector3d pos_world;
  for (int i = 0; i < (int)pathAll_.size(); i++)
  {
    for (int j = 0; j < (int)pathAll_[i].size(); j++)
    {
      pos_world = start_pt + rotWV * pathAll_[i][j];
      pathAllWorld_[i].push_back(pos_world);
    }
  }

  visualization_->displayMultiOptimalPathList(pathAllWorld_, 0.05);
}

bool PPPlannerManager::labelAgentCollisionPaths(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_v, const double &start_time, const Eigen::Matrix3d &rotVW)
{
  // determine velocity id from start velocity
  int vel_id = int(round(start_v.norm() * 10));
  if (vel_id > int(max_vel_ * 10))
  {
    vel_id = max_vel_ * 10;
  }

  // loop over all agents in the swarm
  for (int i = 0; i < (int)swarm_traj.size(); i++)
  {
    // skip agents that were determined to be to far away to cause a collision (id=-2) or have infeasible trajectories (id=-1) and skipp myself
    if (swarm_traj[i].drone_id < 0 || swarm_traj[i].drone_id == drone_id)
    {
      continue;
    }

    // TODO:[??? param set] check time resolution = voxel_resolution/max_vel / 5

    // loop over all positions of the given trajectory
    for (int j = 0; j < static_cast<int>(swarm_traj[i].traj_pos.size()); j += std::max(1, static_cast<int>(floor(voxelSize_ / max_vel_ * 100 / 5))))
    {
      // transform the position of the other agent into our reference frame
      Eigen::Vector3d pos_v = rotVW * (swarm_traj[i].traj_pos[j] - start_pt);

      // check if the position is inside the voxel box
      if ((pos_v(0) >= 1e-4 && pos_v(0) <= voxelX_ - 1e-4) &&
          (pos_v(1) >= -voxelY_ + 1e-4 && pos_v(1) <= voxelY_ - 1e-4) &&
          (pos_v(2) >= -voxelZ_ + 1e-4 && pos_v(2) <= voxelZ_ - 1e-4))
      {
        // determine the position/index of the voxel containing the position of the other agent
        int indX = floor((voxelX_ - pos_v(0)) / voxelSize_);
        int indY = floor((voxelY_ - pos_v(1)) / voxelSize_);
        int indZ = floor((voxelZ_ - pos_v(2)) / voxelSize_);

        // flatten the index
        int ind = voxelNumY_ * voxelNumZ_ * indX + voxelNumZ_ * indY + indZ;

        // get the number of paths that intersect the voxel
        int occPathNumByVoxel = allVelCorrespondences_[vel_id][ind].size() / 3;

        // calculate the offset time
        double other_cur_time = swarm_traj[i].start_time + j * 0.01;
        // loop over the trajectories intersecting the voxel
        for (int k = 0; k < occPathNumByVoxel; k++)
        {
          // check if the time range is critical (if the paths cross at the same time).
          if (other_cur_time > start_time + allVelCorrespondences_[vel_id][ind][3 * k + 1] / 1000 && other_cur_time < start_time + allVelCorrespondences_[vel_id][ind][3 * k + 2] / 1000)
          {
            clearPathList_[allVelCorrespondences_[vel_id][ind][3 * k]]++;
          }
        }
      }
    }
  }

  return true;
}

bool PPPlannerManager::trajReplan(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_v, const double &start_time, const Eigen::Matrix3d &RWV, const Eigen::Vector3d &global_goal, vector<int> &select_path_id)
{
  Eigen::Matrix3d RVW = RWV.inverse();

  bool success_obs, success_agents;
  ros::Time t0, t1, t2, t3;
  t0 = ros::Time::now();
  success_obs = labelObsCollisionPaths(start_pt, RVW);
  t1 = ros::Time::now();

  success_agents = labelAgentCollisionPaths(start_pt, start_v, start_time, RVW);
  t2 = ros::Time::now();

  if (!success_obs || !success_agents)
    return false;

  // visulize all path
  // visAllPaths(start_pt, RWV);

  select_path_id = scorePaths(start_pt, global_goal, RWV);
  t3 = ros::Time::now();

  if (select_path_id.empty())
    return false;

  return true;
}

} // namespace primitive_planner