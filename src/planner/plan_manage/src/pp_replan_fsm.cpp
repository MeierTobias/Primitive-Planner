#include <plan_manage/pp_replan_fsm.h>
#include <plan_manage/shared_memory.h>
#include <cmath>
#include <vector>

#define USE_SHARED_MEMORY false

namespace primitive_planner
{

using namespace std;

double yaw_diff(double yaw1, double yaw2)
{
  double diff = yaw1 - yaw2;
  diff = std::fmod(diff + M_PI, 2 * M_PI) - M_PI;
  if (diff < -M_PI)
    diff += 2 * M_PI;

  return diff;
}

double limit_yaw(double yaw)
{
  if (yaw > M_PI)
    yaw -= 2 * M_PI;
  if (yaw < -M_PI)
    yaw += 2 * M_PI;

  return yaw;
}

PPReplanFSM::~PPReplanFSM()
{
  detach_shared_memory(shared_memory_);
  remove_shared_memory(shm_id_);
}

void PPReplanFSM::init(ros::NodeHandle &nh)
{
  have_odom_ = false;
  have_target_ = false;
  flag_escape_emergency_ = true;
  flag_wait_crash_rec_ = false;
  crash_rec_stage_ = 1;
  have_latest_safe_pt_ = false;
  keep_fails_ = 0;
  have_log_files_ = false;

  exec_state_ = FSM_EXEC_STATE::INIT;
  nh.param("fsm/flight_type", flight_type_, 2);
  nh.param("fsm/thresh_replan_time", replan_thresh_, 1.0);
  nh.param("fsm/realworld_experiment", flag_realworld_experiment_, false);
  nh.param("fsm/fail_safe", enable_fail_safe_, true);
  nh.param("fsm/no_replan_thresh", no_replan_thresh_, 4.0);
  nh.param("fsm/waypoint_num", waypoint_num_, -1);

  int total_drones;
  nh.param("/total_drones", total_drones);

  // Raise the default logger level
  // TODO: This can be removed after development.
  if (ros::console::set_logger_level(
          ROSCONSOLE_DEFAULT_NAME,
          ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

#if USE_SHARED_MEMORY
  // shared memory
  const size_t size = TOTAL_DRONE_NUM_ * sizeof(SharedMemory);
  shm_id_ = get_or_create_shared_memory(size);
  shared_memory_ = attach_shared_memory(shm_id_);
  swarm_traj_ptr_ = static_cast<SharedMemory *>(shared_memory_);
#endif

  // initialize main modules
  visualization_.reset(new PlanningVisualization(nh));
  planner_manager_.reset(new PPPlannerManager);
  planner_manager_->initPlanModules(nh, visualization_);
  this->drone_counter_.init(nh, planner_manager_->getRobotPos(), static_cast<unsigned int>(total_drones));

  readPrimitivePos();

  // external trigger
  have_trigger_ = !flag_realworld_experiment_;
  trigger_sub_ = nh.subscribe("/traj_start_trigger", 100, &PPReplanFSM::triggerCallback, this);

  // commands
  odom_sub_ = nh.subscribe("odom_world", 100, &PPReplanFSM::odometryCallback, this);
  mandatory_stop_sub_ = nh.subscribe("mandatory_stop", 100, &PPReplanFSM::mandatoryStopCallback, this);
  select_path_end_sub_ = nh.subscribe("planning/select_path_end", 100, &PPReplanFSM::pathEndCallback, this);
  cmd_sub_ = nh.subscribe("/position_cmd", 100, &PPReplanFSM::cmdCallback, this);

  // swarm communication
  broadcast_primitive_pub_ = nh.advertise<traj_utils::swarmPrimitiveTraj>("planning/broadcast_primitive_send", 100);

#if not USE_SHARED_MEMORY
  broadcast_primitive_sub_ = nh.subscribe<traj_utils::swarmPrimitiveTraj>("planning/broadcast_primitive_recv", 100,
                                                                          &PPReplanFSM::RecvBroadcastPrimitiveCallback,
                                                                          this,
                                                                          ros::TransportHints().tcpNoDelay());
#endif

  path_id_pub_ = nh.advertise<traj_utils::swarmPrimitiveTraj>("planning/selected_path_id", 100);
  stop_pub_ = nh.advertise<std_msgs::Float64MultiArray>("planning/stop_command", 100);
  heartbeat_pub_ = nh.advertise<std_msgs::Empty>("planning/heartbeat", 100);
  global_pub_ = nh.advertise<std_msgs::Float64MultiArray>("planning/global_goal", 100);
  poly_pub_ = nh.advertise<traj_utils::Polynomial>("planning/polynomial_traj", 100);
  yaw_cmd_pub_ = nh.advertise<std_msgs::Float64>("planning/yaw_cmd", 100);

  // Wait 3s --- ensure to save 30 frames pointCloud
  ROS_INFO("Wait for 3 seconds.");
  ros::Time start_time = ros::Time::now();
  while (ros::ok() && ((ros::Time::now() - start_time).toSec() < 3.0))
  {
    ros::spinOnce();
  }
  while (ros::ok() && !have_odom_)
  {
    ros::spinOnce();
  }
  starting_pos_ = odom_pos_;

  exec_timer_ = nh.createTimer(ros::Duration(0.01), &PPReplanFSM::execFSMCallback, this);

  // global goal
  switch (flight_type_)
  {
  case 1:
  case 3:
    waypoint_sub_ = nh.subscribe("/goal_with_id", 100, &PPReplanFSM::waypointCallback, this);
    break;

  case 2: {
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + std::to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + std::to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + std::to_string(i) + "_z", waypoints_[i][2], -1.0);
      all_goal_.push_back(Eigen::Vector3d(waypoints_[i][0], waypoints_[i][1], waypoints_[i][2]));
    }

    if (all_goal_.size() > 0)
    {
      goal_id_ = 0;
      global_goal_ = all_goal_[goal_id_];
      // send global goal
      std_msgs::Float64MultiArray goal_msg;
      for (int i = 0; i < 3; i++)
      {
        goal_msg.data.push_back(global_goal_(i));
      }
      global_pub_.publish(goal_msg);

      visualization_->displayGoalPoint(global_goal_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

      Eigen::Vector3d dir_to_goal = global_goal_ - odom_pos_;
      double yaw_des = limit_yaw(atan2(dir_to_goal(1), dir_to_goal(0)));
      turnTowardsGoal(yaw_des);

      have_target_ = true;
    }
    break;
  }
  default:
    ROS_ERROR("Unknown flight type");
    break;
  }
  ROS_DEBUG("[FSM] Drone %d: Init finished", planner_manager_->drone_id);
}

void PPReplanFSM::turnTowardsGoal(double yaw_des)
{
  std_msgs::Float64 yaw_cmd;
  yaw_cmd.data = yaw_des;
  int count = 0;
  while (ros::ok())
  {
    if (count++ % 100 == 0)
    {
      yaw_cmd_pub_.publish(yaw_cmd);
    }

    double diff = odom_yaw_ - yaw_des;
    diff = std::fmod(diff + M_PI, 2 * M_PI) - M_PI;
    if (diff < -M_PI)
      diff += 2 * M_PI;
    if (abs(diff) < 0.1)
      break;

    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
}

void PPReplanFSM::newGoalReceived(const Eigen::Vector3d &goal)
{
  waypoint_num_ = 1;
  goal_id_ = 0;
  all_goal_.clear();
  all_goal_.push_back(goal);
  global_goal_ = all_goal_[0];

  std_msgs::Float64MultiArray goal_msg;
  for (int i = 0; i < 3; i++)
  {
    goal_msg.data.push_back(global_goal_(i));
  }
  global_pub_.publish(goal_msg);
  visualization_->displayGoalPoint(global_goal_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.15, 0);
  if (exec_state_ == WAIT_TARGET)
  {
    Eigen::Vector3d dir_to_goal = global_goal_ - odom_pos_;
    double yaw_des = atan2(dir_to_goal(1), dir_to_goal(0));
    turnTowardsGoal(yaw_des);
  }
  have_target_ = true;
  have_trigger_ = true;
  changeFSMExecState(GEN_NEW_TRAJ, "NEW_GOAL");
}

void PPReplanFSM::waypointCallback(const quadrotor_msgs::GoalSetPtr &msg)
{
  if (msg->drone_id != planner_manager_->drone_id)
    return;

  ++goal_tag_;
  ROS_INFO("[FSM] Drone %d: Received goal: %f, %f, %f", planner_manager_->drone_id, msg->goal[0], msg->goal[1], msg->goal[2]);
  newGoalReceived(Eigen::Vector3d(msg->goal[0], msg->goal[1], msg->goal[2]));
}

void PPReplanFSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg)
{
  // generate folder together
  // odom_logger_.init("/home/hjl/project/A_personal_project/primitive_swarm/bags_and_logs", "odom", planner_manager_->drone_id);
  // odom_logger_.appendData("timestamp,px,py,pz,vx,vy,vz,ax,ay,az");
  // computing_time_logger_.init("/home/hjl/project/A_personal_project/primitive_swarm/bags_and_logs", "time", planner_manager_->drone_id);
  // computing_time_logger_.appendData("timestamp,computing_time");

  ROS_INFO("Wait for 2 seconds.");
  int count = 0;
  while (ros::ok() && count++ < 2000)
  {
    ros::spinOnce();
    ros::Duration(0.001).sleep();
  }

  have_trigger_ = true;
  have_log_files_ = true;

  if (flight_type_ == 2 && !have_target_)
  {
    std::vector<Eigen::Vector3d> new_all_goal;
    new_all_goal.reserve(all_goal_.size());
    // Insert every waypoint into the list, except the last
    for (int i = all_goal_.size() - 2; i >= 0; i--)
    {
      new_all_goal.push_back(all_goal_[i]);
    }
    // Instead, "last goal" and "starting_position" swap places
    new_all_goal.push_back(starting_pos_);
    starting_pos_ = all_goal_.back();
    all_goal_ = new_all_goal;

    goal_id_ = 0;
    global_goal_ = all_goal_[goal_id_];
    have_target_ = true;
  }
}

void PPReplanFSM::RecvBroadcastPrimitiveCallback(const traj_utils::swarmPrimitiveTrajConstPtr &msg)
{
  // get sender id
  const size_t recv_id = static_cast<size_t>(msg->drone_id);
  // ignore own messages
  if (static_cast<int>(recv_id) == planner_manager_->drone_id)
    return;

  // extend the trajectory vector if messages from unknown drones arrive
  if (planner_manager_->swarm_traj.size() <= recv_id)
  {
    for (size_t i = planner_manager_->swarm_traj.size(); i <= recv_id; i++)
    {
      LocalTrajData blank;
      blank.drone_id = -1;
      blank.start_time = 0.0;
      planner_manager_->swarm_traj.push_back(blank);
    }
  }

  // get start and end positions of trajectory
  Eigen::Vector3d start_pos, end_pos;
  start_pos << msg->start_p[0], msg->start_p[1], msg->start_p[2];
  end_pos << msg->end_p[0], msg->end_p[1], msg->end_p[2];
  double start_dist = (start_pos - odom_pos_).norm();
  double end_dist = (end_pos - odom_pos_).norm();

  // simulate restricted communication range
  if (planner_manager_->sim_dist_com_)
  {
    if (start_dist > planner_manager_->drone_com_r_)
    {
      return;
    }
  }

  if (flight_type_ == 3) // decentralized global goal
  {
    // check we received a new goal position from a neighbor
    if (msg->goal_tag > goal_tag_)
    {
      ++goal_tag_;
      ROS_INFO("[FSM] Drone %d: New decentralized goal position (%f,%f,%f) from neighbor (id=%ld) received.", planner_manager_->drone_id, global_goal_[0], global_goal_[1], global_goal_[2], recv_id);
      newGoalReceived(Eigen::Vector3d(msg->goal[0], msg->goal[1], msg->goal[2]));
    }
  }

  // check if the received trajectories are close enough to cause interference
  bool far_away = start_dist > planner_manager_->arc_length_ * 2 &&
                  end_dist > planner_manager_->arc_length_ * 2;

  if (!far_away)
  {
    Eigen::Matrix<double, 3, 3> Rwv;
    Rwv << msg->rot_mat[0], msg->rot_mat[1], msg->rot_mat[2],
        msg->rot_mat[3], msg->rot_mat[4], msg->rot_mat[5],
        msg->rot_mat[6], msg->rot_mat[7], msg->rot_mat[8];

    std::vector<int> path_id;
    path_id.push_back(msg->select_path_id);

    std::vector<Eigen::Vector3d> traj_pos;
    double traj_duration;
    int vel_id = msg->vel_id;
    readLocalTrajPos(start_pos, vel_id, Rwv, path_id, traj_pos, traj_duration);
    if (vel_id != msg->vel_id)
    {
      ROS_ERROR("vel_id has been changed! Should not happen!");
    }

    planner_manager_->swarm_traj[recv_id].drone_id = recv_id;
    planner_manager_->swarm_traj[recv_id].start_time = msg->start_time.toSec();
    planner_manager_->swarm_traj[recv_id].traj_duration = traj_duration;

    // if the drone is hovering it sends its current position (goal position) as the end_pos hence we only have to check this point for collisions and not the whole trajectory.
    planner_manager_->swarm_traj[recv_id].hovering_at_goal = msg->hovering_at_goal;
    if (msg->hovering_at_goal)
    {
      planner_manager_->swarm_traj[recv_id].traj_pos = {end_pos};
    }
    else
    {
      planner_manager_->swarm_traj[recv_id].traj_pos = traj_pos;
    }

    // Check collision between current traj and other new traj
#if not USE_SHARED_MEMORY
    if (exec_state_ == EXEC_TRAJ && checkCollision(recv_id))
    {
      changeFSMExecState(REPLAN_TRAJ, "SWARM_CHECK");
    }
#endif
  }
  else
  {
    planner_manager_->swarm_traj[recv_id].drone_id = -2;
    planner_manager_->swarm_traj[recv_id].start_time = msg->start_time.toSec();
  }
}

bool PPReplanFSM::checkCollision(int recv_id)
{
  // my first planning has not started
  if (myself_traj_.start_time < 1e9)
    return false;
  // invalid trajectory
  if (planner_manager_->swarm_traj[recv_id].drone_id != recv_id)
    return false;

  double my_traj_start_time = myself_traj_.start_time;

  double other_traj_start_time = planner_manager_->swarm_traj[recv_id].start_time;

  double t_start = std::max(my_traj_start_time, other_traj_start_time);
  double t_end = std::min(my_traj_start_time + myself_traj_.traj_duration * 2 / 3,
                          other_traj_start_time + planner_manager_->swarm_traj[recv_id].traj_duration);

  // check time resolution = voxel_resolution/max_vel;
  for (double t = t_start; t < t_end; t += planner_manager_->voxelSize_ / planner_manager_->max_vel_)
  {
    // precise approach: interpolation pos by time [to be realized]
    // pos resolution: 10ms; 1000/10
    if ((myself_traj_.traj_pos[round((t - my_traj_start_time) * 100)] - planner_manager_->swarm_traj[recv_id].traj_pos[round((t - other_traj_start_time) * 100)]).norm() < planner_manager_->swarm_clearance_)
    {
      return true;
    }
  }

  return false;
}

bool PPReplanFSM::readLocalTrajPos(Eigen::Vector3d &start_pos, int &vel_id, Eigen::Matrix<double, 3, 3> &Rwv, std::vector<int> &path_id, std::vector<Eigen::Vector3d> &traj_pos, double &traj_duration)
{
  int best_path_id;

  if (vel_id >= static_cast<int>(primitve_pos_.size()))
  {
    ROS_ERROR("vel_id >= primitve_pos_.size()!");
    return false;
  }

  int idx = 0;
  while (true)
  {
    if (path_id[idx] >= static_cast<int>(primitve_pos_[vel_id].size()))
    {
      idx++;
    }
    else
    {
      // success
      best_path_id = path_id[idx];
      break;
    }

    if (idx >= (int)path_id.size())
    {
      string msg = "[FSM] path id ";
      for (auto it : path_id)
        msg += " " + std::to_string(it);
      msg += " not exist for vel_id " + std::to_string(vel_id) + ".";
      ROS_WARN(msg.c_str());
      idx = 0;
      vel_id--;
    }

    if (vel_id < 0)
    {
      ROS_ERROR("[FSM] No suitable path exists!!!");
      return false;
    }
  }

  Eigen::Vector3d pos_body, pos_world;
  int ptNum = primitve_pos_[vel_id][best_path_id].second.size();
  traj_duration = primitve_pos_[vel_id][best_path_id].first;
  for (int i = 0; i < ptNum; i++)
  {

    // velocity frame -> world frame start_pos + Rwv * pos
    pos_body = primitve_pos_[vel_id][best_path_id].second[i];
    pos_world = start_pos + Rwv * pos_body;
    // not push_back
    traj_pos.push_back(pos_world);
  }

  path_id.clear();
  path_id.push_back(best_path_id);
  return true;
}

bool PPReplanFSM::readPrimitivePos()
{
  FILE *filePtr;
  int path_id = 0;
  int vel_id = 0;
  std::string a = planner_manager_->primitiveFolder_ + "/trajectory_pos/";
  std::string b = "/";
  std::string c = "_trajectory.ply";
  while (true)
  {

    std::stringstream ss;
    ss << a << vel_id << b << path_id << c;
    std::string fileName = ss.str();

    filePtr = fopen(fileName.c_str(), "r");
    if (filePtr != NULL)
    {
      while (static_cast<int>(primitve_pos_.size()) < vel_id + 1)
      {
        vector<std::pair<double, vector<Eigen::Vector3d>>> blank;
        primitve_pos_.push_back(blank);
      }

      int pointNum, val1, val2, val3, val4, val5;
      double pos_x, pos_y, pos_z, traj_duration;
      val1 = fscanf(filePtr, "%d", &pointNum);
      val2 = fscanf(filePtr, "%lf", &traj_duration);

      if (val1 != 1 || val2 != 1)
      {
        ROS_ERROR("Error reading number of points or trajectory duration, exit.");
        exit(1);
      }

      vector<Eigen::Vector3d> one_primitive_pos;
      for (int i = 0; i < pointNum; i++)
      {
        // read trajectory_pos
        val3 = fscanf(filePtr, "%lf", &pos_x);
        val4 = fscanf(filePtr, "%lf", &pos_y);
        val5 = fscanf(filePtr, "%lf", &pos_z);

        if (val3 != 1 || val4 != 1 || val5 != 1)
        {
          ROS_ERROR("Error reading trajectory position, exit.");
          exit(1);
        }

        one_primitive_pos.push_back(Eigen::Vector3d(pos_x, pos_y, pos_z));
      }
      primitve_pos_[vel_id].push_back(std::make_pair(traj_duration, one_primitive_pos));

      fclose(filePtr);
      path_id++;
    }
    else
    {
      if (path_id > 0)
      {
        path_id = 0;
        vel_id++;
      }
      else
      {
        return true;
      }
    }
  }

  return false;
}

void PPReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
{
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_q_.x() = msg->pose.pose.orientation.x;
  odom_q_.y() = msg->pose.pose.orientation.y;
  odom_q_.z() = msg->pose.pose.orientation.z;
  odom_q_.w() = msg->pose.pose.orientation.w;

  odom_x_dir_ = odom_q_.toRotationMatrix().col(0);
  odom_yaw_ = atan2(odom_x_dir_(1), odom_x_dir_(0));

  have_odom_ = true;

  static Eigen::Vector3d last_pos(0, 0, 0);
  if ((odom_pos_ - last_pos).norm() > 0.03)
  {
    std::vector<Eigen::Vector3d> pts;
    pts.push_back(odom_pos_);
    pts.push_back(global_goal_);
    visualization_->displayGlobalPathList(pts, 0.02, 0);

    last_pos = odom_pos_;
  }

  if (have_latest_safe_pt_)
  {
    if ((odom_pos_ - latest_safe_pt_).norm() > 0.1)
    {
      if (planner_manager_->has_cloud_ && planner_manager_->depthCloudStack_.size() > 0)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr plannerCloud(new pcl::PointCloud<pcl::PointXYZ>());
        for (int i = 0; i < planner_manager_->depthCloudStackNum_; i++)
        {
          *plannerCloud += *(planner_manager_->depthCloudStack_[i]);
        }

        const double SAFE_DIST = 0.3232; // m
        bool safe = true;
        for (size_t j = 0; j < plannerCloud->points.size(); j++)
        {
          Eigen::Vector3d pc(plannerCloud->points[j].x, plannerCloud->points[j].y, plannerCloud->points[j].z);
          double dist = (odom_pos_ - pc).norm();
          if (dist < SAFE_DIST)
          {
            safe = false;
            break;
          }
        }

        if (safe)
          latest_safe_pt_ = odom_pos_;
      }
    }
  }
  else
  {
    latest_safe_pt_ = odom_pos_;
    have_latest_safe_pt_ = true;
  }

  // if ( have_log_files_ )
  // {
  //   std::vector<double> data;
  //   data.push_back(odom_pos_(0));
  //   data.push_back(odom_pos_(1));
  //   data.push_back(odom_pos_(2));
  //   data.push_back(odom_vel_(0));
  //   data.push_back(odom_vel_(1));
  //   data.push_back(odom_vel_(2));
  //   data.push_back(0);
  //   data.push_back(0);
  //   data.push_back(0);
  //   std::string csv_data = odom_logger_.toCSV(ros::Time::now(), data);
  //   odom_logger_.appendData(csv_data);
  // }
}

void PPReplanFSM::pathEndCallback(const std_msgs::Float64MultiArrayPtr &msg)
{
  select_path_end_last_ = select_path_end_;

  select_path_end_(0) = msg->data[0];
  select_path_end_(1) = msg->data[1];
  select_path_end_(2) = msg->data[2];
}

void PPReplanFSM::cmdCallback(const quadrotor_msgs::PositionCommandPtr &cmd)
{
  start_pt_(0) = cmd->position.x;
  start_pt_(1) = cmd->position.y;
  start_pt_(2) = cmd->position.z;
  start_v_(0) = cmd->velocity.x;
  start_v_(1) = cmd->velocity.y;
  start_v_(2) = cmd->velocity.z;
}

void PPReplanFSM::execFSMCallback(const ros::TimerEvent &e)
{
  exec_timer_.stop(); // To avoid blockage
  std_msgs::Empty heartbeat_msg;
  heartbeat_pub_.publish(heartbeat_msg);

  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 500)
  {
    fsm_num = 0;
    printFSMExecState();
  }

  switch (exec_state_)
  {
  case INIT: {
    if (have_odom_)
    {
      changeFSMExecState(WAIT_TARGET, "FSM");
    }
    break;
  }

  case WAIT_TARGET: {
    // state transition condition
    if (have_target_ && have_trigger_)
    {
      drone_counter_.unsetReachedGoal();
      changeFSMExecState(GEN_NEW_TRAJ, "FSM");
    }
    else
    {
      // publish the current trajectory so that other drones can avoid me even if I'm just hovering (waiting)
      traj_msg.hovering_at_goal = true;
      traj_msg.end_p[0] = odom_pos_[0];
      traj_msg.end_p[1] = odom_pos_[1];
      traj_msg.end_p[2] = odom_pos_[2];
      broadcast_primitive_pub_.publish(traj_msg);
    }
    break;
  }

  case GEN_NEW_TRAJ: {
    bool success = planPrimitive(true);
    if (success)
    {
      changeFSMExecState(EXEC_TRAJ, "FSM");
      flag_escape_emergency_ = true;
    }
    else
    {
      changeFSMExecState(GEN_NEW_TRAJ, "FSM"); // "changeFSMExecState" must be called each time planned
      if (odom_vel_.norm() < 0.1)
      {
        changeFSMExecState(CRASH_RECOVER, "FSM");
        crash_rec_stage_ = 1;
        flag_pub_first_yaw_ = false;
        yaw_cmd_count_ = 0;
      }
    }
    break;
  }

  // TODO: This state could probably be merged with the one above
  case REPLAN_TRAJ: {
    bool success = planPrimitive(false);
    if (success)
    {
      changeFSMExecState(EXEC_TRAJ, "FSM");
    }
    else
    {
      changeFSMExecState(REPLAN_TRAJ, "FSM");
      if (odom_vel_.norm() < 0.1)
      {
        changeFSMExecState(CRASH_RECOVER, "FSM");
        crash_rec_stage_ = 1;
        flag_pub_first_yaw_ = false;
        yaw_cmd_count_ = 0;
      }
    }

    break;
  }

  case EXEC_TRAJ: {
    double delta_t = (ros::Time::now() - start_time_).toSec();
    if ((odom_pos_ - global_goal_).norm() < no_replan_thresh_)
    {
      if (goal_id_ == (waypoint_num_ - 1))
      {
        changeFSMExecState(APPROACH_GOAL, "FSM");
        std::stringstream ss;
        ss << "odom_pos_=" << odom_pos_.transpose() << " global_goal_=" << global_goal_.transpose() << " norm=" << (odom_pos_ - global_goal_).norm() << " thres=" << no_replan_thresh_;
        ROS_DEBUG(ss.str().c_str());
      }
      else
      {
        goal_id_++;
        global_goal_ = all_goal_[goal_id_];

        // send global goal
        std_msgs::Float64MultiArray goal_msg;
        for (int i = 0; i < 3; i++)
        {
          goal_msg.data.push_back(global_goal_(i));
        }
        global_pub_.publish(goal_msg);
      }
    }
    else if (delta_t > replan_thresh_)
    {
      changeFSMExecState(REPLAN_TRAJ, "FSM");
    }

    break;
  }

  case APPROACH_GOAL: {

    pubPolyTraj(odom_pos_, odom_vel_, global_goal_, 1.0);

    // publish the current trajectory so that other drones can avoid me while I'm steering towords my goal
    traj_msg.hovering_at_goal = true;
    traj_msg.end_p[0] = global_goal_[0];
    traj_msg.end_p[1] = global_goal_[1];
    traj_msg.end_p[2] = global_goal_[2];
    broadcast_primitive_pub_.publish(traj_msg);

    have_target_ = false;
    have_trigger_ = false;
    drone_counter_.setReachedGoal();
    if (drone_counter_.allDronesArrived())
    {
      changeFSMExecState(WAIT_TARGET, "FSM");
    }

    break;
  }

  case EMERGENCY_STOP: {
    if (flag_escape_emergency_) // Avoiding repeated calls
    {
      callEmergencyStop(odom_pos_);
    }
    else
    {
      if (enable_fail_safe_ && odom_vel_.norm() < 0.1)
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
    }

    flag_escape_emergency_ = false;
    break;
  }

  case CRASH_RECOVER: {
    std::stringstream ss;
    ss << "ID=" << planner_manager_->drone_id << " crash_rec_stage_=" << crash_rec_stage_;
    ROS_DEBUG(ss.str().c_str());
    if (crash_rec_stage_ == 1)
    {
      std::stringstream ss;
      ss << "ID=" << planner_manager_->drone_id << "A=" << yaw_cmd_count_ << " B=" << flag_pub_first_yaw_ << " C=" << odom_yaw_ << " D=" << final_yaw_des_ << " E=" << yaw_diff(odom_yaw_, final_yaw_des_) << endl;
      ROS_DEBUG(ss.str().c_str());
      if (planPrimitive(true))
      {
        yaw_cmd_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "CRASH");
        ROS_INFO("Recover from crash! id=%d, crash_rec_stage_=1", planner_manager_->drone_id);
      }
      else if (!flag_pub_first_yaw_ || fabs(yaw_diff(odom_yaw_, final_yaw_des_)) < 0.05)
      {
        if (yaw_cmd_count_ < 4)
        {
          yaw_cmd_count_++;
          std_msgs::Float64 yaw_cmd;
          final_yaw_des_ = limit_yaw(odom_yaw_ + M_PI * 0.5);
          yaw_cmd.data = final_yaw_des_;
          yaw_cmd_pub_.publish(yaw_cmd);

          flag_pub_first_yaw_ = true;
        }
        else
        {
          yaw_cmd_count_ = 0;
          crash_rec_stage_ = 2;
          ROS_WARN("Crash recovery change to stage 2");
        }
      }
    }
    else if (crash_rec_stage_ == 2 && have_latest_safe_pt_)
    {
      if (flag_wait_crash_rec_)
      {
        if ((ros::Time::now() - crash_rec_start_time_).toSec() > 5.0) // 5.0s time out
        {
          flag_wait_crash_rec_ = false;
          changeFSMExecState(CRASH_RECOVER, "FSM");
        }
        if ((latest_safe_pt_ - odom_pos_).norm() < 0.01)
        {
          flag_wait_crash_rec_ = false;
          ROS_INFO("Recover from crash! id=%d, latest_safe_pt_=(%f, %f, %f)",
                   planner_manager_->drone_id, latest_safe_pt_(0), latest_safe_pt_(1), latest_safe_pt_(2));
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
      }
      else
      {
        pubPolyTraj(odom_pos_, odom_vel_, latest_safe_pt_, 1.0);
        flag_wait_crash_rec_ = true;
        crash_rec_start_time_ = ros::Time::now();
      }
    }
    else
    {
      ROS_ERROR("crash_rec_stage_=%d, have_latest_safe_pt_=%d !!!", crash_rec_stage_, have_latest_safe_pt_);
    }
  }
  }

  exec_timer_.start();
}

void PPReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call)
{
  static std::string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "APPROACH_GOAL", "CRASH_RECOVER"};
  int pre_s = int(exec_state_);
  exec_state_ = new_state;
  ROS_DEBUG("[%s] Drone %d from %s to %s", pos_call.c_str(), planner_manager_->drone_id, state_str[pre_s].c_str(), state_str[int(new_state)].c_str());
}

void PPReplanFSM::printFSMExecState()
{
  static std::string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "APPROACH_GOAL", "CRASH_RECOVER"};

  std::string msg = "[FSM] Drone " + std::to_string(planner_manager_->drone_id) + " State: " + state_str[int(exec_state_)];

  // some warnings
  if (!have_odom_ || !have_target_ || !have_trigger_)
  {
    msg += ". Waiting for ";
  }
  if (!have_odom_)
  {
    msg += "odom,";
  }
  if (!have_target_)
  {
    msg += "target,";
  }
  if (!have_trigger_)
  {
    msg += "trigger,";
  }

  ROS_DEBUG(msg.c_str());
}

bool PPReplanFSM::planPrimitive(bool first_plan, double xV_offset /*= 0.0*/)
{
  ros::Time t0 = ros::Time::now();
#if USE_SHARED_MEMORY
  // share trajecotry using shared memory
  if (planner_manager_->swarm_traj.size() < TOTAL_DRONE_NUM_)
  {
    for (size_t i = planner_manager_->swarm_traj.size(); i < TOTAL_DRONE_NUM_; i++)
    {
      LocalTrajData blank;
      blank.drone_id = -1;
      blank.start_time = 0.0;
      planner_manager_->swarm_traj.push_back(blank);
    }
  }
  for (int i = 0; i < TOTAL_DRONE_NUM_; ++i)
  {
    if (!swarm_traj_ptr_[i].first)
    {

      if (swarm_traj_ptr_[i].second.drone_id != -1 && std::abs(swarm_traj_ptr_[i].second.start_time.toSec() - planner_manager_->swarm_traj[i].start_time) > 0.001)
      {
        traj_utils::swarmPrimitiveTrajPtr traj_ptr = boost::make_shared<traj_utils::swarmPrimitiveTraj>();
        std::memcpy(traj_ptr.get(), &(swarm_traj_ptr_[i].second), sizeof(traj_utils::swarmPrimitiveTraj));
        RecvBroadcastPrimitiveCallback(traj_ptr);
      }
    }
  }
  // ros::Time t1 = ros::Time::now();
#endif

  start_time_ = ros::Time::now();
  // build velocity frame {V}
  Eigen::Vector3d xV;
  // 0.05 not 0: because small value(0-0.05)'coordinate system is singular.
  if (first_plan || (odom_vel_.norm() < 0.05))
  {
    start_pt_ = odom_pos_;
    start_v_ = odom_vel_;
    Eigen::AngleAxisd rotation_angle_axis(xV_offset, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond rotation_quaternion(rotation_angle_axis);
    odom_x_dir_ = rotation_quaternion * odom_x_dir_;
    xV = odom_x_dir_.normalized();
  }
  else
  {
    // velocity = 0
    xV = start_v_.normalized();
  }
  Eigen::Vector3d yV = (xV.cross(Eigen::Vector3d(0, 0, -1))).normalized();
  Eigen::Vector3d zV = xV.cross(yV);
  Eigen::Matrix3d RWV;
  RWV.col(0) = xV;
  RWV.col(1) = yV;
  RWV.col(2) = zV;

  vector<int> select_path_id;
  bool plan_success = planner_manager_->trajReplan(start_pt_, start_v_, start_time_.toSec(), RWV, global_goal_, select_path_id, myself_traj_);

  std::vector<Eigen::Vector3d> traj_pos;
  double traj_duration;
  int vel_id = std::min(int(round(start_v_.norm() * 10)), int(planner_manager_->max_vel_ * 10));
  if (plan_success)
  {
    plan_success &= readLocalTrajPos(start_pt_, vel_id, RWV, select_path_id, traj_pos, traj_duration);
  }
  if (plan_success)
  {
    // if ( have_log_files_ )
    // {
    //   ros::Time end_time = ros::Time::now();
    //   std::vector<double> data;
    //   data.push_back((end_time - start_time_).toSec());
    //   std::string csv_data = computing_time_logger_.toCSV(start_time_, data);
    //   computing_time_logger_.appendData(csv_data);
    // }

    myself_traj_.drone_id = planner_manager_->drone_id;
    myself_traj_.start_time = start_time_.toSec();
    myself_traj_.traj_duration = traj_duration;
    myself_traj_.traj_pos = traj_pos;

    traj_msg.drone_id = planner_manager_->drone_id;
    traj_msg.start_time = start_time_;
    traj_msg.start_p[0] = start_pt_(0), traj_msg.start_p[1] = start_pt_(1), traj_msg.start_p[2] = start_pt_(2);
    traj_msg.start_v[0] = start_v_(0), traj_msg.start_v[1] = start_v_(1), traj_msg.start_v[2] = start_v_(2);
    traj_msg.end_p[0] = traj_pos.back()(0), traj_msg.end_p[1] = traj_pos.back()(1), traj_msg.end_p[2] = traj_pos.back()(2);
    traj_msg.rot_mat[0] = xV(0), traj_msg.rot_mat[1] = yV(0), traj_msg.rot_mat[2] = zV(0);
    traj_msg.rot_mat[3] = xV(1), traj_msg.rot_mat[4] = yV(1), traj_msg.rot_mat[5] = zV(1);
    traj_msg.rot_mat[6] = xV(2), traj_msg.rot_mat[7] = yV(2), traj_msg.rot_mat[8] = zV(2);
    traj_msg.vel_id = vel_id;
    traj_msg.hovering_at_goal = false;

    traj_msg.select_path_id = select_path_id[0];

    // publish own goal for decentralized information propagation
    if (flight_type_ == 3)
    {
      traj_msg.goal_tag = goal_tag_;
      traj_msg.goal[0] = global_goal_[0];
      traj_msg.goal[1] = global_goal_[1];
      traj_msg.goal[2] = global_goal_[2];
    }

    path_id_pub_.publish(traj_msg);
#if not USE_SHARED_MEMORY
    broadcast_primitive_pub_.publish(traj_msg);
#else
    if (planner_manager_->drone_id >= 0 && planner_manager_->drone_id < TOTAL_DRONE_NUM_)
    {
      swarm_traj_ptr_[planner_manager_->drone_id].first = true;
      swarm_traj_ptr_[planner_manager_->drone_id].second = traj_msg;
      swarm_traj_ptr_[planner_manager_->drone_id].first = false;
      ROS_INFO("Share Traj %d, t=%d.%d", planner_manager_->drone_id, start_time_.sec, start_time_.nsec);
    }
    else
    {
      ROS_ERROR("planner_manager_->drone_id >= 0 && planner_manager_->drone_id < TOTAL_DRONE_NUM_");
    }
#endif

    // vis
    std::vector<Eigen::Vector3d> traj_pos_vis;
    traj_pos_vis.push_back(traj_pos[0]);
    for (size_t i = 1; i < traj_pos.size(); ++i)
      if ((traj_pos[i] - traj_pos_vis.back()).norm() > 0.1)
        traj_pos_vis.push_back(traj_pos[i]);
    traj_pos_vis.push_back(traj_pos.back());
    visualization_->displayOptimalList(traj_pos_vis, 0);
  }

  ros::Time t2 = ros::Time::now();
  ROS_DEBUG("\033[44;97mDroneID=%d, plan_time=%.2f ms \033[0m\n", planner_manager_->drone_id, (t2 - t0).toSec() * 1000);
  return plan_success;
}

bool PPReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
{
  std_msgs::Float64MultiArray stop_msg;
  for (int i = 0; i < 3; i++)
    stop_msg.data.push_back(stop_pos(i));
  stop_pub_.publish(stop_msg);

  return true;
}

void PPReplanFSM::mandatoryStopCallback(const std_msgs::Empty &msg)
{
  ROS_ERROR("Received a mandatory stop command!");
  changeFSMExecState(EMERGENCY_STOP, "Mandatory Stop");
  enable_fail_safe_ = false;
}

void PPReplanFSM::pubPolyTraj(const Eigen::Vector3d &start_p, const Eigen::Vector3d &start_v, const Eigen::Vector3d &end_p, const double dura)
{
  double t1 = dura, t2 = t1 * t1, t3 = t2 * t1, t4 = t3 * t1, t5 = t4 * t1;
  Eigen::Matrix<double, 6, 6> T;
  T << 0, 0, 0, 0, 0, 1,
      0, 0, 0, 0, 1, 0,
      0, 0, 0, 2, 0, 0,
      t5, t4, t3, t2, t1, 1,
      5 * t4, 4 * t3, 3 * t2, 2 * t1, 1, 0,
      20 * t3, 12 * t2, 6 * t1, 2, 0, 0;
  Eigen::Matrix<double, 6, 3> B = Eigen::Matrix<double, 6, 3>::Zero();
  B.row(0) = start_p.transpose();
  B.row(1) = start_v.transpose();
  B.row(3) = end_p.transpose();
  Eigen::Matrix<double, 3, 6, Eigen::RowMajor> coeff = (T.inverse() * B).transpose();

  traj_utils::Polynomial poly_traj;
  memcpy(&poly_traj.coeff_x, coeff.row(0).data(), 6 * 8);
  memcpy(&poly_traj.coeff_y, coeff.row(1).data(), 6 * 8);
  memcpy(&poly_traj.coeff_z, coeff.row(2).data(), 6 * 8);
  poly_traj.start_time = ros::Time::now();
  poly_traj.duration = t1;
  poly_pub_.publish(poly_traj);
}

} // namespace primitive_planner