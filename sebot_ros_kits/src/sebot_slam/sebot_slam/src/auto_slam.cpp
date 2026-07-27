/**
 * @file auto_slam.cpp
 * @brief Autonomous frontier exploration and return-home workflow.
 */

#include <auto_slam.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <thread>

#include <costmap_2d/cost_values.h>
#include <ros/package.h>

#include "std_msgs/String.h"

namespace autoSlam
{
  namespace
  {
    std::string shellQuote(const std::string &value)
    {
      std::string quoted = "'";
      for (const char ch : value)
      {
        if (ch == '\'')
        {
          quoted += "'\\''";
        }
        else
        {
          quoted += ch;
        }
      }
      quoted += "'";
      return quoted;
    }

    std::string parentDir(const std::string &path)
    {
      const std::size_t pos = path.find_last_of('/');
      if (pos == std::string::npos)
      {
        return ".";
      }
      return path.substr(0, pos);
    }
  } // namespace

  AutoSlam::AutoSlam()
      : privateNode("~"),
        listenerTF(ros::Duration(10.0)),
        costmapClient(privateNode, relativeNode, &listenerTF),
        move_base_client("move_base"), prevDistance(0), lastMarkersCount(0)
  {
    privateNode.param("planner_frequency", planFrequency, 0.33);
    privateNode.param("progress_timeout", timeout, 30.0);
    progressTimeout = ros::Duration(timeout);
    privateNode.param("visualize", visualize, false);
    privateNode.param("potential_scale", potentialScale, 1e-3);
    privateNode.param("gain_scale", gainScale, 1.0);
    privateNode.param("min_frontier_size", minFrontierSize, 0.5);
    privateNode.param("debug", debug, false);
    privateNode.param("audio", audio, false);
    privateNode.param("max_empty_frontier_count", max_empty_frontier_count, 5);
    const std::string navigationPackagePath = ros::package::getPath("sebot_navigation");
    const std::string defaultMapSavePath = navigationPackagePath.empty() ? "map" : navigationPackagePath + "/map/map";
    privateNode.param<std::string>("map_save_path", mapSavePath, defaultMapSavePath);

    clear_costmap_client = relativeNode.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

    frontierSearch = frontier_exploration::FrontierSearch(costmapClient.getCostmap(), potentialScale, gainScale, minFrontierSize);

    if (visualize)
    {
      pubMarkerArray = privateNode.advertise<visualization_msgs::MarkerArray>("frontiers", 10);
    }

    if (audio)
    {
      pubAudio = privateNode.advertise<std_msgs::String>("/audio", 2);
    }

    ROS_INFO("Waiting to connect to move_base server");
    move_base_client.waitForServer();
    ROS_INFO("Connected to move_base server");

    if (audio)
    {
      std_msgs::String msg;
      msg.data = "startSlam";
      pubAudio.publish(msg);
    }

    autoSlamTime = relativeNode.createTimer(
        ros::Duration(1. / planFrequency),
        [this](const ros::TimerEvent &)
        {
          makePlan();
        },
        false, false);
  }

  AutoSlam::~AutoSlam()
  {
    if (state != SlamState::FINISHED)
    {
      stop();
    }
  }

  void AutoSlam::visualizeFrontiers(const std::vector<frontier_exploration::Frontier> &frontiers)
  {
    std_msgs::ColorRGBA blue;
    std_msgs::ColorRGBA red;
    std_msgs::ColorRGBA green;
    blue.r = 0;
    blue.g = 0;
    blue.b = 1.0;
    blue.a = 1.0;
    red.r = 1.0;
    red.g = 0;
    red.b = 0;
    red.a = 1.0;
    green.r = 0;
    green.g = 1.0;
    green.b = 0;
    green.a = 1.0;

    ROS_DEBUG("visualising %lu frontiers", frontiers.size());
    visualization_msgs::MarkerArray markers_msg;
    std::vector<visualization_msgs::Marker> &markers = markers_msg.markers;
    visualization_msgs::Marker m;

    m.header.frame_id = costmapClient.getGlobalFrameID();
    m.header.stamp = ros::Time::now();
    m.ns = "frontiers";
    m.scale.x = 1.0;
    m.scale.y = 1.0;
    m.scale.z = 1.0;
    m.color.r = 0;
    m.color.g = 0;
    m.color.b = 255;
    m.color.a = 255;
    m.lifetime = ros::Duration(0);
    m.frame_locked = true;

    double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

    m.action = visualization_msgs::Marker::ADD;
    size_t id = 0;
    for (auto &frontier : frontiers)
    {
      m.type = visualization_msgs::Marker::POINTS;
      m.id = int(id);
      m.pose.position = {};
      m.scale.x = 0.1;
      m.scale.y = 0.1;
      m.scale.z = 0.1;
      m.points = frontier.points;
      if (goalOnBlacklist(frontier.centroid))
      {
        m.color = red;
      }
      else
      {
        m.color = blue;
      }
      markers.push_back(m);
      ++id;

      m.type = visualization_msgs::Marker::SPHERE;
      m.id = int(id);
      m.pose.position = frontier.initial;
      double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
      m.scale.x = scale;
      m.scale.y = scale;
      m.scale.z = scale;
      m.points = {};
      m.color = green;
      markers.push_back(m);
      ++id;
    }
    size_t current_markers_count = markers.size();

    m.action = visualization_msgs::Marker::DELETE;
    for (; id < lastMarkersCount; ++id)
    {
      m.id = int(id);
      markers.push_back(m);
    }

    lastMarkersCount = current_markers_count;
    pubMarkerArray.publish(markers_msg);
  }

  void AutoSlam::makePlan()
  {
    if (state != SlamState::EXPLORING)
    {
      return;
    }

    auto pose = costmapClient.getRobotPose();
    auto frontiers = frontierSearch.searchFrom(pose.position);
    ROS_DEBUG("found %lu frontiers", frontiers.size());
    for (size_t i = 0; i < frontiers.size(); ++i)
    {
      ROS_DEBUG("frontier %zd cost: %f", i, frontiers[i].cost);
    }

    if (visualize)
    {
      visualizeFrontiers(frontiers);
    }

    if (!getInitial)
    {
      initial_pose = pose;
      getInitial = true;
    }

    if (frontiers.empty())
    {
      ++empty_frontier_count;
      ROS_WARN("No frontiers found: %d / %d", empty_frontier_count, max_empty_frontier_count);
      if (empty_frontier_count >= max_empty_frontier_count)
      {
        ROS_INFO("No frontiers found for several cycles. Start returning home.");
        backHome();
      }
      return;
    }

    geometry_msgs::Point targetPosition;
    geometry_msgs::Point targetBlacklistPoint;
    double targetMinDistance = 0.0;
    bool foundReachableFrontier = false;
    for (auto &frontier : frontiers)
    {
      if (goalOnBlacklist(frontier.centroid) || goalOnBlacklist(frontier.initial))
      {
        continue;
      }

      if (!selectFrontierGoal(frontier, targetPosition))
      {
        ROS_WARN("Selected frontier has no reachable goal. Add centroid to blacklist.");
        frontierBlacklist.push_back(frontier.centroid);
        continue;
      }

      targetMinDistance = frontier.minDistance;
      targetBlacklistPoint = frontier.centroid;
      foundReachableFrontier = true;
      break;
    }

    if (!foundReachableFrontier)
    {
      ++empty_frontier_count;
      ROS_WARN("No reachable frontiers found: %d / %d", empty_frontier_count, max_empty_frontier_count);
      if (empty_frontier_count >= max_empty_frontier_count)
      {
        ROS_INFO("No reachable frontiers found for several cycles. Start returning home.");
        backHome();
      }
      return;
    }
    empty_frontier_count = 0;

    bool same_goal = hasPrevGoal && isSameGoal(prevGoal, targetPosition, 0.15);
    prevGoal = targetPosition;
    hasPrevGoal = true;

    if (!same_goal || prevDistance > targetMinDistance)
    {
      lastProgress = ros::Time::now();
      prevDistance = targetMinDistance;
    }

    if (ros::Time::now() - lastProgress > progressTimeout)
    {
      ROS_WARN("Goal timeout. Add current frontier to blacklist.");
      frontierBlacklist.push_back(targetBlacklistPoint);
      prevDistance = std::numeric_limits<double>::infinity();
      lastProgress = ros::Time::now();
      return;
    }

    if (same_goal)
    {
      return;
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose.position = targetPosition;
    goal.target_pose.pose.orientation.w = 1.;
    goal.target_pose.header.frame_id = costmapClient.getGlobalFrameID();
    goal.target_pose.header.stamp = ros::Time::now();
    move_base_client.sendGoal(goal, [this, targetBlacklistPoint](
                                        const actionlib::SimpleClientGoalState &status,
                                        const move_base_msgs::MoveBaseResultConstPtr &result)
                              {
                                reachedGoal(status, result, targetBlacklistPoint);
                              });
    ROS_INFO("[---] publish goal !!!");
  }

  bool AutoSlam::isGoalReachable(const geometry_msgs::Point &p)
  {
    costmap_2d::Costmap2D *costmap = costmapClient.getCostmap();

    unsigned int mx, my;
    if (!costmap->worldToMap(p.x, p.y, mx, my))
    {
      ROS_WARN("Goal point is outside costmap.");
      return false;
    }

    unsigned char cost = costmap->getCost(mx, my);
    if (cost == costmap_2d::LETHAL_OBSTACLE ||
        cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
        cost == costmap_2d::NO_INFORMATION)
    {
      ROS_WARN("Goal point is not reachable. cost=%d", cost);
      return false;
    }

    return true;
  }

  bool AutoSlam::findReachableGoalNear(const geometry_msgs::Point &seed,
                                       geometry_msgs::Point &selected_goal,
                                       unsigned int search_radius_cells)
  {
    costmap_2d::Costmap2D *costmap = costmapClient.getCostmap();

    unsigned int seed_mx, seed_my;
    if (!costmap->worldToMap(seed.x, seed.y, seed_mx, seed_my))
    {
      return false;
    }

    const int size_x = static_cast<int>(costmap->getSizeInCellsX());
    const int size_y = static_cast<int>(costmap->getSizeInCellsY());
    const int center_x = static_cast<int>(seed_mx);
    const int center_y = static_cast<int>(seed_my);
    const int radius = static_cast<int>(search_radius_cells);

    bool found = false;
    unsigned int best_mx = seed_mx;
    unsigned int best_my = seed_my;
    double best_score = std::numeric_limits<double>::infinity();

    for (int dy = -radius; dy <= radius; ++dy)
    {
      for (int dx = -radius; dx <= radius; ++dx)
      {
        const int mx = center_x + dx;
        const int my = center_y + dy;
        if (mx < 0 || my < 0 || mx >= size_x || my >= size_y)
        {
          continue;
        }

        const unsigned char cost = costmap->getCost(static_cast<unsigned int>(mx), static_cast<unsigned int>(my));
        if (cost == costmap_2d::LETHAL_OBSTACLE ||
            cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
            cost == costmap_2d::NO_INFORMATION)
        {
          continue;
        }

        const double distance_score = static_cast<double>(dx * dx + dy * dy);
        const double cost_score = static_cast<double>(cost) / 255.0;
        const double score = distance_score + cost_score;
        if (score < best_score)
        {
          best_score = score;
          best_mx = static_cast<unsigned int>(mx);
          best_my = static_cast<unsigned int>(my);
          found = true;
        }
      }
    }

    if (!found)
    {
      return false;
    }

    costmap->mapToWorld(best_mx, best_my, selected_goal.x, selected_goal.y);
    selected_goal.z = 0.0;
    return true;
  }

  bool AutoSlam::selectFrontierGoal(const frontier_exploration::Frontier &frontier,
                                    geometry_msgs::Point &selected_goal)
  {
    if (isGoalReachable(frontier.centroid))
    {
      selected_goal = frontier.centroid;
      return true;
    }

    if (findReachableGoalNear(frontier.centroid, selected_goal, 12))
    {
      return true;
    }

    if (isGoalReachable(frontier.initial))
    {
      selected_goal = frontier.initial;
      return true;
    }

    if (findReachableGoalNear(frontier.initial, selected_goal, 12))
    {
      return true;
    }

    if (isGoalReachable(frontier.middle))
    {
      selected_goal = frontier.middle;
      return true;
    }

    if (findReachableGoalNear(frontier.middle, selected_goal, 12))
    {
      return true;
    }

    return false;
  }

  bool AutoSlam::isSameGoal(const geometry_msgs::Point &a,
                            const geometry_msgs::Point &b,
                            double tolerance)
  {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy) < tolerance;
  }

  bool AutoSlam::goalOnBlacklist(const geometry_msgs::Point &goal)
  {
    constexpr static size_t tolerace = 5;
    costmap_2d::Costmap2D *costmap2d = costmapClient.getCostmap();

    for (auto &frontier_goal : frontierBlacklist)
    {
      double x_diff = std::fabs(goal.x - frontier_goal.x);
      double y_diff = std::fabs(goal.y - frontier_goal.y);

      if (x_diff < tolerace * costmap2d->getResolution() &&
          y_diff < tolerace * costmap2d->getResolution())
      {
        return true;
      }
    }
    return false;
  }

  void AutoSlam::backHome()
  {
    if (state == SlamState::RETURNING_HOME || state == SlamState::FINISHED)
    {
      return;
    }

    if (audio)
    {
      std_msgs::String msg;
      msg.data = "finishSlam";
      pubAudio.publish(msg);
    }

    ROS_INFO("auto_slam frontier exploration has been completed.");
    ROS_INFO("Now prepare to return to the starting position.");
    state = SlamState::RETURNING_HOME;
    home_retry_count = 0;
    autoSlamTime.stop();
    oneshot.stop();
    move_base_client.cancelGoal();
    sendHomeGoal();
  }

  void AutoSlam::sendHomeGoal()
  {
    if (!getInitial)
    {
      ROS_ERROR("Cannot return home: initial pose has not been recorded.");
      state = SlamState::FINISHED;
      finish = false;
      stop();
      return;
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose = initial_pose;
    goal.target_pose.header.frame_id = costmapClient.getGlobalFrameID();
    goal.target_pose.header.stamp = ros::Time::now();

    move_base_client.sendGoal(goal, [this](
                                        const actionlib::SimpleClientGoalState &status,
                                        const move_base_msgs::MoveBaseResultConstPtr &result)
                              {
                                reachedHome(status, result);
                              });
    ROS_INFO("Published return-home goal.");
  }

  void AutoSlam::reachedHome(const actionlib::SimpleClientGoalState &status,
                             const move_base_msgs::MoveBaseResultConstPtr &)
  {
    if (state != SlamState::RETURNING_HOME)
    {
      return;
    }

    ROS_INFO("Return-home goal finished with status: %s", status.toString().c_str());
    if (status == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Robot returned to the initial pose successfully.");
      state = SlamState::FINISHED;
      finish = true;
      stop();
      return;
    }

    ++home_retry_count;
    if (home_retry_count <= max_home_retry)
    {
      ROS_WARN("Return-home failed, retry %d/%d after clearing costmaps.", home_retry_count, max_home_retry);
      std_srvs::Empty srv;
      if (clear_costmap_client.call(srv))
      {
        ROS_INFO("move_base costmaps cleared.");
      }
      else
      {
        ROS_WARN("Failed to call /move_base/clear_costmaps; retrying return-home anyway.");
      }

      ros::Duration(1.0).sleep();
      sendHomeGoal();
      return;
    }

    ROS_ERROR("Return-home failed after %d retries; map will not be saved as successful.", max_home_retry);
    state = SlamState::FINISHED;
    finish = false;
    stop();
  }

  void AutoSlam::reachedGoal(const actionlib::SimpleClientGoalState &status,
                             const move_base_msgs::MoveBaseResultConstPtr &,
                             const geometry_msgs::Point &frontier_goal)
  {
    if (state != SlamState::EXPLORING)
    {
      return;
    }

    ROS_DEBUG("Reached frontier goal with status: %s", status.toString().c_str());
    if (status == actionlib::SimpleClientGoalState::ABORTED ||
        status == actionlib::SimpleClientGoalState::REJECTED)
    {
      frontierBlacklist.push_back(frontier_goal);
      ROS_DEBUG("Adding current goal to black list");
    }

    oneshot = relativeNode.createTimer(
        ros::Duration(0, 0), [this](const ros::TimerEvent &)
        {
          makePlan();
        },
        true);
  }

  void AutoSlam::start()
  {
    state = SlamState::EXPLORING;
    finish = false;
    home_retry_count = 0;
    empty_frontier_count = 0;
    hasPrevGoal = false;
    prevDistance = std::numeric_limits<double>::infinity();
    lastProgress = ros::Time::now();
    autoSlamTime.start();
    startTime = std::chrono::high_resolution_clock::now();
  }

  void AutoSlam::stop()
  {
    if (audio)
    {
      std_msgs::String msg;
      msg.data = "backHome";
      pubAudio.publish(msg);
    }

    endTime = std::chrono::high_resolution_clock::now();
    move_base_client.cancelAllGoals();
    autoSlamTime.stop();
    oneshot.stop();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    if (finish && state == SlamState::FINISHED)
    {
      const std::string command = "mkdir -p " + shellQuote(parentDir(mapSavePath)) +
                                  " && rosrun map_server map_saver -f " + shellQuote(mapSavePath);
      const int ret = system(command.c_str());
      if (ret == 0)
      {
        ROS_INFO("Map saved successfully: %s", mapSavePath.c_str());
      }
      else
      {
        ROS_ERROR("Map save failed, return code: %d", ret);
      }
    }
    else
    {
      ROS_WARN("Auto slam did not finish successfully; skip map save.");
    }

    ROS_INFO("Exploration stopped.");
    ROS_INFO("Execution time: %ld s", duration.count() / 1000);
  }

} // namespace autoSlam

int main(int argc, char **argv)
{
  ros::init(argc, argv, "auto_slam");
  std::this_thread::sleep_for(std::chrono::seconds(5));
  autoSlam::AutoSlam autoSlam;
  if (autoSlam.debug)
  {
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
      ros::console::notifyLoggerLevelsChanged();
    }
  }

  autoSlam.start();
  ros::spin();

  return 0;
}
