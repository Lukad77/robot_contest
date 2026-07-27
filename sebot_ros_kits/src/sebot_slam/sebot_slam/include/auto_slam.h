/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#ifndef __AUTO_SLAM__
#define __AUTO_SLAM__

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/MarkerArray.h>

#include <frontier_search.h>
#include <map2costmap.h>

namespace autoSlam
{
  enum class SlamState
  {
    EXPLORING,
    RETURNING_HOME,
    FINISHED
  };

  class AutoSlam
  {
  public:
    AutoSlam();
    ~AutoSlam();

    void start();
    void stop();

    bool debug;
    bool audio;
    std::string audioNum = "1";

  private:
    void makePlan();
    void visualizeFrontiers(const std::vector<frontier_exploration::Frontier> &frontiers);
    void reachedGoal(const actionlib::SimpleClientGoalState &status,
                     const move_base_msgs::MoveBaseResultConstPtr &result,
                     const geometry_msgs::Point &frontier_goal);
    void sendHomeGoal();
    void reachedHome(const actionlib::SimpleClientGoalState &status,
                     const move_base_msgs::MoveBaseResultConstPtr &result);
    bool goalOnBlacklist(const geometry_msgs::Point &goal);
    bool isGoalReachable(const geometry_msgs::Point &p);
    bool findReachableGoalNear(const geometry_msgs::Point &seed,
                               geometry_msgs::Point &selected_goal,
                               unsigned int search_radius_cells);
    bool selectFrontierGoal(const frontier_exploration::Frontier &frontier,
                            geometry_msgs::Point &selected_goal);
    bool isSameGoal(const geometry_msgs::Point &a,
                    const geometry_msgs::Point &b,
                    double tolerance);
    void backHome();

    ros::NodeHandle privateNode;
    ros::NodeHandle relativeNode;
    ros::Publisher pubMarkerArray;
    ros::Publisher pubAudio;
    ros::ServiceClient clear_costmap_client;
    tf::TransformListener listenerTF;

    Costmap2DClient costmapClient;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client;
    frontier_exploration::FrontierSearch frontierSearch;
    ros::Timer autoSlamTime;
    ros::Timer oneshot;

    SlamState state = SlamState::EXPLORING;
    std::vector<geometry_msgs::Point> frontierBlacklist;
    geometry_msgs::Point prevGoal;
    double prevDistance;
    ros::Time lastProgress;
    size_t lastMarkersCount;

    double minFrontierSize;
    double planFrequency;
    double potentialScale;
    double gainScale;
    double timeout;
    ros::Duration progressTimeout;
    bool visualize;
    bool finish = false;
    bool getInitial = false;
    bool hasPrevGoal = false;
    geometry_msgs::Pose initial_pose;
    int home_retry_count = 0;
    int max_home_retry = 3;
    int empty_frontier_count = 0;
    int max_empty_frontier_count = 5;
    std::string mapSavePath;

    std::chrono::high_resolution_clock::time_point startTime = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point endTime = std::chrono::high_resolution_clock::now();
  };
}

#endif
