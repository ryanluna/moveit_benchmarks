/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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
*********************************************************************/

/* Author: Ryan Luna */

#ifndef MOVEIT_BENCHMARKS_BENCHMARK_SERVER_
#define MOVEIT_BENCHMARKS_BENCHMARK_SERVER_

#include "moveit/benchmarks/BenchmarkOptions.h"

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/warehouse/planning_scene_storage.h>
#include <moveit/warehouse/planning_scene_world_storage.h>
#include <moveit/warehouse/state_storage.h>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit/warehouse/trajectory_constraints_storage.h>
#include <moveit/planning_interface/planning_interface.h>
#include <pluginlib/class_loader.h>

#include <map>
#include <vector>
#include <string>

namespace moveit_benchmarks
{

/// A class that executes motion plan requests and aggregates data across multiple runs
/// Note: This class operates outside of MoveGroup and does NOT use PlanningRequestAdapters
class BenchmarkServer
{
public:
    BenchmarkServer(const std::string& robot_description_param="robot_description");
    virtual ~BenchmarkServer();

    virtual void clear();

    virtual bool runBenchmarks(const BenchmarkOptions& opts);

protected:

    /// Structure to hold information for a single call to 'solve'
    typedef std::map<std::string, std::string> PlannerRunData;
    /// Structure to hold information for a single planner's benchmark data.
    typedef std::vector<PlannerRunData> PlannerBenchmarkData;

    struct BenchmarkRequest
    {
        std::string name;
        moveit_msgs::MotionPlanRequest request;
    };

    struct StartState
    {
        moveit_msgs::RobotState state;
        std::string name;
    };

    struct PathConstraints
    {
        std::vector<moveit_msgs::Constraints> constraints;
        std::string name;
    };

    struct TrajectoryConstraints
    {
        moveit_msgs::TrajectoryConstraints constraints;
        std::string name;
    };

    virtual bool initializeBenchmarks(const BenchmarkOptions& opts, moveit_msgs::PlanningScene& scene_msg,
                                      std::vector<BenchmarkRequest>& queries);

    virtual void collectMetrics(PlannerRunData& metrics, const planning_interface::MotionPlanDetailedResponse& mp_res,
                                bool solved, double total_time);

    virtual void writeOutput(const BenchmarkRequest& brequest, const std::string& start_time,
                             double benchmark_duration);

    /// Invoked before each run of a benchmark
    virtual void preRunEvent(moveit_msgs::MotionPlanRequest& request) {}
    /// Invoked after each run of a benchmark
    virtual void postRunEvent(const moveit_msgs::MotionPlanRequest& request,  planning_interface::MotionPlanDetailedResponse& response) {}

    /// Invoked during benchmarking when the planner is changed
    virtual void plannerSwitchEvent(moveit_msgs::MotionPlanRequest& request) {}

    /// Invoked when the benchmark problem is changed (e.g., when running multiple queries,
    /// this is called before each query starts benchmarking)
    virtual void querySwitchEvent(moveit_msgs::MotionPlanRequest& request) {}

    /// Check that the desired planner plugins and algorithms exist for the given group
    bool plannerConfigurationsExist(const std::map<std::string, std::vector<std::string> >& planners, const std::string& group_name);

    /// Check that the given requests can be run on the set of planner plugins and algorithms
    bool queriesAndPlannersCompatible(const std::vector<BenchmarkRequest>& requests,
                                      const std::map<std::string, std::vector<std::string> >& planners);

    /// Load the planning scene with the given name from the warehouse
    bool loadPlanningScene(const std::string& scene_name, moveit_msgs::PlanningScene& scene_msg);

    /// Load all states matching the given regular expression from the warehouse
    bool loadStates(const std::string& regex, std::vector<StartState>& start_states);

    /// Load all constraints matching the given regular expression from the warehouse
    bool loadPathConstraints(const std::string& regex, std::vector<PathConstraints>& constraints);

    /// Load all trajectory constraints from the warehouse that match the given regular expression
    bool loadTrajectoryConstraints(const std::string& regex, std::vector<TrajectoryConstraints>& constraints);

    /// Load all motion plan requests matching the given regular expression from the warehouse
    bool loadQueries(const std::string& regex, const std::string& scene_name,
                     std::vector<BenchmarkRequest>& queries);

    /// Duplicate the given benchmark request for all combinations of start states and path constraints
    void createRequestCombinations(const BenchmarkRequest& brequest, const std::vector<StartState>& start_states,
                                   const std::vector<PathConstraints>& path_constraints, std::vector<BenchmarkRequest>& combos);

    /// Execute the given motion plan request on the set of planners for the set number of runs
    void runBenchmark(moveit_msgs::MotionPlanRequest request, const std::map<std::string, std::vector<std::string> >& planners, int runs);

    planning_scene_monitor::PlanningSceneMonitor* psm_;
    moveit_warehouse::PlanningSceneStorage* pss_;
    moveit_warehouse::PlanningSceneWorldStorage* psws_;
    moveit_warehouse::RobotStateStorage* rs_;
    moveit_warehouse::ConstraintsStorage* cs_;
    moveit_warehouse::TrajectoryConstraintsStorage* tcs_;

    planning_scene::PlanningScenePtr planning_scene_;

    BenchmarkOptions options_;

    boost::shared_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader_;
    std::map<std::string, planning_interface::PlannerManagerPtr> planner_interfaces_;

    std::vector<PlannerBenchmarkData> benchmark_data_;
};

}

#endif