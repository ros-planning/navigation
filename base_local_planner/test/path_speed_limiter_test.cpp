/**
 *path_speed_limiter_test.cpp
 *
 *      Author: Apoorva Gupta
 */
#include <gtest/gtest.h>
#include <base_local_planner/map_grid.h>
#include <base_local_planner/speed_limiters/path_speed_limiter.h>
#include "wavefront_map_accessor.h"

/**
 * createPlan() creates a vector of poses
 * for 10m long global plan
*/
std::vector<geometry_msgs::PoseStamped> createPlan() {
    std::vector<geometry_msgs::PoseStamped> out;

    for (size_t i = 0; i < 10; i++) {
        geometry_msgs::PoseStamped p0;
        p0.pose.position.x = (i+1);
        p0.pose.position.y = 1;
        p0.pose.position.z = 0;
        p0.pose.orientation.x = 0;
        p0.pose.orientation.y = 0;
        p0.pose.orientation.z = 0;
        p0.pose.orientation.w = 1;
        out.push_back(p0);
    }
    return out;
}

namespace base_local_planner {
/**
 * setup the config for path speed limiter(psl)
 * and create  path speed limiter object
 * set the global plan for psl and return it
*/
PathSpeedLimiter setupAndCreatePSL() {
    costmap_2d::Costmap2DROS* map(NULL);
    PathSpeedLimiter path_speed_limiter = PathSpeedLimiter(map);
    
    PathSpeedLimiterConfig cfg;
    cfg.max_distance_from_path = 0.5;
    cfg.max_lookahead_distance = 2.0;
    cfg.min_lookahead_distance = 1.0;
    cfg.min_heading_difference = 0.2;
    cfg.max_heading_difference = 1.0;
    cfg.min_linear_velocity = 0.12;
    path_speed_limiter.reconfigure(cfg);
    path_speed_limiter.setPlan(createPlan());
    return path_speed_limiter;
}

/**
 * Test 1: Check which speed limiter to use
 * Test 2: Check  if path starts behind the robot, 
 * then calculateLimitsFromPathAndPose shouldn't calculate heading diff
 * and apply speed limit to it.
*/
TEST(PathSpeedLimiter, heading_diff)
{
    auto psl = setupAndCreatePSL();
    
    double v_lim = 0, w_lim = 0;
    
    geometry_msgs::PoseStamped currentRobotPose;
    currentRobotPose.pose.position.x = 5;
    currentRobotPose.pose.position.y = 1;
    currentRobotPose.pose.position.z = 0;
    currentRobotPose.pose.orientation.x = 0;
    currentRobotPose.pose.orientation.y = 0;
    currentRobotPose.pose.orientation.z = 0;
    currentRobotPose.pose.orientation.w = 1;

    bool result = psl.calculateLimitsFromPathAndPose(v_lim, w_lim, currentRobotPose);

    EXPECT_EQ(psl.getName(), std::string("Path"));
    EXPECT_EQ(v_lim, 1.0);
}

}
