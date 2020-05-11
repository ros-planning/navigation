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

/**
 * createPlan() creates a vector of poses
 * for 10m but global path is curved
*/
std::vector<geometry_msgs::PoseStamped> createCurvedPlan() {
    std::vector<geometry_msgs::PoseStamped> out;

    for (size_t i = 0; i < 10; i++) {
        geometry_msgs::PoseStamped p0;

        if( i > 5) {
            p0.pose.position.x = 5;
            p0.pose.position.y = i;
            p0.pose.position.z = 0;
            p0.pose.orientation.x = 0;
            p0.pose.orientation.y = 0;
            p0.pose.orientation.z = 0.7;
            p0.pose.orientation.w = 0.7;
        } else {
            p0.pose.position.x = (i+1);
            p0.pose.position.y = 1;
            p0.pose.orientation.x = 0;
            p0.pose.orientation.y = 0;
            p0.pose.orientation.z = 0;
            p0.pose.orientation.w = 1;
        }
        
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
    
    return path_speed_limiter;
}

/**
 * Test 1: Check which speed limiter to use
 * Test 2: If path starts behind the robot, 
 * then calculateLimitsFromPathAndPose shouldn't calculate heading diff
 * and apply speed limit to it.
*/
TEST(PathSpeedLimiter, heading_diff)
{
    auto psl = setupAndCreatePSL();
    psl.setPlan(createPlan());
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

/**
 * Test 1: If path is curved, path limiter should limit the velocity
 * by setting the linear velocity to min_linear_velocity 
 * set by PathSpeedLimiterConfig in setupAndCreatePSL function 
*/
TEST(PathSpeedLimiter, limiter_in_action)
{
    auto psl = setupAndCreatePSL();
    psl.setPlan(createCurvedPlan());
    
    double v_lim = 0, w_lim = 0;
    geometry_msgs::PoseStamped currentRobotPose;
    currentRobotPose.pose.position.x = 4;
    currentRobotPose.pose.position.y = 1;
    currentRobotPose.pose.position.z = 0;
    currentRobotPose.pose.orientation.x = 0;
    currentRobotPose.pose.orientation.y = 0;
    currentRobotPose.pose.orientation.z = 0.5;
    currentRobotPose.pose.orientation.w = 0.8660254;

    bool result = psl.calculateLimitsFromPathAndPose(v_lim, w_lim, currentRobotPose);
    EXPECT_EQ(v_lim, 0.12);
}

}
