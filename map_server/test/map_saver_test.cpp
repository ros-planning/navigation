/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Wang Jinge */

#include "ros/ros.h"
#include "gtest/gtest.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include "map_server/map_mode.h"
#include "map_server/map_generator.h"
#include "map_server/image_loader.h"

class MapSaverTest : public testing::Test
{
public:

    MapSaverTest()
    {
    }

    ~MapSaverTest()
    {
    }

    nav_msgs::OccupancyGrid generateMap() {
        nav_msgs::OccupancyGrid map;
        map.info.width = 100;
        map.info.height = 100;
        map.info.resolution = 0.05;
        map.info.origin.position.x = -3.2;
        map.info.origin.position.y = -10.736029;
        map.info.origin.position.z = 0.0;
        map.info.origin.orientation.x = 0.0;
        map.info.origin.orientation.y = 0.0;
        map.info.origin.orientation.z = 0.0;
        map.info.origin.orientation.w = 1.0;
        map.data.resize(map.info.width * map.info.height);
        for (int i=0; i<map.info.height; i++) {
            for (int j=0; j<map.info.width; j++) {
                map.data[i * map.info.width + j] = (i * 24678 + j * 32491) % 200 > 100 ? -1 : (i * 24678 + j * 32491) % 200;
            }
        }
        ROS_INFO("Map generated.");
        return map;
    }

    void publishMap(nav_msgs::OccupancyGrid map) {
        ros::Publisher publisher = n.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
        publisher.publish(map);
        ROS_INFO("Map published.");
    }

    bool equals(nav_msgs::OccupancyGrid map1, nav_msgs::OccupancyGrid map2) {
        if (map1.data.size() != map2.data.size()) {
            return false;
        }
        for (size_t i=0; i<map1.data.size(); i++) {
            if (abs((int)map1.data[i] - (int)map2.data[i]) > 1) {
                ROS_ERROR("map1.data(%d) != map2.data(%d)", map1.data[i], map2.data[i]);
                return false;
            }
        }
        return true;
    }

private:
    ros::NodeHandle n;
};

TEST_F(MapSaverTest, saveScaleMode)
{
    map_server::MapGenerator mapGenerator("saved_map", 100, 0, map_server::SCALE);

    nav_msgs::OccupancyGrid map = generateMap();
    publishMap(map);

    while(!mapGenerator.Saved() && ros::ok())
        ros::spinOnce();

    nav_msgs::GetMap::Response mapResponse;
    double origin[] = {map.info.origin.position.x, map.info.origin.position.y, map.info.origin.position.z};
    map_server::loadMapFromFile(&mapResponse, "saved_map.png", map.info.resolution, false, 1.0, 0.0, origin, map_server::SCALE);

    EXPECT_TRUE(equals(map, mapResponse.map));
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "map_saver_test");
    return RUN_ALL_TESTS();
}