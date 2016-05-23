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

/* Author: Brian Gerkey */

#include <gtest/gtest.h>
#include <ros/service.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/LoadMap.h>

#include "test_constants.h"

int g_argc;
char** g_argv;

class MapClientTest : public testing::Test
{
  public:
    // A node is needed to make a service call
    ros::NodeHandle* n_;

    MapClientTest()
    {
      ros::init(g_argc, g_argv, "map_client_test");
      n_ = new ros::NodeHandle();
      got_map_ = false;
      got_map_metadata_ = false;
    }

    ~MapClientTest()
    {
      delete n_;
    }

    bool got_map_;
    boost::shared_ptr<nav_msgs::OccupancyGrid const> map_;
    void mapCallback(const boost::shared_ptr<nav_msgs::OccupancyGrid const>& map)
    {
      map_ = map;
      got_map_ = true;
    }

    bool got_map_metadata_;
    boost::shared_ptr<nav_msgs::MapMetaData const> map_metadata_;
    void mapMetaDataCallback(const boost::shared_ptr<nav_msgs::MapMetaData const>& map_metadata)
    {
      map_metadata_ = map_metadata;
      got_map_metadata_ = true;
    }
};

/* Try to retrieve the map via service, and compare to ground truth */
TEST_F(MapClientTest, call_service)
{
  nav_msgs::GetMap::Request  req;
  nav_msgs::GetMap::Response resp;
  ASSERT_TRUE(ros::service::waitForService("static_map", 5000));
  ASSERT_TRUE(ros::service::call("static_map", req, resp));
  ASSERT_FLOAT_EQ(resp.map.info.resolution, g_valid_image_res);
  ASSERT_EQ(resp.map.info.width, g_valid_image_width);
  ASSERT_EQ(resp.map.info.height, g_valid_image_height);
  ASSERT_STREQ(resp.map.header.frame_id.c_str(), "map");
  for(unsigned int i=0; i < resp.map.info.width * resp.map.info.height; i++)
    ASSERT_EQ(g_valid_image_content[i], resp.map.data[i]);
}

/* Try to retrieve the map via topic, and compare to ground truth */
TEST_F(MapClientTest, subscribe_topic)
{
  ros::Subscriber sub = n_->subscribe<nav_msgs::OccupancyGrid>("map", 1, boost::bind(&MapClientTest::mapCallback, this, _1));

  // Try a few times, because the server may not be up yet.
  int i=20;
  while(!got_map_ && i > 0)
  {
    ros::spinOnce();
    ros::Duration d = ros::Duration().fromSec(0.25);
    d.sleep();
    i--;
  }
  ASSERT_TRUE(got_map_);
  ASSERT_FLOAT_EQ(map_->info.resolution, g_valid_image_res);
  ASSERT_EQ(map_->info.width, g_valid_image_width);
  ASSERT_EQ(map_->info.height, g_valid_image_height);
  ASSERT_STREQ(map_->header.frame_id.c_str(), "map");
  for(unsigned int i=0; i < map_->info.width * map_->info.height; i++)
    ASSERT_EQ(g_valid_image_content[i], map_->data[i]);
}

/* Try to retrieve the metadata via topic, and compare to ground truth */
TEST_F(MapClientTest, subscribe_topic_metadata)
{
  ros::Subscriber sub = n_->subscribe<nav_msgs::MapMetaData>("map_metadata", 1, boost::bind(&MapClientTest::mapMetaDataCallback, this, _1));

  // Try a few times, because the server may not be up yet.
  int i=20;
  while(!got_map_metadata_ && i > 0)
  {
    ros::spinOnce();
    ros::Duration d = ros::Duration().fromSec(0.25);
    d.sleep();
    i--;
  }
  ASSERT_TRUE(got_map_metadata_);
  ASSERT_FLOAT_EQ(map_metadata_->resolution, g_valid_image_res);
  ASSERT_EQ(map_metadata_->width, g_valid_image_width);
  ASSERT_EQ(map_metadata_->height, g_valid_image_height);
}

/* Change the map, retrieve the map via topic, and compare to ground truth */
TEST_F(MapClientTest, change_map)
{
  ros::Subscriber sub = n_->subscribe<nav_msgs::OccupancyGrid>("map", 1, boost::bind(&MapClientTest::mapCallback, this, _1));

  // Try a few times, because the server may not be up yet.
  for (int i = 20; i > 0 && !got_map_; i--)
  {
    ros::spinOnce();
    ros::Duration d = ros::Duration().fromSec(0.25);
    d.sleep();
  }
  ASSERT_TRUE(got_map_);

  // Now change the map
  got_map_ = false;
  nav_msgs::LoadMap::Request  req;
  nav_msgs::LoadMap::Response resp;
  req.map_url = ros::package::getPath("map_server") + "/test/testmap2.yaml";
  ASSERT_TRUE(ros::service::waitForService("change_map", 5000));
  ASSERT_TRUE(ros::service::call("change_map", req, resp));
  ASSERT_EQ(0u, resp.result);
  for (int i = 20; i > 0 && !got_map_; i--)
  {
    ros::spinOnce();
    ros::Duration d = ros::Duration().fromSec(0.25);
    d.sleep();
  }

  ASSERT_FLOAT_EQ(tmap2::g_valid_image_res, map_->info.resolution);
  ASSERT_EQ(tmap2::g_valid_image_width, map_->info.width);
  ASSERT_EQ(tmap2::g_valid_image_height, map_->info.height);
  ASSERT_STREQ("map", map_->header.frame_id.c_str());
  for(unsigned int i=0; i < map_->info.width * map_->info.height; i++)
    ASSERT_EQ(tmap2::g_valid_image_content[i], map_->data[i]) << "idx:" << i;

  //Put the old map back so the next test isn't broken
  got_map_ = false;
  req.map_url = ros::package::getPath("map_server") + "/test/testmap.yaml";
  ASSERT_TRUE(ros::service::call("change_map", req, resp));
  ASSERT_EQ(0u, resp.result);
  for (int i = 20; i > 0 && !got_map_; i--)
  {
    ros::spinOnce();
    ros::Duration d = ros::Duration().fromSec(0.25);
    d.sleep();
  }
  ASSERT_TRUE(got_map_);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  g_argc = argc;
  g_argv = argv;

  return RUN_ALL_TESTS();
}
