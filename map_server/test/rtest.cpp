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
#include <ros/node.h>
#include <nav_msgs/GetMap.h>

#include "test_constants.h"

int g_argc;
char** g_argv;

class MapClientTest : public testing::Test
{
  private:
    // A node is needed to make a service call
    ros::NodeHandle* n;

  protected:
    virtual void SetUp()
    {
      ros::init(g_argc, g_argv, "map_client_test");
      n = new ros::NodeHandle();
    }
    virtual void TearDown()
    {
      
      delete n;
    }
};

/* Try to retrieve the map via a valid PNG file.  Succeeds if no
 * exception is thrown, if the call succeeds, and if
 * the loaded image matches the known dimensions of the map.  */
TEST_F(MapClientTest, retrieve_valid_bmp)
{
  try
  {
    nav_msgs::GetMap::Request  req;
    nav_msgs::GetMap::Response resp;
    // Try a few times, because the server may not be up yet.
    int i=10;
    bool call_result;
    while(i > 0)
    {
      call_result = ros::service::call("static_map", req, resp);
      if(call_result)
        break;
      else
      {
        ros::Duration d = ros::Duration().fromSec(0.25);
        d.sleep();
      }
    }
    ASSERT_TRUE(call_result);
    ASSERT_FLOAT_EQ(resp.map.info.resolution, g_valid_image_res);
    ASSERT_EQ(resp.map.info.width, g_valid_image_width);
    ASSERT_EQ(resp.map.info.height, g_valid_image_height);
    for(unsigned int i=0; i < resp.map.info.width * resp.map.info.height; i++)
      ASSERT_EQ(g_valid_image_content[i], resp.map.data[i]);
  }
  catch(...)
  {
    FAIL() << "Uncaught exception : " << "This is OK on OS X";
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  g_argc = argc;
  g_argv = argv;

  return RUN_ALL_TESTS();
}
