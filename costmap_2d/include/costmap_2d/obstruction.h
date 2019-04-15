/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2017, 6 River Systems, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Daniel Grieneisen
 *********************************************************************/
#ifndef COSTMAP_2D_OBSTRUCTION_H_
#define COSTMAP_2D_OBSTRUCTION_H_

#include <ros/ros.h>
#include <costmap_2d/ObstructionMsg.h>


namespace costmap_2d
{

enum class ObstructionType
{
  STATIC,
  DYNAMIC,
  PSEUDOSTATIC
};

/**
 * Struct to hold information about things in the world seen by sensors.
 */
struct Obstruction
{
  // Obstruction(){};  // No default constructor

  /**
   * Constructor
   * @param x The x location (in meters) of the obstruction
   * @param y The y location (in meters) of the obstruction
   * @param type The type of obstruction
   * @param frame The frame in which the x and y params are defined
   */
  Obstruction(float x, float y, ObstructionType type, std::string frame, std::string costmap_name) : x_(x), y_(y),
    type_(type), frame_(frame), first_sighting_time_(ros::Time::now()),
    last_sighting_time_(ros::Time::now()), last_level_time_(ros::Time::now()),
    seen_this_cycle_(true), updated_(true), costmap_name_(costmap_name) {}

  /**
   * Mark the obstruction as seen again.
   */
  void touch(ObstructionType type)
  {
    last_sighting_time_ = ros::Time::now();
    last_level_time_ = last_sighting_time_;
    cleared_ = false;
    seen_this_cycle_ = true;
    type_ = type;
    if (level_ != 0)
    {
      level_ = 0;
      updated_ = true;
    }
  }

  ros::Time first_sighting_time_ = ros::Time(0); // time obstruction was first seen
  ros::Time last_sighting_time_ = ros::Time(0);  // last time obstruction was seen
  ros::Time last_level_time_ = ros::Time(0);  // last time obstruction changed levels
  float x_ = 0.0; // x location of obstruction
  float y_ = 0.0; // y location of obstruction
  unsigned int level_ = 0; // obstruction level
  ObstructionType type_ = ObstructionType::DYNAMIC; // type of the obstruction
  bool cleared_ = false; // flag to indicate if the obstruction is cleared and should be removed
  bool seen_this_cycle_ = false; // flag to indicate if the obstruction was seen this cycle
  bool updated_ = false; // flag to indicate if something about the obstruction was updated
  float radius_ = -1; // the effective radius
  unsigned char max_cost_ = 255;
  std::string frame_ = ""; // the frame in which it is defined
  std::string costmap_name_ = ""; //the costmap reporting this obstruction
};

/**
 * Adapter class to convert obstructions
 */
class ObstructionAdapter
{
public:
  static ObstructionMsg obstructionToMsg(const Obstruction& obs)
  {
    ObstructionMsg msg;
    msg.x = obs.x_;
    msg.y = obs.y_;
    msg.last_sighting_time = obs.last_sighting_time_;
    msg.first_sighting_time = obs.first_sighting_time_;
    msg.cleared = obs.cleared_;
    msg.effective_radius = obs.radius_;
    msg.max_cost = obs.max_cost_;
    msg.frame_id = obs.frame_;
    msg.costmap_name = obs.costmap_name_;
    switch (obs.type_)
    {
      case ObstructionType::DYNAMIC:
        msg.type = ObstructionMsg::DYNAMIC;
        break;
      case ObstructionType::PSEUDOSTATIC:
        msg.type = ObstructionMsg::PSEUDOSTATIC;
        break;
      case ObstructionType::STATIC:
        msg.type = ObstructionMsg::STATIC;
        break;
    }
    if (obs.cleared_)
    {
      msg.updated = true;
    }
    else
    {
      msg.updated = obs.updated_;
    }
    return msg;
  }
};

}

#endif // COSTMAP_2D_OBSTRUCTION_H_
