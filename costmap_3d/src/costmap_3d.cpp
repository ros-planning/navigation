/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Badger Technologies LLC
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
 * Author: C. Andy Martin
 *********************************************************************/

#include <costmap_3d/costmap_3d.h>

namespace costmap_3d
{

Costmap3D::Costmap3D(double resolution) : octomap::OcTree(resolution)
{
  init();
}

Costmap3D::Costmap3D(const Costmap3D& rhs) : octomap::OcTree(rhs)
{
  // Be sure to init() on copy, as the octomap copy constructor does not do this
  init();
}

void Costmap3D::init()
{
  setProbHit(octomap::probability(LETHAL-FREE));
  setProbMiss(octomap::probability(FREE-LETHAL));
  setClampingThresMin(octomap::probability(FREE));
  setClampingThresMax(octomap::probability(LETHAL));
  // By default, only lethal cells are considered occupied
  setOccupancyThres(octomap::probability(LETHAL));
  // TODO: FCL has an OcTree wrapper class that defines both a free threshold
  // and an occupied threshold.
  // It copies the lethal threshold, but defaults to a 0 probability for the
  // free threshold (i.e., cells are never free!).
  // However, the MoveIt interfaces currently provide no way to set the FCL
  // properties.
  // We need a way to change this to 0.5 (0 log odds) to get the cost to be
  // accurate.
  // This isn't a big deal for non-cost queries (such as binary collision or
  // distance queries).
}

octomap::point3d toOctomapPoint(const geometry_msgs::Point& point)
{
  return octomap::point3d(point.x, point.y, point.z);
}

geometry_msgs::Point fromOctomapPoint(const octomap::point3d& point)
{
  geometry_msgs::Point p;
  p.x = point.x();
  p.y = point.y();
  p.z = point.z();
  return p;
}

}  // namespace costmap_3d
