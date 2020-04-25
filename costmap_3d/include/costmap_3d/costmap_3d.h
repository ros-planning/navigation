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
#ifndef COSTMAP_3D_COSTMAP_3D_H_
#define COSTMAP_3D_COSTMAP_3D_H_

#include <memory>
#include <octomap/octomap.h>
#include <geometry_msgs/Point.h>

namespace costmap_3d
{

enum CombinationMethod
{
  OVERWRITE = 0,
  MAXIMUM = 1,
  NOTHING = 99,
};

/* OccupancyOcTree Log odds values for various costmap states. */
typedef float Cost;
// Sentinel. Don't choose NAN as it can't be pruned in octomap
const Cost UNKNOWN = -200.0;
const Cost FREE = -10.0;
const Cost LETHAL = 10.0;
/* A log-odds between FREE and LETHAL represents a non-lethal,
 * but greater than zero cost for that space */

using Costmap3DIndex = octomap::OcTreeKey;
using Costmap3DIndexEntryType = octomap::key_type;

class Costmap3D : public octomap::OcTree
{
public:
  Costmap3D(double resolution);
  Costmap3D(const Costmap3D& rhs);
protected:
  virtual void init();
};

typedef std::shared_ptr<Costmap3D> Costmap3DPtr;
typedef std::shared_ptr<const Costmap3D> Costmap3DConstPtr;

octomap::point3d toOctomapPoint(const geometry_msgs::Point& pt);
geometry_msgs::Point fromOctomapPoint(const octomap::point3d& point);

}  // namespace costmap_3d

#endif  // COSTMAP_3D_COSTMAP_3D_H_
