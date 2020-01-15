 /*
  * Software License Agreement (BSD License)
  *
  *  Point Cloud Library (PCL) - www.pointclouds.org
  *  Copyright (c) 2011, Willow Garage, Inc.
  *
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
  *   * Neither the name of the copyright holder(s) nor the names of its
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
  */

#ifndef COSTMAP_3D_PCL_CROP_HULL_H_
#define COSTMAP_3D_PCL_CROP_HULL_H_

#include <pcl/point_types.h>
#include <pcl/Vertices.h>

namespace costmap_3d
{

/** \brief A thread-safe version of the PCL CropHull filter.
  * \author James Crosby
  * \author C. Andy Martin
  */
template<typename PointT>
class CropHull
{
  using PointCloud = typename pcl::PointCloud<PointT>;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = typename PointCloud::ConstPtr;

public:
  using Ptr = boost::shared_ptr<CropHull<PointT> >;
  using ConstPtr = boost::shared_ptr<const CropHull<PointT> >;

  /** \brief Empty Constructor. */
  CropHull () :
    crop_outside_(true)
  {
  }

  /** \brief Set the vertices of the hull used to filter points.
    * \param[in] polygons Vector of polygons (Vertices structures) forming
    * the hull used for filtering points.
    */
  inline void
  setHullIndices (const std::vector<pcl::Vertices>& polygons)
  {
    hull_polygons_ = polygons;
  }

  /** \brief Get the vertices of the hull used to filter points.
    */
  const std::vector<pcl::Vertices>&
  getHullIndices () const
  {
    return (hull_polygons_);
  }

  /** \brief Set the point cloud that the hull indices refer to
    * \param[in] points the point cloud that the hull indices refer to
    */
  inline void
  setHullCloud (PointCloudPtr points)
  {
    hull_cloud_ = points;
  }

  /** \brief Get the point cloud that the hull indices refer to. */
  PointCloudPtr
  getHullCloud () const
  {
    return (hull_cloud_);
  }

  /** \brief Remove points outside the hull (default), or those inside the hull.
    * \param[in] crop_outside If true, the filter will remove points
    * outside the hull. If false, those inside will be removed.
    */
  inline void
  setCropOutside(bool crop_outside)
  {
    crop_outside_ = crop_outside;
  }

  /** \brief Filter the input points using the 2D or 3D polygon hull.
    * \param[in] input the input cloud.
    * \param[out] indices the indices of the set of points that passed the filter.
    */
  void
  applyFilter (const PointCloud& input, std::vector<int> &indices)
  {
    for (std::size_t index = 0; index < input.size(); index++)
    {
      std::size_t crossings[3] = {0,0,0};
      Eigen::Vector3f rays[3] =
      {
        Eigen::Vector3f(0.264882f,  0.688399f, 0.675237f),
        Eigen::Vector3f(0.0145419f, 0.732901f, 0.68018f),
        Eigen::Vector3f(0.856514f,  0.508771f, 0.0868081f)
      };

      for (std::size_t poly = 0; poly < hull_polygons_.size (); poly++)
        for (std::size_t ray = 0; ray < 3; ray++)
          crossings[ray] += rayTriangleIntersect
            (input.points[index], rays[ray], hull_polygons_[poly], *hull_cloud_);

      if (crop_outside_ && (crossings[0]&1) + (crossings[1]&1) + (crossings[2]&1) > 1)
        indices.push_back (index);
      else if (!crop_outside_)
        indices.push_back (index);
    }
  }

  /** \brief Does a ray cast from a point intersect with an arbitrary
    * triangle in 3D?
    * See: http://softsurfer.com/Archive/algorithm_0105/algorithm_0105.htm#intersect_RayTriangle()
    * \param[in] point Point from which the ray is cast.
    * \param[in] ray   Vector in direction of ray.
    * \param[in] verts Indices of vertices making the polygon.
    * \param[in] cloud Cloud from which the vertex indices are drawn.
    */
  inline static bool
  rayTriangleIntersect (const PointT& point,
                        const Eigen::Vector3f& ray,
                        const pcl::Vertices& verts,
                        const PointCloud& cloud)
  {
    // Algorithm here is adapted from:
    // http://softsurfer.com/Archive/algorithm_0105/algorithm_0105.htm#intersect_RayTriangle()
    //
    // Original copyright notice:
    // Copyright 2001, softSurfer (www.softsurfer.com)
    // This code may be freely used and modified for any purpose
    // providing that this copyright notice is included with it.
    //
    assert (verts.vertices.size () == 3);

    const Eigen::Vector3f p = point.getVector3fMap ();
    const Eigen::Vector3f a = cloud[verts.vertices[0]].getVector3fMap ();
    const Eigen::Vector3f b = cloud[verts.vertices[1]].getVector3fMap ();
    const Eigen::Vector3f c = cloud[verts.vertices[2]].getVector3fMap ();
    const Eigen::Vector3f u = b - a;
    const Eigen::Vector3f v = c - a;
    const Eigen::Vector3f n = u.cross (v);
    const float n_dot_ray = n.dot (ray);

    if (std::fabs (n_dot_ray) < 1e-9)
      return (false);

    const float r = n.dot (a - p) / n_dot_ray;

    if (r < 0)
      return (false);

    const Eigen::Vector3f w = p + r * ray - a;
    const float denominator = u.dot (v) * u.dot (v) - u.dot (u) * v.dot (v);
    const float s_numerator = u.dot (v) * w.dot (v) - v.dot (v) * w.dot (u);
    const float s = s_numerator / denominator;
    if (s < 0 || s > 1)
      return (false);

    const float t_numerator = u.dot (v) * w.dot (u) - u.dot (u) * w.dot (v);
    const float t = t_numerator / denominator;
    if (t < 0 || s+t > 1)
      return (false);

    return (true);
  }

protected:
  /** \brief The vertices of the hull used to filter points. */
  std::vector<pcl::Vertices> hull_polygons_;

  /** \brief The point cloud that the hull indices refer to. */
  PointCloudPtr hull_cloud_;

  /** \brief If true, the filter will remove points outside the hull. If
   * false, those inside will be removed.
   */
  bool crop_outside_;
};

} // namespace costmap_3d

#endif  // COSTMAP_3D_PCL_CROP_HULL_H_
