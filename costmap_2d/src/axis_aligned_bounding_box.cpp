#include "costmap_2d/axis_aligned_bounding_box.h"

#include <string>     // for string
#include <algorithm>  // for min
#include <vector>     // for vector<>

namespace costmap_2d
{

AxisAlignedBoundingBox::AxisAlignedBoundingBox()
  : min_x_(0), max_x_(0), min_y_(0), max_y_(0), initialized_(false)
{
}

AxisAlignedBoundingBox::AxisAlignedBoundingBox(const AxisAlignedBoundingBox &r)
{
  r.copyTo(*this);
}

bool AxisAlignedBoundingBox::same(const AxisAlignedBoundingBox &r, int tol)
{
  if (!initialized() || !r.initialized())
    return false;
  if (tol)
  {
    if (abs(r.min_x_ - min_x_) > tol) return false;
    if (abs(r.min_y_ - min_y_) > tol) return false;
    if (abs(r.max_x_ - max_x_) > tol) return false;
    if (abs(r.max_y_ - max_y_) > tol) return false;
    return true;
  }
  if (r.min_x_ != min_x_) return false;
  if (r.min_y_ != min_y_) return false;
  if (r.max_x_ != max_x_) return false;
  if (r.max_y_ != max_y_) return false;
  return true;
}

void AxisAlignedBoundingBox::copyTo(AxisAlignedBoundingBox &dst) const
{
  dst.min_x_ = min_x_;
  dst.min_y_ = min_y_;
  dst.max_x_ = max_x_;
  dst.max_y_ = max_y_;
  dst.setInitialized(initialized());
}

AxisAlignedBoundingBox::AxisAlignedBoundingBox(int px, int py)
{
  initialized_ = true;
  min_x_ = px;
  min_y_ = py;
  max_x_ = px;
  max_y_ = py;
}

AxisAlignedBoundingBox::AxisAlignedBoundingBox(int px1, int py1, int px2, int py2)
{
  initialized_ = true;
  min_x_ = std::min(px1, px2);
  max_x_ = std::max(px1, px2);
  min_y_ = std::min(py1, py2);
  max_y_ = std::max(py1, py2);
}


void AxisAlignedBoundingBox::expandBoundingBox(const AxisAlignedBoundingBox &r)
{
  if (initialized_)
  {
    min_x_ = std::min(min_x_, r.min_x_);
    max_x_ = std::max(max_x_, r.max_x_);
    min_y_ = std::min(min_y_, r.min_y_);
    max_y_ = std::max(max_y_, r.max_y_);
  }
  else
  {
    r.copyTo(*this);
  }
}

void AxisAlignedBoundingBox::expandBoundingBox(int r)
{
  if (initialized_)
  {
    min_x_ -= r;
    min_y_ -= r;
    max_x_ += r;
    max_y_ += r;
  }
}

bool AxisAlignedBoundingBox::inside(int px, int py, int margin) const
{
  if(!initialized_)
    return false;
  if (px < min_x_+margin) return false;
  if (px > max_x_-margin) return false;
  if (py < min_y_+margin) return false;
  if (py > max_y_-margin) return false;
  return true;
}


void AxisAlignedBoundingBox::ensureInside(int px, int py)
{
  if(!initialized_)
  {
    setInitialized(true);
    min_x_ = max_x_ = px;
    min_y_ = max_y_ = py;
  }
  else
  {
    min_x_ = std::min(min_x_, px);
    max_x_ = std::max(max_x_, px);
    min_y_ = std::min(min_y_, py);
    max_y_ = std::max(max_y_, py);
  }
}

bool AxisAlignedBoundingBox::inside(const AxisAlignedBoundingBox& bb, int margin) const
{
  return
      inside(bb.min_x_, bb.min_y_, margin) &&
      inside(bb.max_x_, bb.max_y_, margin);
}

void AxisAlignedBoundingBox::clip(const AxisAlignedBoundingBox &bb, AxisAlignedBoundingBox &clipped) const
{
  bb.copyTo(clipped);
  clipped.min_x_ = std::max(min_x_, clipped.min_x_);
  clipped.min_y_ = std::max(min_y_, clipped.min_y_);
  clipped.max_x_ = std::min(max_x_, clipped.max_x_);
  clipped.max_y_ = std::min(max_y_, clipped.max_y_);

  clipped.clampBounds();
}

void AxisAlignedBoundingBox::clampBounds()
{
  if (max_x_ < min_x_)
    max_x_ = min_x_;
  if (max_y_ < min_y_)
    max_y_ = min_y_;
}

double AxisAlignedBoundingBox::ratioInside(const AxisAlignedBoundingBox &bb) const
{
  AxisAlignedBoundingBox clipped;
  clip(bb, clipped);

  const double denom = clipped.area();
  const double numer = area();
  
  if(denom == 0)
    return 0;
  return numer/ denom;
}

int AxisAlignedBoundingBox::area() const
{
  return xn() * yn();
}

int AxisAlignedBoundingBox::xn() const
{
  if (!initialized_)
    return 0;
  return max_x_ - min_x_;
}

int AxisAlignedBoundingBox::yn() const
{
  if (!initialized_)
    return 0;
  return max_y_ - min_y_;
}

bool AxisAlignedBoundingBox::intersect(const AxisAlignedBoundingBox &bb, int margin) const
{
  margin *= 2;  // since margin applied to both
  if (bb.max_x_ + margin < min_x_) return false;
  if (bb.min_x_ - margin > max_x_) return false;
  if (bb.max_y_ + margin < min_y_) return false;
  if (bb.min_y_ - margin > max_y_) return false;
  return true;
}

void AxisAlignedBoundingBox::mergeIntersecting(std::vector<AxisAlignedBoundingBox> &vec_aabb, int margin)
{
  const int n = vec_aabb.size();
  std::vector<AxisAlignedBoundingBox> cpy;
  for (int i = 0; i < n; i++)
    cpy.push_back(vec_aabb.at(i));
  vec_aabb.clear();

  std::vector<bool> got_it;  // if this data is in our new list
  got_it.resize(n, false);

  for (int i = 0; i < n; i++)
  {
    if (!got_it[i])
    {
      AxisAlignedBoundingBox bb = cpy[i];
      got_it[i] = true;

      for (int j = i+1; j < n; j++)
      {
        if (!got_it[j])
        {
          if (bb.intersect(cpy[j], margin))
          {
            bb.expandBoundingBox(cpy[j]);
            got_it[j] = true;
          }
        }
      }
      vec_aabb.push_back(bb);
    }
  }
}

int AxisAlignedBoundingBox::x0() const
{
  if (!initialized_)
    return 0;
  return min_x_;
}

int AxisAlignedBoundingBox::y0() const
{
  if (!initialized_)
    return 0;
  return min_y_;
}

}  // end namespace costmap_2d
