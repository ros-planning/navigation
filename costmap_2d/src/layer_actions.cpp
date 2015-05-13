#include "costmap_2d/layer_actions.h"

#include <algorithm>  // for min
#include <vector>     // for vector<>

namespace costmap_2d
{
void LayerActions::addAction(const AxisAlignedBoundingBox& r, Costmap2D* map, LayerActions::Action a)
{
  v_actions_.push_back(a);

  // we either don't know the source map or there is no source map
  // for instance, the footprint source is a polygon that gets written
  v_source_maps_.push_back(0);
  v_source_rect_.push_back(AxisAlignedBoundingBox());

  v_destination_maps_.push_back(map);
  v_destination_rect_.push_back(AxisAlignedBoundingBox(r));
}

void LayerActions::addAction(const AxisAlignedBoundingBox& src_rect, Costmap2D* src_map,
                             const AxisAlignedBoundingBox& dst_rect, Costmap2D* dst_map,
                             LayerActions::Action a)
{
  v_actions_.push_back(a);

  v_source_maps_.push_back(src_map);
  v_source_rect_.push_back(src_rect);

  v_destination_maps_.push_back(dst_map);
  v_destination_rect_.push_back(dst_rect);
}

void LayerActions::clear()
{
  v_actions_.clear();
  v_source_maps_.clear();
  v_source_rect_.clear();
  v_destination_maps_.clear();
  v_destination_rect_.clear();
}

void LayerActions::copyTo(LayerActions& la_dest)
{
  la_dest.clear();

  const int n = v_actions_.size();
  for (int i = 0; i < n; i++)
  {
    appendActionTo(la_dest, i);
  }
}

void LayerActions::appendActionTo(LayerActions &la_dest, int action_index)
{
  if(!validIndex(action_index))
    return;
  la_dest.v_actions_.push_back(v_actions_[action_index]);
  la_dest.v_source_maps_.push_back(v_source_maps_[action_index]);
  la_dest.v_source_rect_.push_back(v_source_rect_[action_index]);
  la_dest.v_destination_maps_.push_back(v_destination_maps_[action_index]);
  la_dest.v_destination_rect_.push_back(v_destination_rect_[action_index]);
}

void LayerActions::copyToWithMatchingDest(LayerActions &la_dest, Costmap2D *match_pattern)
{
  la_dest.clear();

  const int n = size();
  for (int i = 0; i < n; i++)
  {
    if (v_destination_maps_[i] == match_pattern)
      appendActionTo(la_dest, i);
  }
}

LayerActions::Action LayerActions::actionAt(int idx)
{
  if (validIndex(idx))
    return v_actions_[idx];
  return NONE;
}

bool LayerActions::actionIs(int idx, LayerActions::Action a)
{
  return actionAt(idx) == a;
}

bool LayerActions::validIndex(int idx)
{
  return idx >= 0 && idx < size();
}

Costmap2D* LayerActions::destinationCostmapAt(int idx)
{
  if (validIndex(idx))
    return v_destination_maps_[idx];
  return 0;
}

Costmap2D* LayerActions::sourceCostmapAt(int idx)
{
  if (validIndex(idx))
    return v_source_maps_[idx];
  return 0;
}

const AxisAlignedBoundingBox& LayerActions::sourceAxisAlignedBoundingBoxAt(int idx)
{
  if (validIndex(idx))
    return v_source_rect_[idx];
  return nullAxisAlignedBoundingBox;
}

const AxisAlignedBoundingBox& LayerActions::destinationAxisAlignedBoundingBoxAt(int idx)
{
  if (validIndex(idx))
    return v_destination_rect_[idx];
  return nullAxisAlignedBoundingBox;
}

}  // end namespace costmap_2d
