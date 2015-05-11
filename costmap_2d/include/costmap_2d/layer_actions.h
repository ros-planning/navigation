#ifndef COSTMAP_2D_COSTMAP_2D_LAYER_ACTIONS_H_
#define COSTMAP_2D_COSTMAP_2D_LAYER_ACTIONS_H_

#include <costmap_2d/axis_aligned_bounding_box.h>

#include <vector>
#include <string>

namespace costmap_2d
{
class Costmap2D;

/**
 * Auxiliary storage object to track actions taken by the layer plugins to help with optimization. 
 * Each layer plugin will add action data when they modify a Costmap2D. This data will later be 
 * used to make decisions mainly about what needs to be inflated versus reusing old inflation data.
 */
class LayerActions
{
public:
  LayerActions() {}

  /**
   * @brief List of actions that can be taken by a layer plugin on a Costmap2D
   */
  enum Action{
    NONE,            ///< No action taken
    OVERWRITE,       ///< CostmapLayer::updateWithOverwrite action, overwite with exerything except NO_INFORMATION
    TRUEOVERWRITE,   ///< CostmapLayer::updateWithTureOverwrite action, overwite with exerything
    MODIFY,          ///< Make a modification to the Costmap2D
    MAX              ///< CostmapLayer::updateWithMax action, use the maximum value between the source and dest ignoring NO_INFORMATION
  };

  /**
   * @brief Add an action defined as a modification to a region of a map
   * @param r   Region of the impacted map that is being modified
   * @param map The destination map that will be modified
   * @param a Action type of the added action
   */
  void addAction(const AxisAlignedBoundingBox& r, Costmap2D *map, LayerActions::Action a);

  /**
   * @brief Add an action defined as a modification to a region of a map based on data from a source map
   * @param src_rect Region of the soruce map containing the data
   * @param src_map  The source map with the new data
   * @param dst_rect Region of the impacted map that is being modified
   * @param dst_map  The destination map that will be modified
   * @param a Action type of the added action
   */
  void addAction(const AxisAlignedBoundingBox& src_rect, Costmap2D *src_map,
                 const AxisAlignedBoundingBox& dst_rect, Costmap2D *dst_map, LayerActions::Action a);

  /// Clear all actions
  void clear();

  /**
   * @brief Copy all actions to another LayerActions map
   * @param la_dest LayerAction object that will receive the copy
   */
  void copyTo(LayerActions& la_dest);

  /**
   * @brief Append an action to another Layer Action object
   * @param la_dest LayerAction object that will receive the action
   * @param action_index Index of the internal action that will be copied to the destination object
   */
  void appendActionTo(LayerActions& la_dest, int action_index);

  /**
   * @brief Copy all source object that impact a given Costmap2D to a destination LayerActions object
   * @param la_dest LayerAction object that will receive the action
   * @param action_index Index of the internal action that will be copied to the destination object
   */
  void copyToWithMatchingDest(LayerActions& la_dest, Costmap2D* match_pattern);

  /// Number of actions stored in this object
  int size() {return (int)v_actions_.size();}

  /// Get the action type at the given index
  LayerActions::Action actionAt(int idx);
  
  /// Test if the action at the given index is of the given type
  bool actionIs(int idx, Action a);

  /// Test if the given index is positive and less than the size()
  bool validIndex(int idx);

  /// Get the impacted Costmap2D at the given index
  Costmap2D* destinationCostmapAt(int idx);

  /// Get the source Costmap2D at the given index
  Costmap2D* sourceCostmapAt(int idx);

  /// Get the source region at the given index
  const AxisAlignedBoundingBox& sourceAxisAlignedBoundingBoxAt(int idx);
  /// Get the destination region at the given index
  const AxisAlignedBoundingBox& destinationAxisAlignedBoundingBoxAt(int idx);

  // These define the actions that we have collected
  // we have the source and destination rectangle with the corresponding maps
  // and the action type
  std::vector<AxisAlignedBoundingBox>    v_source_rect_;
  std::vector<AxisAlignedBoundingBox>    v_destination_rect_;
  std::vector<Costmap2D*>                v_source_maps_;
  std::vector<Costmap2D*>                v_destination_maps_;
  std::vector<LayerActions::Action>      v_actions_;

private:
  AxisAlignedBoundingBox nullAxisAlignedBoundingBox;
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_COSTMAP_2D_LAYER_ACTIONS_H_
