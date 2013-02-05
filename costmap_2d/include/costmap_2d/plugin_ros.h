#ifndef COSTMAP_PLUGIN_ROS_H_
#define COSTMAP_PLUGIN_ROS_H_
#include <costmap_2d/plugin_base.h>
#include <tf/tf.h>

namespace costmap_2d {
  class CostmapPluginROS : public CostmapPlugin {
    public:
      void initialize(LayeredCostmap* costmap, std::string name, tf::TransformListener &tf){
        tf_ = &tf;
        initialize(costmap, name);
      }
      virtual void initialize(LayeredCostmap* costmap, std::string name)= 0;
    protected:
      CostmapPluginROS() {}
      tf::TransformListener* tf_;
  };
};  // namespace layered_costmap
#endif
