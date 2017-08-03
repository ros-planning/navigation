#ifndef COSTMAP_2D_TESTING_HELPER_H
#define COSTMAP_2D_TESTING_HELPER_H

#include<costmap_2d/cost_values.h>
#include<costmap_2d/costmap_2d.h>
#include <costmap_2d/static_layer.h>
#include <costmap_2d/obstacle_layer.h>
#include <costmap_2d/inflation_layer.h>

const double MAX_Z(1.0);

void setValues(costmap_2d::Costmap2D& costmap, const unsigned char* map)
{
  int index = 0;
  for (int i = 0; i < costmap.getSizeInCellsY(); i++){
    for (int j = 0; j < costmap.getSizeInCellsX(); j++){
      costmap.setCost(j, i, map[index]);
    }
  }
}

char printableCost(unsigned char cost)
{
  switch (cost)
  {
  case costmap_2d::NO_INFORMATION: return '?';
  case costmap_2d::LETHAL_OBSTACLE: return 'L';
  case costmap_2d::INSCRIBED_INFLATED_OBSTACLE: return 'I';
  case costmap_2d::FREE_SPACE: return '.';
  default: return '0' + (unsigned char) (10 * cost / 255);
  }
}

void printMap(costmap_2d::Costmap2D& costmap)
{
  printf("map:\n");
  for (int i = 0; i < costmap.getSizeInCellsY(); i++){
    for (int j = 0; j < costmap.getSizeInCellsX(); j++){
      printf("%4d", int(costmap.getCost(j, i)));
    }
    printf("\n\n");
  }
}

unsigned int countValues(costmap_2d::Costmap2D& costmap, unsigned char value, bool equal = true)
{
  unsigned int count = 0;
  for (int i = 0; i < costmap.getSizeInCellsY(); i++){
    for (int j = 0; j < costmap.getSizeInCellsX(); j++){
      unsigned char c = costmap.getCost(j, i);
      if ((equal && c == value) || (!equal && c != value))
      {
        count+=1;
      }
    }
  }
  return count;
}

void addStaticLayer(costmap_2d::LayeredCostmap& layers, tf::TransformListener& tf)
{
  costmap_2d::StaticLayer* slayer = new costmap_2d::StaticLayer();
  layers.addPlugin(boost::shared_ptr<costmap_2d::Layer>(slayer));
  slayer->initialize(&layers, "static", &tf);
}

costmap_2d::ObstacleLayer* addObstacleLayer(costmap_2d::LayeredCostmap& layers, tf::TransformListener& tf)
{
  costmap_2d::ObstacleLayer* olayer = new costmap_2d::ObstacleLayer();
  olayer->initialize(&layers, "obstacles", &tf);
  layers.addPlugin(boost::shared_ptr<costmap_2d::Layer>(olayer));
  return olayer;
}

void addObservation(costmap_2d::ObstacleLayer* olayer, double x, double y, double z = 0.0,
                    double ox = 0.0, double oy = 0.0, double oz = MAX_Z, double min_raytrace_range = 0.0,
                    double max_raytrace_range = 100., double min_obstacle_range=0.0){
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.points.resize(1);
  cloud.points[0].x = x;
  cloud.points[0].y = y;
  cloud.points[0].z = z;

  geometry_msgs::Point p;
  p.x = ox;
  p.y = oy;
  p.z = oz;

  // max obstacle range = 100.0
  costmap_2d::Observation obs(p, cloud, min_obstacle_range, 100.0, min_raytrace_range, max_raytrace_range);
  olayer->addStaticObservation(obs, true, true);
}

costmap_2d::InflationLayer* addInflationLayer(costmap_2d::LayeredCostmap& layers, tf::TransformListener& tf)
{
  costmap_2d::InflationLayer* ilayer = new costmap_2d::InflationLayer();
  ilayer->initialize(&layers, "inflation", &tf);
  boost::shared_ptr<costmap_2d::Layer> ipointer(ilayer);
  layers.addPlugin(ipointer);
  return ilayer;
}


#endif  // COSTMAP_2D_TESTING_HELPER_H
