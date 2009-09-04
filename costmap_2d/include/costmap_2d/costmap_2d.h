/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/
#ifndef COSTMAP_COSTMAP_2D_H_
#define COSTMAP_COSTMAP_2D_H_

#include <vector>
#include <queue>
#include <costmap_2d/observation.h>
#include <costmap_2d/cell_data.h>
#include <costmap_2d/cost_values.h>
#include <sensor_msgs/PointCloud.h>
#include <boost/thread.hpp>

namespace costmap_2d {
  //convenient for storing x/y point pairs
  struct MapLocation {
    unsigned int x;
    unsigned int y;
  };

  /**
   * @class Costmap2D
   * @brief A 2D costmap provides a mapping between points in the world and their associated "costs".
   */
  class Costmap2D {
    public:
      /**
       * @brief  Constructor for a costmap
       * @param  cells_size_x The x size of the map in cells
       * @param  cells_size_y The y size of the map in cells
       * @param  resolution The resolution of the map in meters/cell
       * @param  origin_x The x origin of the map
       * @param  origin_y The y origin of the map
       * @param  inscribed_radius The inscribed radius of the robot
       * @param  circumscribed_radius The circumscribed radius of the robot
       * @param  inflation_radius How far out to inflate obstacles
       * @param  max_obstacle_range The maximum range at which obstacles will be put into the costmap from any sensor
       * @param  max_obstacle_height The maximum height of obstacles that will be considered
       * @param  max_raytrace_range The maximum distance we'll raytrace out to with any sensor
       * @param  weight The scaling factor for the cost function. 
       * @param  static_data Data used to initialize the costmap
       * @param  lethal_threshold The cost threshold at which a point in the static data is considered a lethal obstacle
       */
      Costmap2D(unsigned int cells_size_x, unsigned int cells_size_y, 
          double resolution, double origin_x, double origin_y, double inscribed_radius = 0.0,
          double circumscribed_radius = 0.0, double inflation_radius = 0.0, double max_obstacle_range = 0.0,
          double max_obstacle_height = 0.0, double max_raytrace_range = 0.0, double weight = 25.0,
          const std::vector<unsigned char>& static_data = std::vector<unsigned char>(0), unsigned char lethal_threshold = 0);

      /**
       * @brief  Copy constructor for a costmap, creates a copy efficiently
       * @param map The costmap to copy 
       */
      Costmap2D(const Costmap2D& map);

      /**
       * @brief  Overloaded assignment operator
       * @param  map The costmap to copy
       * @return A reference to the map after the copy has finished
       */
      Costmap2D& operator=(const Costmap2D& map);

      /**
       * @brief  Default constructor
       */
      Costmap2D();

      /**
       * @brief  Destructor
       */
      virtual ~Costmap2D();

      /**
       * @brief  Revert to the static map outside of a specified window centered at a world coordinate
       * @param wx The x coordinate of the center point of the window in world space (meters)
       * @param wy The y coordinate of the center point of the window in world space (meters)
       * @param w_size_x The x size of the window in meters
       * @param w_size_y The y size of the window in meters
       */
      virtual void resetMapOutsideWindow(double wx, double wy, double w_size_x, double w_size_y);

      /**
       * @brief Re-inflate obstacles within a given window
       * @param wx The x coordinate of the center point of the window in world space (meters)
       * @param wy The y coordinate of the center point of the window in world space (meters)
       * @param w_size_x The x size of the window in meters
       * @param w_size_y The y size of the window in meters
       * @param clear When set to true, will clear all non-lethal obstacles before inflation
       */
      void reinflateWindow(double wx, double wy, double w_size_x, double w_size_y, bool clear = true);

      /**
       * @brief  Clears non lethal obstacles in a specified window
       * @param wx The x coordinate of the center point of the window in world space (meters)
       * @param wy The y coordinate of the center point of the window in world space (meters)
       * @param w_size_x The x size of the window in meters
       * @param w_size_y The y size of the window in meters
       * @param clear_no_info If set to true, NO_INFORMATION will be cleared, if set to false NO_INFORMATION will be treated as a lethal obstacle
       */
      void clearNonLethal(double wx, double wy, double w_size_x, double w_size_y, bool clear_no_info = false);

      /**
       * @brief  Update the costmap with new observations
       * @param obstacles The point clouds of obstacles to insert into the map 
       * @param clearing_observations The set of observations to use for raytracing 
       */
      void updateWorld(double robot_x, double robot_y, 
          const std::vector<Observation>& observations, const std::vector<Observation>& clearing_observations);

      /**
       * @brief  Get the cost of a cell in the costmap
       * @param mx The x coordinate of the cell 
       * @param my The y coordinate of the cell 
       * @return The cost of the cell
       */
      unsigned char getCost(unsigned int mx, unsigned int my) const;

      /**
       * @brief  Set the cost of a cell in the costmap
       * @param mx The x coordinate of the cell 
       * @param my The y coordinate of the cell 
       * @param cost The cost to set the cell to
       */
      void setCost(unsigned int mx, unsigned int my, unsigned char cost);

      /**
       * @brief  Convert from map coordinates to world coordinates
       * @param  mx The x map coordinate
       * @param  my The y map coordinate
       * @param  wx Will be set to the associated world x coordinate
       * @param  wy Will be set to the associated world y coordinate
       */
      void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const;

      /**
       * @brief  Convert from world coordinates to map coordinates
       * @param  wx The x world coordinate
       * @param  wy The y world coordinate
       * @param  mx Will be set to the associated map x coordinate
       * @param  my Will be set to the associated map y coordinate
       * @return True if the conversion was successful (legal bounds) false otherwise
       */
      bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const;

      /**
       * @brief  Convert from world coordinates to map coordinates without checking for legal bounds
       * @param  wx The x world coordinate
       * @param  wy The y world coordinate
       * @param  mx Will be set to the associated map x coordinate
       * @param  my Will be set to the associated map y coordinate
       * @note   The returned map coordinates <b>are not guaranteed to lie within the map.</b>
       */
      void worldToMapNoBounds(double wx, double wy, int& mx, int& my) const;

      /**
       * @brief  Given two map coordinates... compute the associated index
       * @param mx The x coordinate 
       * @param my The y coordinate 
       * @return The associated index
       */
      inline unsigned int getIndex(unsigned int mx, unsigned int my) const{
        return my * size_x_ + mx;
      }

      /**
       * @brief  Given an index... compute the associated map coordinates
       * @param  index The index
       * @param  mx Will be set to the x coordinate
       * @param  my Will be set to the y coordinate
       */
      inline void indexToCells(unsigned int index, unsigned int& mx, unsigned int& my) const{
        my = index / size_x_;
        mx = index - (my * size_x_);
      }

      /**
       * @brief  Will return a immutable pointer to the underlying unsigned char array used as the costmap
       * @return A pointer to the underlying unsigned char array storing cost values
       */
      const unsigned char* getCharMap() const;

      /**
       * @brief  Accessor for the x size of the costmap in cells
       * @return The x size of the costmap
       */
      unsigned int getSizeInCellsX() const;

      /**
       * @brief  Accessor for the y size of the costmap in cells
       * @return The y size of the costmap
       */
      unsigned int getSizeInCellsY() const;

      /**
       * @brief  Accessor for the x size of the costmap in meters
       * @return The x size of the costmap (returns the centerpoint of the last legal cell in the map)
       */
      double getSizeInMetersX() const;

      /**
       * @brief  Accessor for the y size of the costmap in meters
       * @return The y size of the costmap (returns the centerpoint of the last legal cell in the map)
       */
      double getSizeInMetersY() const;

      /**
       * @brief  Accessor for the x origin of the costmap
       * @return The x origin of the costmap
       */
      double getOriginX() const;

      /**
       * @brief  Accessor for the y origin of the costmap
       * @return The y origin of the costmap
       */
      double getOriginY() const;

      /**
       * @brief  Accessor for the resolution of the costmap
       * @return The resolution of the costmap
       */
      double getResolution() const;
    
      /**
       * @brief  Accessor for the inscribed radius of the robot
       * @return The inscribed radius
       */
      double getInscribedRadius() const { return inscribed_radius_; }
    
      /**
       * @brief  Accessor for the circumscribed radius of the robot
       * @return The circumscribed radius
       */
      double getCircumscribedRadius() const { return circumscribed_radius_; }
    
      /**
       * @brief  Accessor for the inflation radius of the robot
       * @return The inflation radius
       */
      double getInflationRadius() const { return inflation_radius_; }

      /**
       * @brief  Sets the cost of a convex polygon to a desired value
       * @param polygon The polygon to perform the operation on 
       * @param cost_value The value to set costs to
       * @return True if the polygon was filled... false if it could not be filled
       */
      bool setConvexPolygonCost(const std::vector<geometry_msgs::Point>& polygon, unsigned char cost_value);

      /**
       * @brief  Get the map cells that make up the outline of a polygon
       * @param polygon The polygon in map coordinates to rasterize 
       * @param polygon_cells Will be set to the cells contained in the outline of the polygon
       */
      void polygonOutlineCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells);

      /**
       * @brief  Get the map cells that fill a convex polygon
       * @param polygon The polygon in map coordinates to rasterize 
       * @param polygon_cells Will be set to the cells that fill the polygon
       */
      void convexFillCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells);


      /**
       * @brief  Move the origin of the costmap to a new location.... keeping data when it can
       * @param  new_origin_x The x coordinate of the new origin
       * @param  new_origin_y The y coordinate of the new origin
       */
      virtual void updateOrigin(double new_origin_x, double new_origin_y);

      /**
       * @brief  Check if a cell falls within the circumscribed radius of the
       * robot but outside the inscribed radius of the robot
       * @param The x coordinate of the cell
       * @param The y coordinate of the cell
       * @return True if the cell is inside the circumscribed radius but outside the inscribed radius, false otherwise
       */
      bool isCircumscribedCell(unsigned int x, unsigned int y) const;

      /**
       * @brief  Given a distance... compute a cost
       * @param  distance The distance from an obstacle in cells
       * @return A cost value for the distance
       */
      inline unsigned char computeCost(double distance) const {
        unsigned char cost = 0;
        if(distance == 0)
          cost = LETHAL_OBSTACLE;
        else if(distance <= cell_inscribed_radius_)
          cost = INSCRIBED_INFLATED_OBSTACLE;
        else {
          //make sure cost falls off by Euclidean distance
          double euclidean_distance = distance * resolution_;
          double factor = exp(-1.0 * weight_ * (euclidean_distance - inscribed_radius_));
          cost = (unsigned char) ((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
        }
        return cost;
      }
    
      /***
       * @brief Get the lower bound of cost for cells inside the circumscribed radius
       * @return the circumscribed_cost_lb_ attribute, which gets initialized to computeCost(cell_circumscribed_radius_)
       */
      inline unsigned char getCircumscribedCost() const {
        return circumscribed_cost_lb_;
      }

    protected:
      /**
       * @brief  Given an index of a cell in the costmap, place it into a priority queue for obstacle inflation
       * @param  index The index of the cell
       * @param  mx The x coordinate of the cell (can be computed from the index, but saves time to store it)
       * @param  my The y coordinate of the cell (can be computed from the index, but saves time to store it)
       * @param  src_x The x index of the obstacle point inflation started at
       * @param  src_y The y index of the obstacle point inflation started at
       * @param  inflation_queue The priority queue to insert into
       */
      inline void enqueue(unsigned int index, unsigned int mx, unsigned int my, 
          unsigned int src_x, unsigned int src_y, std::priority_queue<CellData>& inflation_queue){
        unsigned char* marked = &markers_[index];
        //set the cost of the cell being inserted
        if(*marked == 0){
          //we compute our distance table one cell further than the inflation radius dictates so we can make the check below
          double distance = distanceLookup(mx, my, src_x, src_y);

          //we only want to put the cell in the queue if it is within the inflation radius of the obstacle point
          if(distance >= cell_inflation_radius_)
            return;

          //assign the cost associated with the distance from an obstacle to the cell
          updateCellCost(index, costLookup(mx, my, src_x, src_y));

          //push the cell data onto the queue and mark
          CellData data(distance, index, mx, my, src_x, src_y);
          inflation_queue.push(data);
          *marked = 1;
        }
      }

    private:
      /**
       * @brief  Insert new obstacles into the cost map
       * @param obstacles The point clouds of obstacles to insert into the map 
       * @param inflation_queue The queue to place the obstacles into for inflation
       */
      virtual void updateObstacles(const std::vector<Observation>& observations, std::priority_queue<CellData>& inflation_queue);

      /**
       * @brief  Clear freespace based on any number of observations
       * @param clearing_observations The observations used to raytrace 
       */
      void raytraceFreespace(const std::vector<Observation>& clearing_observations);

      /**
       * @brief  Clear freespace from an observation
       * @param clearing_observation The observation used to raytrace 
       */
      virtual void raytraceFreespace(const Observation& clearing_observation);

      /**
       * @brief  Provides support for re-inflating obstacles within a certain window (used after raytracing)
       * @param wx The x coordinate of the center point of the window in world space (meters)
       * @param wy The y coordinate of the center point of the window in world space (meters)
       * @param w_size_x The x size of the window in meters
       * @param w_size_y The y size of the window in meters
       * @param inflation_queue The priority queue to push items back onto for propogation
       * @param clear When set to true, will clear all non-lethal obstacles before inflation
       */
      void resetInflationWindow(double wx, double wy, double w_size_x, double w_size_y,
          std::priority_queue<CellData>& inflation_queue, bool clear = true );

      /**
       * @brief  Raytrace a line and apply some action at each step
       * @param  at The action to take... a functor
       * @param  x0 The starting x coordinate
       * @param  y0 The starting y coordinate
       * @param  x1 The ending x coordinate
       * @param  y1 The ending y coordinate
       * @param  max_length The maximum desired length of the segment... allows you to not go all the way to the endpoint
       */
      template <class ActionType>
        inline void raytraceLine(ActionType at, unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1, unsigned int max_length = UINT_MAX){
          int dx = x1 - x0;
          int dy = y1 - y0;

          unsigned int abs_dx = abs(dx);
          unsigned int abs_dy = abs(dy);

          int offset_dx = sign(dx);
          int offset_dy = sign(dy) * size_x_;

          unsigned int offset = y0 * size_x_ + x0;

          //we need to chose how much to scale our dominant dimension, based on the maximum length of the line
          double dist = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
          double scale = std::min(1.0,  max_length / dist);

          //if x is dominant
          if(abs_dx >= abs_dy){
            int error_y = abs_dx / 2;
            bresenham2D(at, abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx));
            return;
          }

          //otherwise y is dominant
          int error_x = abs_dy / 2;
          bresenham2D(at, abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy));

        }

      /**
       * @brief  A 2D implementation of Bresenham's raytracing algorithm... applies an action at each step
       */
      template <class ActionType>
        inline void bresenham2D(ActionType at, unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a, int offset_b, 
            unsigned int offset, unsigned int max_length){
          unsigned int end = std::min(max_length, abs_da);
          for(unsigned int i = 0; i < end; ++i){
            at(offset);
            offset += offset_a;
            error_b += abs_db;
            if((unsigned int)error_b >= abs_da){
              offset += offset_b;
              error_b -= abs_da;
            }
          }
          at(offset);
        }

      /**
       * @brief  Given a priority queue with the actual obstacles, compute the inflated costs for the costmap
       * @param  inflation_queue A priority queue contatining the cell data for the actual obstacles
       */
      void inflateObstacles(std::priority_queue<CellData>& inflation_queue);

      /**
       * @brief  Takes the max of existing cost and the new cost... keeps static map obstacles from being overridden prematurely
       * @param index The index od the cell to assign a cost to 
       * @param cost The cost
       */
      inline void updateCellCost(unsigned int index, unsigned char cost){
        unsigned char* cell_cost = &costmap_[index];
        if(*cell_cost != NO_INFORMATION)
          *cell_cost = std::max(cost, *cell_cost);
      }

      /**
       * @brief  Lookup pre-computed costs
       * @param mx The x coordinate of the current cell 
       * @param my The y coordinate of the current cell 
       * @param src_x The x coordinate of the source cell 
       * @param src_y The y coordinate of the source cell 
       * @return 
       */
      inline char costLookup(int mx, int my, int src_x, int src_y){
        unsigned int dx = abs(mx - src_x);
        unsigned int dy = abs(my - src_y);
        return cached_costs_[dx][dy];
      }

      /**
       * @brief  Lookup pre-computed distances
       * @param mx The x coordinate of the current cell 
       * @param my The y coordinate of the current cell 
       * @param src_x The x coordinate of the source cell 
       * @param src_y The y coordinate of the source cell 
       * @return 
       */
      inline double distanceLookup(int mx, int my, int src_x, int src_y){
        unsigned int dx = abs(mx - src_x);
        unsigned int dy = abs(my - src_y);
        return cached_distances_[dx][dy];
      }

      inline int sign(int x){
        return x > 0 ? 1.0 : -1.0;
      }


    protected:
      /**
       * @brief  Given distance in the world... convert it to cells
       * @param  world_dist The world distance
       * @return The equivalent cell distance
       */
      unsigned int cellDistance(double world_dist);


      unsigned int size_x_;
      unsigned int size_y_;
      double resolution_;
      double origin_x_;
      double origin_y_;
      unsigned char* static_map_;
      unsigned char* costmap_;
      unsigned char* markers_;
      double max_obstacle_range_;
      double max_obstacle_height_;
      double max_raytrace_range_;
      unsigned char** cached_costs_;
      double** cached_distances_;
      double inscribed_radius_, circumscribed_radius_, inflation_radius_;
      unsigned int cell_inscribed_radius_, cell_circumscribed_radius_, cell_inflation_radius_;
      double weight_;
      unsigned char circumscribed_cost_lb_;
      std::priority_queue<CellData> inflation_queue_;

      //functors for raytracing actions
      class ClearCell {
        public:
          ClearCell(unsigned char* costmap) : costmap_(costmap) {}
          inline void operator()(unsigned int offset){
            costmap_[offset] = 0;
          }
        private:
          unsigned char* costmap_;
      };

      class MarkCell {
        public:
          MarkCell(unsigned char* costmap) : costmap_(costmap) {}
          inline void operator()(unsigned int offset){
            costmap_[offset] = LETHAL_OBSTACLE;
          }
        private:
          unsigned char* costmap_;
      };

      class PolygonOutlineCells {
        public:
          PolygonOutlineCells(const Costmap2D& costmap, const unsigned char* char_map, std::vector<MapLocation>& cells) 
            : costmap_(costmap), char_map_(char_map), cells_(cells){}

          //just push the relevant cells back onto the list
          inline void operator()(unsigned int offset){
            MapLocation loc;
            costmap_.indexToCells(offset, loc.x, loc.y);
            cells_.push_back(loc);
          }

        private:
          const Costmap2D& costmap_;
          const unsigned char* char_map_;
          std::vector<MapLocation>& cells_;
      };
  };
};

#endif
