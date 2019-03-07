/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * author: Dmitrij Dorezyuk
 */

#ifndef COSTMAP_2D_MAP_PARAMETERS_HPP
#define COSTMAP_2D_MAP_PARAMETERS_HPP

#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/OccupancyGrid.h>

#include <ostream>

namespace costmap_2d{

/// @brief Struct contains the parameters of a costmap
struct MapParameters {
    /// @brief default c'tor
    MapParameters();

    /**
     * @brief c'tor from costmap
     * @param _map contains the parameters to be replicated
     */
    explicit MapParameters(const Costmap2D& _map);

    /**
     * @brief c'tor from nav_msgs
     * @param _msg contains the parameters to be replicated
     */
    explicit MapParameters(const nav_msgs::OccupancyGrid& _msg);

    bool operator==(const MapParameters& _rhs) const;
    bool operator!=(const MapParameters& _rhs) const;
    friend std::ostream& operator<<(std::ostream& _os, const MapParameters& _param);
    unsigned int size_x; //< size of the map in cells in x direction
    unsigned int size_y; //< size of the map in cells in y direction
    double resolution;  //< resolution of the map (size of one cell)
    double origin_x;    //< origin of the map, x coordinate
    double origin_y;    //< origin of the map, y coordinate
};
} // namespace costmap_2d

#endif //COSTMAP_2D_MAP_PARAMETERS_HPP
