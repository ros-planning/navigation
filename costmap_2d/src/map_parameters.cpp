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

#include <costmap_2d/map_parameters.h>

namespace costmap_2d {

MapParameters::MapParameters() :
        size_x(0), size_y(0), resolution(0), origin_x(0), origin_y(0) {}

MapParameters::MapParameters(const Costmap2D &_map) :
        size_x(_map.getSizeInCellsX()),
        size_y(_map.getSizeInCellsY()),
        resolution(_map.getResolution()),
        origin_x(_map.getOriginX()),
        origin_y(_map.getOriginY()) {}

MapParameters::MapParameters(const nav_msgs::OccupancyGrid& _msg) :
        size_x(_msg.info.width),
        size_y(_msg.info.height),
        resolution(_msg.info.resolution),
        origin_x(_msg.info.origin.position.x),
        origin_y(_msg.info.origin.position.y){}

bool MapParameters::operator==(const MapParameters& _rhs) const {
    return (size_x == _rhs.size_x &&
            size_y == _rhs.size_y &&
            resolution == _rhs.resolution &&
            origin_x == _rhs.origin_x &&
            origin_y == _rhs.origin_y);
}

bool MapParameters::operator!=(const MapParameters& _rhs) const {
    return !(*this == _rhs);
}

std::ostream& operator<<(std::ostream& _os, const MapParameters& _param){
    _os << "<< size_x " << _param.size_x
    << " size_y " << _param.size_y
    << " resolution " << _param.resolution
    << " origin_x " << _param.origin_x
    << " origin_y " << _param.origin_y << ">>";
    return _os;
}

} // namespace costmap_2d