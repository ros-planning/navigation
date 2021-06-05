/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
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
 */

/*
 * This file contains helper functions for loading images as maps.
 *
 * Author: Brian Gerkey
 */

#include <cstring>
#include <stdexcept>

#include <stdlib.h>
#include <stdio.h>

// We use SDL_image to load the image from disk
#include <SDL/SDL_image.h>

#include "map_server/image_loader.h"

namespace map_server
{

namespace
{

/// Compute a linear index given map coords
size_t toMapIdx(const size_t sx, const size_t i, const size_t j)
{
  return (sx * j) + i;
}

}

void
loadMapFromFile(const std::string& fname,
                const double res, 
                const bool negate,
                const double occ_th,
                const double free_th,
                const std::array<double, 3>& origin,
                const MapMode mode,
                nav_msgs::GetMap::Response& resp)
{
  SDL_Surface* img;

  // Load the image using SDL.  If we get NULL back, the image load failed.
  if(!(img = IMG_Load(fname.c_str())))
  {
    const std::string errmsg = std::string("failed to open image file \"") +
                               fname + std::string("\": ") + IMG_GetError();
    throw std::runtime_error(errmsg);
  }

  // Copy the image data into the map structure
  auto& info = resp.map.info;
  info.width = img->w;
  info.height = img->h;
  info.resolution = res;
  info.origin.position.x = origin[0];
  info.origin.position.y = origin[1];
  info.origin.position.z = 0.0;

  const double yaw = origin[2];
  info.origin.orientation.x = 0.0;
  info.origin.orientation.y = 0.0;
  info.origin.orientation.z = sin(yaw / 2.0);
  info.origin.orientation.w = cos(yaw / 2.0);

  // Allocate space to hold the data
  resp.map.data.resize(info.width * info.height);

  // Get values that we'll need to iterate through the pixels
  const auto rowstride = img->pitch;  // the length of a row of pixels in bytes
  const auto n_channels = img->format->BytesPerPixel;

  // NOTE: Trinary mode still overrides here to preserve existing behavior.
  // Alpha will be averaged in with color channels when using trinary mode.
  const int avg_channels = (mode == MapMode::TRINARY || !img->format->Amask) ? n_channels : n_channels - 1;

  // Copy pixel data into the map structure
  unsigned char* pixels = static_cast<unsigned char*>(img->pixels);
  for(unsigned int j = 0; j < info.height; j++)
  {
    for (unsigned int i = 0; i < info.width; i++)
    {
      // Compute mean of RGB for this pixel
      unsigned char* p = pixels + j*rowstride + i*n_channels;
      int color_sum = 0;
      for(int k = 0; k < avg_channels; k++)
      {
        color_sum += p[k];
      }
      double color_avg = color_sum / (double)avg_channels;

      if (negate)
      {
        color_avg = 255.0 - color_avg;
      }

      int8_t value = 0;

      if (mode == MapMode::RAW)
      {
          value = color_avg;
          resp.map.data[toMapIdx(info.width, i, info.height - j - 1)] = value;
          continue;
      }

      // If negate is true, we consider blacker pixels free, and whiter
      // pixels occupied.  Otherwise, it's vice versa.
      const double occ = (255 - color_avg) / 255.0;

      const int alpha = (n_channels == 1) ? 1 : p[n_channels-1];

      // Apply thresholds to RGB means to determine occupancy values for
      // map.  Note that we invert the graphics-ordering of the pixels to
      // produce a map with cell (0,0) in the lower-left corner.
      if (occ > occ_th)
      {
        value = 100;
      }
      else if (occ < free_th)
      {
        value = 0;
      }
      else if (mode == MapMode::TRINARY || alpha < 1)
      {
        value = -1;
      }
      else 
      {
        const double ratio = (occ - free_th) / (occ_th - free_th);
        value = 99 * ratio;
      }

      resp.map.data[toMapIdx(info.width, i, info.height - j - 1)] = value;
    }
  }

  SDL_FreeSurface(img);
}

}
