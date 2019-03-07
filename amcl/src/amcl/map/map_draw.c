/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/**************************************************************************
 * Desc: Local map GUI functions
 * Author: Andrew Howard
 * Date: 18 Jan 2003
 * CVS: $Id: map_draw.c 7057 2008-10-02 00:44:06Z gbiggs $
**************************************************************************/

#ifdef INCLUDE_RTKGUI

#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <rtk.h>
#include "amcl/map/map.h"


void map_draw(map_t* map, rtk_fig_t* fig, void* col_function)
{
  uint16_t* image = malloc(map->size_x * map->size_y * sizeof(image[0]));

  // Draw occupancy
  int col;
  map_cell_t* cell;
  uint16_t* pixel;
  for (int j = 0; j < map->size_y; ++j)
  {
    for (int i =  0; i < map->size_x; ++i)
    {
      cell = map->cells + MAP_INDEX(map, i, j);
      pixel = image + (j * map->size_x + i);

      col = col_function(cell, map);
      *pixel = RTK_RGB16(col, col, col);
    }
  }

  // Draw the entire occupancy map as an image
  rtk_fig_image(fig, map->origin_x, map->origin_y, 0,
                map->scale, map->size_x, map->size_y, 16, image, NULL);

  free(image);

  return;
}


////////////////////////////////////////////////////////////////////////////
// Draw the occupancy map
void col_occ(map_cell_t* cell, map_t* map)
{
  return 127 - 127 * cell->occ_state;
}


void map_draw_occ(map_t* map, rtk_fig_t* fig)
{
  map_draw(map, fig, &col_occ);
}


////////////////////////////////////////////////////////////////////////////
// Draw the cspace map
void col_cspace(map_cell_t* cell, map_t* map)
{
  return 255 * cell->occ_dist / map->max_occ_dist;
}


void map_draw_cspace(map_t* map, rtk_fig_t* fig)
{
  map_draw(map, fig, &col_cspace);
}


////////////////////////////////////////////////////////////////////////////
// Draw a wifi map
void map_draw_wifi(map_t* map, rtk_fig_t* fig, int index)
{
  uint16_t* image = malloc(map->size_x * map->size_y * sizeof(image[0]));
  uint16_t* mask = malloc(map->size_x * map->size_y * sizeof(mask[0]));

  // Draw wifi levels
  for (int j = 0; j < map->size_y; ++j)
  {
    for (int i =  0; i < map->size_x; ++i)
    {
      map_cell_t* cell = map->cells + MAP_INDEX(map, i, j);
      uint16_t* ipix = image + (j * map->size_x + i);
      uint16_t* mpix = mask + (j * map->size_x + i);

      int level = cell->wifi_levels[index];

      if (cell->occ_state == -1 && level != 0)
      {
        int col = 255 * (100 + level) / 100;
        *ipix = RTK_RGB16(col, col, col);
        *mpix = 1;
      }
      else
      {
        *mpix = 0;
      }
    }
  }

  // Draw the entire occupancy map as an image
  rtk_fig_image(fig, map->origin_x, map->origin_y, 0,
                map->scale, map->size_x, map->size_y, 16, image, mask);

  free(mask);
  free(image);

  return;
}


#endif
