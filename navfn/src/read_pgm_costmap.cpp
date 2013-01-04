/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include <navfn/navfn.h>
#include <stdlib.h>
#include <stdint.h>

extern "C" {
#include <stdio.h>
// pgm.h is not very friendly with system headers... need to undef max() and min() afterwards
#include <pgm.h>
#undef max
#undef min
}

void
setcostobs(COSTTYPE *cmap, int n, int w)
{
  int CS = 11;
  for (int i=-CS/2; i<CS/2; i++)
    {
      COSTTYPE *cm = i*w + &cmap[n];
      for (int j=-CS/2; j<CS/2; j++)
	cm[j] = COST_NEUTRAL + 50;
    }
  CS = 7;
  for (int i=-CS/2; i<CS/2; i++)
    {
      COSTTYPE *cm = i*w + &cmap[n];
      for (int j=-CS/2; j<CS/2; j++)
	cm[j] = COST_OBS;
    }
}

void setcostunk(COSTTYPE *cmap, int n, int w)
{
  cmap[n] = COST_OBS;
}

#define unknown_gray 0xCC	// seems to be the value of "unknown" in maps

COSTTYPE *
readPGM(const char *fname, int *width, int *height, bool raw)
{
  pm_init("navfn_tests",0);

  FILE *pgmfile;
  pgmfile = fopen(fname,"r");
  if (!pgmfile)
  {
    printf("readPGM() Can't find file %s\n", fname);
    return NULL;
  }

  printf("readPGM() Reading costmap file %s\n", fname);
  int ncols, nrows;
  gray maxval;
  int format;
  pgm_readpgminit(pgmfile, &ncols, &nrows, &maxval, &format);
  printf("readPGM() Size: %d x %d\n", ncols, nrows);

  // set up cost map
  COSTTYPE *cmap = (COSTTYPE *)malloc(ncols*nrows*sizeof(COSTTYPE));
  if (!raw)
    for (int i=0; i<ncols*nrows; i++)
      cmap[i] = COST_NEUTRAL;

  gray * row(pgm_allocrow(ncols));
  int otot = 0;
  int utot = 0;
  int ftot = 0;
  for (int ii = 0; ii < nrows; ii++) {
    pgm_readpgmrow(pgmfile, row, ncols, maxval, format);
    if (raw)			// raw costmap from ROS
    {
      for (int jj(ncols - 1); jj >= 0; --jj)
      {
        int v = row[jj];
        cmap[ii*ncols+jj] = v;
        if (v >= COST_OBS_ROS)
          otot++;
        if (v == 0)
          ftot++;
      }
    }
    else
    {
      ftot = ncols*nrows;
      for (int jj(ncols - 1); jj >= 0; --jj)
      {
        if (row[jj] < unknown_gray && ii < nrows-7 && ii > 7)
        {
          setcostobs(cmap,ii*ncols+jj,ncols);
          otot++;
          ftot--;
        }
        else if (row[jj] <= unknown_gray)
        {
          setcostunk(cmap,ii*ncols+jj,ncols);
          utot++;
          ftot--;
        }
      }
    }
  }
  printf("readPGM() Found %d obstacle cells, %d free cells, %d unknown cells\n", otot, ftot, utot);
  pgm_freerow(row);
  *width = ncols;
  *height = nrows;
  return cmap;
}
