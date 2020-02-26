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

#include <base_local_planner/point_grid.h>
#include <ros/console.h>
#ifdef HAVE_SYS_TIME_H
#include <sys/time.h>
#endif

#include <math.h>
#include <cstdio>
#include <sensor_msgs/point_cloud2_iterator.h>

using namespace std;
using namespace costmap_2d;

void printPoint(const geometry_msgs::Point& pt){
  printf("(%.2f, %.2f, %.2f)", pt.x, pt.y, pt.z);
}

void printPSHeader(){
  printf("%%!PS\n");
  printf("%%%%Creator: Eitan Marder-Eppstein (Willow Garage)\n");
  printf("%%%%EndComments\n");
}

void printPSFooter(){
  printf("showpage\n%%%%EOF\n");
}

void printPolygonPS(const std::vector<geometry_msgs::Point32>& poly, double line_width){
  if(poly.size() < 2)
    return;

  printf("%.2f setlinewidth\n", line_width);
  printf("newpath\n");
  printf("%.4f\t%.4f\tmoveto\n", poly[0].x * 10, poly[0].y * 10);
  for(unsigned int i = 1; i < poly.size(); ++i)
    printf("%.4f\t%.4f\tlineto\n", poly[i].x * 10, poly[i].y * 10);
  printf("%.4f\t%.4f\tlineto\n", poly[0].x * 10, poly[0].y * 10);
  printf("closepath stroke\n");

}

using namespace base_local_planner;

int main(int argc, char** argv){
  geometry_msgs::Point origin;
  origin.x = 0.0;
  origin.y = 0.0;
  PointGrid pg(50.0, 50.0, 0.2, origin, 2.0, 3.0, 0.0);
  /*
     double x = 10.0;
     double y = 10.0;
     for(int i = 0; i < 100; ++i){
     for(int j = 0; j < 100; ++j){
     pcl::PointXYZ pt;
     pt.x = x;
     pt.y = y;
     pt.z = 1.0;
     pg.insert(pt);
     x += .03;
     }
     y += .03;
     x = 10.0;
     }
     */
  std::vector<geometry_msgs::Point> footprint, footprint2, footprint3;
  geometry_msgs::Point pt;

  pt.x = 1.0;
  pt.y = 1.0;
  footprint.push_back(pt);

  pt.x = 1.0;
  pt.y = 1.65;
  footprint.push_back(pt);

  pt.x = 1.325;
  pt.y = 1.75;
  footprint.push_back(pt);

  pt.x = 1.65;
  pt.y = 1.65;
  footprint.push_back(pt);

  pt.x = 1.65;
  pt.y = 1.0;
  footprint.push_back(pt);

  pt.x = 1.325;
  pt.y = 1.00;
  footprint2.push_back(pt);

  pt.x = 1.325;
  pt.y = 1.75;
  footprint2.push_back(pt);

  pt.x = 1.65;
  pt.y = 1.75;
  footprint2.push_back(pt);

  pt.x = 1.65;
  pt.y = 1.00;
  footprint2.push_back(pt);

  pt.x = 0.99;
  pt.y = 0.99;
  footprint3.push_back(pt);

  pt.x = 0.99;
  pt.y = 1.66;
  footprint3.push_back(pt);

  pt.x = 1.3255;
  pt.y = 1.85;
  footprint3.push_back(pt);

  pt.x = 1.66;
  pt.y = 1.66;
  footprint3.push_back(pt);

  pt.x = 1.66;
  pt.y = 0.99;
  footprint3.push_back(pt);

  pt.x = 1.325;
  pt.y = 1.325;

  geometry_msgs::Point32 point;
  point.x = 1.2;
  point.y = 1.2;
  point.z = 1.0;

#ifdef HAVE_SYS_TIME_H
  struct timeval start, end;
  double start_t, end_t, t_diff;
#endif

  printPSHeader();

#ifdef HAVE_SYS_TIME_H
  gettimeofday(&start, NULL);
#endif

  for(unsigned int i = 0; i < 2000; ++i){
    pg.insert(point);
  }

#ifdef HAVE_SYS_TIME_H
  gettimeofday(&end, NULL);
  start_t = start.tv_sec + double(start.tv_usec) / 1e6;
  end_t = end.tv_sec + double(end.tv_usec) / 1e6;
  t_diff = end_t - start_t;
  printf("%%Insertion Time: %.9f \n", t_diff);
#endif

  vector<Observation> obs;
  vector<PlanarLaserScan> scan;

#ifdef HAVE_SYS_TIME_H
  gettimeofday(&start, NULL);
#endif
  pg.updateWorld(footprint, obs, scan);

  double legal = pg.footprintCost(pt, footprint, 0.0, .95);
  pg.updateWorld(footprint, obs, scan);
  double legal2 = pg.footprintCost(pt, footprint, 0.0, .95);

#ifdef HAVE_SYS_TIME_H
  gettimeofday(&end, NULL);
  start_t = start.tv_sec + double(start.tv_usec) / 1e6;
  end_t = end.tv_sec + double(end.tv_usec) / 1e6;
  t_diff = end_t - start_t;

  printf("%%Footprint calc: %.9f \n", t_diff);
#endif

  if(legal >= 0.0)
    printf("%%Legal footprint %.4f, %.4f\n", legal, legal2);
  else
    printf("%%Illegal footprint\n");

  printPSFooter();

  return 0;
}
