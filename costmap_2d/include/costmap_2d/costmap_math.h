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
 *         David V. Lu!!
 *********************************************************************/
#ifndef COSTMAP_MATH_H_
#define COSTMAP_MATH_H_

#include<math.h>
#include<geometry_msgs/Polygon.h>
#include<algorithm>

inline double sign(double x) {
    return x < 0.0 ? -1.0 : 1.0;
}

inline double distance(double x0, double y0, double x1, double y1) {
    return sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
}

double distanceToLine(double pX, double pY, double x0, double y0, double x1, double y1) {
    double A = pX - x0;
    double B = pY - y0;
    double C = x1 - x0;
    double D = y1 - y0;

    double dot = A * C + B * D;
    double len_sq = C * C + D * D;
    double param = dot / len_sq;

    double xx, yy;

    if (param < 0) {
        xx = x0;
        yy = y0;
    } else if (param > 1) {
        xx = x1;
        yy = y1;
    } else {
        xx = x0 + param * C;
        yy = y0 + param * D;
    }

    return distance(pX, pY, xx, yy);
}

bool intersects(geometry_msgs::Polygon* polygon, float testx, float testy) {
    bool c = false;
    int i, j, nvert = polygon->points.size();
    for (i = 0, j = nvert - 1; i < nvert; j = i++) {
        float yi = polygon->points[i].y, yj = polygon->points[j].y, xi = polygon->points[i].x,
              xj = polygon->points[j].x;

        if (((yi > testy) != (yj > testy)) && (testx < (xj - xi) * (testy - yi) / (yj - yi) + xi))
            c = !c;
    }
    return c;
}

bool intersects_helper(geometry_msgs::Polygon* polygon1, geometry_msgs::Polygon* polygon2) {
    for (unsigned int i = 0; i < polygon1->points.size(); i++)
        if (intersects(polygon2, polygon1->points[i].x, polygon1->points[i].y))
            return true;
    return false;
}

bool intersects(geometry_msgs::Polygon* polygon1, geometry_msgs::Polygon* polygon2) {
    return intersects_helper(polygon1, polygon2) || intersects_helper(polygon2, polygon1);
}

#endif
