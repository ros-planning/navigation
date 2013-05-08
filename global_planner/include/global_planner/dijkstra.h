/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
#ifndef _DIJKSTRA_H
#define _DIJKSTRA_H

#define PRIORITYBUFSIZE 10000
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include <global_planner/planner_core.h>
#include <global_planner/expander.h>

// inserting onto the priority blocks
#define push_cur(n)  { if (n>=0 && n<ns_ && !pending_[n] && getCost(costs, n)<lethal_cost_ && currentEnd_<PRIORITYBUFSIZE){ currentBuffer_[currentEnd_++]=n; pending_[n]=true; }}
#define push_next(n) { if (n>=0 && n<ns_ && !pending_[n] && getCost(costs, n)<lethal_cost_ &&    nextEnd_<PRIORITYBUFSIZE){    nextBuffer_[   nextEnd_++]=n; pending_[n]=true; }}
#define push_over(n) { if (n>=0 && n<ns_ && !pending_[n] && getCost(costs, n)<lethal_cost_ &&    overEnd_<PRIORITYBUFSIZE){    overBuffer_[   overEnd_++]=n; pending_[n]=true; }}
// potential defs
#define POT_HIGH 1.0e10		// unassigned cell potential

namespace global_planner {
class DijkstraExpansion : public Expander {
    public:
        DijkstraExpansion(PotentialCalculator* p_calc, int nx, int ny);
        bool calculatePotentials(unsigned char* costs, int start_x, int start_y, int end_x, int end_y, int cycles,
                                float* potential);

        /**
         * @brief  Sets or resets the size of the map
         * @param nx The x size of the map
         * @param ny The y size of the map
         */
        void setSize(int nx, int ny); /**< sets or resets the size of the map */

        void setNeutralCost(unsigned char neutral_cost) {
            neutral_cost_ = neutral_cost;
            priorityIncrement_ = 2 * neutral_cost_;
        }
    private:

        /**
         * @brief  Updates the cell at index n
         * @param n The index to update
         */
        void updateCell(unsigned char* costs, float* potential, int n); /**< updates the cell at index <n> */

        float getCost(unsigned char* costs, int n) {
            float c = costs[n];
            if (c < lethal_cost_ - 1) {
                c = c * factor_ + neutral_cost_;
                if (c >= lethal_cost_)
                    c = lethal_cost_ - 1;
                return c;
            }
            return lethal_cost_;
        }

        /** block priority buffers */
        int *buffer1_, *buffer2_, *buffer3_; /**< storage buffers for priority blocks */
        int *currentBuffer_, *nextBuffer_, *overBuffer_; /**< priority buffer block ptrs */
        int currentEnd_, nextEnd_, overEnd_; /**< end points of arrays */
        bool *pending_; /**< pending_ cells during propagation */

        /** block priority thresholds */
        float threshold_; /**< current threshold */
        float priorityIncrement_; /**< priority threshold increment */

};
} //end namespace global_planner
#endif
