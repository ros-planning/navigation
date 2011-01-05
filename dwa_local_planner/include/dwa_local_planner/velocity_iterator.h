/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*********************************************************************/
#ifndef DWA_LOCAL_PLANNER_VELOCITY_ITERATOR_H_
#define DWA_LOCAL_PLANNER_VELOCITY_ITERATOR_H_
#include <algorithm>

namespace dwa_local_planner {
  class VelocityIterator {
    public:
      VelocityIterator(double min, double max, double step_size):
        min_(min),
        max_(max),
        step_size_(step_size),
        current_sample_(min),
        finished_(false)
      {
      }

      double getVelocity(){
        return current_sample_;
      }

      VelocityIterator& operator++(int){
        if(current_sample_ == max_){
          finished_ = true;
          return *this;
        }

        double next_sample_ = current_sample_ + step_size_;

        if(next_sample_ * current_sample_ < 0.0)
          current_sample_ = 0.0;
        else
          current_sample_ = next_sample_;

        if(current_sample_ >= max_)
          current_sample_ = max_;

        return *this;
      }

      bool isFinished(){
        return finished_;
      }

    private:
      double min_, max_, step_size_, current_sample_;
      bool finished_;
  };
};
#endif
