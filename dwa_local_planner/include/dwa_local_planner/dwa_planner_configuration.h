/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, 6 River Systems
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
* Author: Daniel Grieneisen
*********************************************************************/
#ifndef DWA_LOCAL_PLANNER_DWA_PLANNER_CONFIGURATION_H_
#define DWA_LOCAL_PLANNER_DWA_PLANNER_CONFIGURATION_H_

#include <dwa_local_planner/DWAPlannerConfig.h>
#include <string>

namespace dwa_local_planner {
  /**
   * @class DWAPlanner
   * @brief A class implementing a local planner using the Dynamic Window Approach
   */
  class DWAPlannerConfiguration {
    public:
      /**
       * @brief  Constructor for the planner mode
       * @param name The name of the mode
       */
      DWAPlannerConfiguration(std::string name) : name_(name), initialized(false)
      {
        ros::NodeHandle private_nh("~/controller/" + name);
        dsrv_ = new dynamic_reconfigure::Server<DWAPlannerConfig>(private_nh);
        dynamic_reconfigure::Server<DWAPlannerConfig>::CallbackType cb
            = boost::bind(&DWAPlannerConfiguration::reconfigure, this, _1, _2);
        dsrv_->setCallback(cb);
      };

      /**
       * @brief  Destructor for the planner
       */
      ~DWAPlannerConfiguration() {}

      /**
       * @brief Reconfigures the dwa planner mode
       */
      void reconfigure(DWAPlannerConfig &cfg, uint32_t level)
      {
        boost::mutex::scoped_lock l(mutex_);

        if (initialized && cfg.restore_defaults)
        {
          config_ = default_config_;
        }
        else
        {
          config_ = cfg;
        }

        if (!initialized)
        {
          initialized = true;
          default_config_ = cfg;
        }
      }

      DWAPlannerConfig getConfig()
      {
        boost::mutex::scoped_lock l(mutex_);
        return config_;
      }

    private:
      DWAPlannerConfig config_;
      DWAPlannerConfig default_config_;
      bool initialized;
      dynamic_reconfigure::Server<DWAPlannerConfig> *dsrv_;
      boost::mutex mutex_;
      std::string name_;
  };
};
#endif
