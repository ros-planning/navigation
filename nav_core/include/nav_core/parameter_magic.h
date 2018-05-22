/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, Open Source Robotics Foundation
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
*   * Neither the name of OSRF nor the names of its
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
* Author: David Lu
*********************************************************************/

#ifndef NAV_CORE_PARAMETER_MAGIC_H
#define NAV_CORE_PARAMETER_MAGIC_H

namespace nav_core
{

/**
 * @brief Load a parameter from one of two namespaces. Complain if it uses the old name.
 * @param nh NodeHandle to look for the parameter in
 * @param current_name Parameter name that is current, i.e. not deprecated
 * @param old_name Deprecated parameter name
 * @param default_value If neither parameter is present, return this value
 * @return The value of the parameter or the default value
 */
template<class param_t>
param_t loadParameterWithDeprecation(const ros::NodeHandle& nh, const std::string current_name,
                                     const std::string old_name, const param_t& default_value)
{
  param_t value;
  if (nh.hasParam(current_name))
  {
    nh.getParam(current_name, value);
    return value;
  }
  if (nh.hasParam(old_name))
  {
    ROS_WARN("Parameter %s is deprecated. Please use the name %s instead.", old_name.c_str(), current_name.c_str());
    nh.getParam(old_name, value);
    return value;
  }
  return default_value;
}

/**
 * @brief Warn if a parameter exists under a deprecated (and unsupported) name.
 *
 * Parameters loaded exclusively through dynamic reconfigure can't really use loadParamWithDeprecation.
 */
void warnRenamedParameter(const ros::NodeHandle& nh, const std::string current_name, const std::string old_name)
{
  if (nh.hasParam(old_name))
  {
    ROS_WARN("Parameter %s is deprecated (and will not load properly). Use %s instead.", old_name.c_str(), current_name.c_str());
  }
}

}  // namespace nav_core

#endif  // NAV_CORE_PARAMETER_MAGIC_H
