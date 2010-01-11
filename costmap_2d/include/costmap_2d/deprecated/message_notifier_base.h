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

/*********************************************************************************
THIS CLASS IS DEPRECATED AND WILL BE REMOVED WHEN PENDING FEATURES TO THE
MESSAGE_FILTER ARE ADDED. PLEASE USE THE MESSAGE FILTER INSTEAD.
*********************************************************************************/

#ifndef MESSAGE_NOTIFIER_BASE_H_
#define MESSAGE_NOTIFIER_BASE_H_
#include <tf/tf.h>

#include <string>
#include <list>
#include <vector>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

namespace costmap_2d {
  /**
   * @class MessageNotifierBase
   * @brief An abstract base class for the various types of message notifiers
   */
  class MessageNotifierBase
  {
    public:

      /**
       * \brief Destructor
       */
      virtual ~MessageNotifierBase(){}

      /**
       * \brief Set the frame you need to be able to transform to before getting a message callback
       */
      virtual void setTargetFrame(const std::string& target_frame) = 0;

      /**
       * \brief Set the frame you need to be able to transform to before getting a message callback
       */
      virtual void setTargetFrame(const std::vector<std::string>& target_frames) = 0;

      /**
       * \brief Set the topic to listen on
       */
      virtual void setTopic(const std::string& topic) = 0;

      /**
       * \brief Set the required tolerance for the notifier to return true
       */
      virtual void setTolerance(const ros::Duration& tolerance) = 0;

      /**
       * \brief Clear any messages currently in the queue
       */
      virtual void clear() = 0;

      /**
       * \brief Subscribe to the message topic
       */
      virtual void subscribeToMessage() = 0;

      /**
       * \brief Unsubscribe from the message topic
       */
      virtual void unsubscribeFromMessage() = 0;


    protected:
      MessageNotifierBase(){}

  };
};
#endif
