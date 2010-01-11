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

/** \author Josh Faust */

/*********************************************************************************
THIS CLASS IS DEPRECATED AND WILL BE REMOVED WHEN PENDING FEATURES TO THE
MESSAGE_FILTER ARE ADDED. PLEASE USE THE MESSAGE FILTER INSTEAD.
*********************************************************************************/

#ifndef TF_MESSAGE_NOTIFIER_H
#define TF_MESSAGE_NOTIFIER_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>
#include <costmap_2d/deprecated/message_notifier_base.h>

#include <string>
#include <list>
#include <vector>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#define NOTIFIER_DEBUG(fmt, ...) \
  ROS_DEBUG_NAMED("message_notifier", "MessageNotifier [topic=%s, target=%s]: "fmt, topic_.c_str(), getTargetFramesString().c_str(), __VA_ARGS__)

#define NOTIFIER_WARN(fmt, ...) \
  ROS_WARN_NAMED("message_notifier", "MessageNotifier [topic=%s, target=%s]: "fmt, topic_.c_str(), getTargetFramesString().c_str(), __VA_ARGS__)

namespace costmap_2d
{

/**
 * \brief For internal use only.
 *
 * In general, allocating memory inside a library's header using new/delete is a good way to cause difficult to find bugs.
 * This provides a way for MessageNotifier to allocate from within the tf library, rather than within the calling library.
 */
void* notifierAllocate(uint32_t size);
/**
 * \brief For internal use only.
 *
 * In general, allocating memory inside a library's header using new/delete is a good way to cause difficult to find bugs.
 * This provides a way for MessageNotifier to allocate from within the tf library, rather than within the calling library.
 */
void notifierDeallocate(void* p);

//class Transformer;

/**
 * \class MessageNotifier
 * \brief Queues messages that include a Header until there is transform data available for the time the timestamp on the message.
 *
 * \deprecated Deprecated in favor of tf::MessageFilter
 * For use instead of extrapolation, MessageNotifier provides a way of waiting until it is possible to transform a message
 * before getting a callback notifying that the message is available.
 *
 * \section callback THE CALLBACK
 * The callback takes one argument, which is a boost shared_ptr to a message.  MessageNotifier provides a MessagePtr typedef,
 * so the signature of the callback is:
 \verbatim
 void funcName(const MessageNotifier<MessageT>::MessagePtr& message);
 \endverbatim
 *
 * A bare function can be passed directly as the callback.  Methods must be passed using boost::bind.  For example
 \verbatim
 boost::bind(&MyObject::funcName, this, _1);
 \endverbatim
 *
 * The message is \b not locked when your callback is invoked.
 *
 * \section threading THREADING
 * MessageNotifier spins up a single thread to call your callback from, so that it's possible to do a lot of work in your callback
 * without blocking the rest of the application.
 *
 \endverbatim

 */
template<class MessageT>
class MessageNotifier : public MessageNotifierBase
{
public:
  typedef boost::shared_ptr<MessageT> MessagePtr;
  typedef boost::function<void(const MessagePtr&)> Callback;

  /**
   * \brief Constructor
   * \param tf The Transformer to use for checking if the transform data is available
   * \param callback The function to call when a message is ready to be transformed
   * \param topic The topic to listen on
   * \param target_frame The frame we need to be able to transform to before a message is ready
   * \param queue_size The number of messages to keep around waiting for transform data.
   * \note A queue size of 0 means infinite, which can be dangerous
   */
  MessageNotifier(tf::Transformer& tf, Callback callback,
      const std::string& topic, const std::string& target_frame,
      uint32_t queue_size)
  : tf_(tf)
  , callback_(callback)
  , queue_size_(queue_size)
  , message_count_(0)
  , destructing_(false)
  , new_messages_(false)
  , new_transforms_(false)
  , successful_transform_count_(0)
  , failed_transform_count_(0)
  , failed_out_the_back_count_(0)
  , transform_message_count_(0)
  , incoming_message_count_(0)
  , dropped_message_count_(0)
  , time_tolerance_(0.0)
  {
    target_frames_.resize(1);
    target_frames_[0] = target_frame;
    target_frames_string_ = target_frame;

    setTopic(topic);

    tf_subscriber_1_ = node_.subscribe<tf::tfMessage>("/tf", 1,
                                                  boost::bind(&MessageNotifier::incomingTFMessage, this, _1));

    tf_subscriber_2_ = node_.subscribe<tf::tfMessage>("/tf_message", 1,
                                                  boost::bind(&MessageNotifier::incomingTFMessage, this, _1));

    thread_handle_ = boost::thread(boost::bind(&MessageNotifier::workerThread, this));
  }

  /**
   * \brief Destructor
   */
  ~MessageNotifier()
  {
    NOTIFIER_DEBUG("Successful Transforms: %llu, Failed Transforms: %llu, Discarded due to age: %llu, Transform messages received: %llu, Messages received: %llu, Total dropped: %llu",
                   (long long unsigned int)successful_transform_count_, (long long unsigned int)failed_transform_count_, 
                   (long long unsigned int)failed_out_the_back_count_, (long long unsigned int)transform_message_count_, 
                   (long long unsigned int)incoming_message_count_, (long long unsigned int)dropped_message_count_);

    unsubscribeFromMessage();


    // Tell the worker thread that we're destructing
    destructing_ = true;
    new_data_.notify_all();

    // Wait for the worker thread to exit
    thread_handle_.join();

    clear();
  }

  /**
   * \brief Set the frame you need to be able to transform to before getting a message callback
   */
  void setTargetFrame(const std::string& target_frame)
  {
    std::vector<std::string> frames;
    frames.push_back(target_frame);
    setTargetFrame(frames);
  }

  /**
   * \brief Set the frames you need to be able to transform to before getting a message callback
   */
  void setTargetFrame(const std::vector<std::string>& target_frames)
  {
    boost::mutex::scoped_lock list_lock(list_mutex_);
    boost::mutex::scoped_lock string_lock(target_frames_string_mutex_);

    target_frames_ = target_frames;
    new_data_.notify_all();

    std::stringstream ss;
    for (std::vector<std::string>::const_iterator it = target_frames_.begin(); it != target_frames_.end(); ++it)
    {
      ss << tf::resolve(tf_.getTFPrefix(), *it) << " ";
    }
    target_frames_string_ = ss.str();
  }
  /**
   * \brief Get the target frames as a string for debugging
   */
  std::string getTargetFramesString()
  {
    boost::mutex::scoped_lock lock(target_frames_string_mutex_);
    return target_frames_string_;
  };

  /**
   * \brief Set the topic to listen on
   */
  void setTopic(const std::string& topic)
  {
    unsubscribeFromMessage();

    topic_ = topic;

    subscribeToMessage();
  }

  /**
   * \brief Set the required tolerance for the notifier to return true
   */
  void setTolerance(const ros::Duration& tolerance)
  {
    time_tolerance_ = tolerance;
  }

  /**
   * \brief Clear any messages currently in the queue
   */
  void clear()
  {
    boost::mutex::scoped_lock list_lock(list_mutex_);
    boost::mutex::scoped_lock queue_lock(queue_mutex_);

    messages_.clear();
    new_message_queue_.clear();
    message_count_ = 0;
  }

  void enqueueMessage(const MessagePtr& message)
  {
    {
      boost::mutex::scoped_lock lock(queue_mutex_);

      new_message_queue_.push_back(message);

      NOTIFIER_DEBUG("Added message to message queue, count now %d", (int)new_message_queue_.size());

      new_messages_ = true;

      // Notify the worker thread that there is new data available
      new_data_.notify_all();
    }

    ++incoming_message_count_;
  }

  /**
   * \brief Subscribe to the message topic
   */
  void subscribeToMessage()
  {
    if (!topic_.empty())
    {
      subscriber_ = node_.subscribe<MessageT>(topic_, queue_size_,
                                              boost::bind(&MessageNotifier::incomingMessage,  this, _1));
    }
  }

  /**
   * \brief Unsubscribe from the message topic
   */
  void unsubscribeFromMessage()
  {
    if (!topic_.empty())
    {
      subscriber_.shutdown();
    }
  }

private:

  typedef std::vector<MessagePtr> V_Message;
  typedef std::list<MessagePtr> L_Message;

  /**
   * \brief Gather any messages ready to be transformed
   * \param to_notify Filled with the messages ready to be transformed, in the order they were received
   * \note Assumes the message list is already locked
   */
  void gatherReadyMessages(V_Message& to_notify)
  {
    if (!messages_.empty() && getTargetFramesString() == " ")
    {
      ROS_WARN_NAMED("message_notifier", "MessageNotifier [topic=%s, target=%s]: empty target frame", topic_.c_str(), getTargetFramesString().c_str());
    }

    to_notify.reserve(message_count_);

    int i = 0;

    typename L_Message::iterator it = messages_.begin();
    for (; it != messages_.end(); ++i)
    {
      MessagePtr& message = *it;

      bool should_step_out = false;
      //Throw out messages which are too old
      //! \todo combine getLatestCommonTime call with the canTransform call
      for (std::vector<std::string>::iterator target_it = target_frames_.begin(); target_it != target_frames_.end(); ++target_it)
      {
        const std::string& target_frame = *target_it;

        if (target_frame != message->header.frame_id)
        {
          ros::Time latest_transform_time ;
          std::string error_string ;

          tf_.getLatestCommonTime(message->header.frame_id, target_frame, latest_transform_time, &error_string) ;
          if (message->header.stamp + tf_.getCacheLength() < latest_transform_time)
          {
            --message_count_;
            ++failed_out_the_back_count_;
            ++dropped_message_count_;
            NOTIFIER_DEBUG("Discarding Message %d , in frame %s, Out of the back of Cache Time(stamp: %.3f + cache_length: %.3f < latest_transform_time %.3f.  Message Count now: %d", i, message->header.frame_id.c_str(), message->header.stamp.toSec(),  tf_.getCacheLength().toSec(), latest_transform_time.toSec(), message_count_);

            last_out_the_back_stamp_ = message->header.stamp;
            last_out_the_back_frame_ = message->header.frame_id;
            it = messages_.erase(it);
            should_step_out = true;
            break;
          }
        }

      }
      if (should_step_out) //If we just deleted a message don't try to use it
      {
        should_step_out = false;
        continue;
      }

      bool ready = !target_frames_.empty();
      for (std::vector<std::string>::iterator target_it = target_frames_.begin(); ready && target_it != target_frames_.end(); ++target_it)
      {
        std::string& target_frame = *target_it;
        if (time_tolerance_ != ros::Duration(0.0))
        {
          ready = ready && (tf_.canTransform(target_frame, message->header.frame_id, message->header.stamp) &&
                            tf_.canTransform(target_frame, message->header.frame_id, message->header.stamp + time_tolerance_) );
        }
        else
        {
          ready = ready && tf_.canTransform(target_frame, message->header.frame_id, message->header.stamp);
        }
      }

      if (ready)
      {
        // If we get here the transform succeeded, so push the message onto the notify list, and erase it from or message list
        to_notify.push_back(message);

        --message_count_;

        NOTIFIER_DEBUG("Message %d ready in frame %s at time %.3f, count now %d", i, message->header.frame_id.c_str(), message->header.stamp.toSec(), message_count_);

        it = messages_.erase(it);

        ++successful_transform_count_;
      }
      else
      {
        ++it;
        ++failed_transform_count_;
      }
    }
  }

  /**
   * \brief Calls the notification callback on each message in the passed vector
   * \param to_notify The list of messages to call the callback on
   */
  void notify(V_Message& to_notify)
  {
    typename V_Message::iterator it = to_notify.begin();
    typename V_Message::iterator end = to_notify.end();
    for (; it != end; ++it)
    {
      callback_(*it);
    }
  }

  /**
   * \brief Adds messages into the message list, removing old messages if necessary
   */
  void processNewMessages(V_Message& messages)
  {
    typename V_Message::iterator it = messages.begin();
    typename V_Message::iterator end = messages.end();
    for (; it != end; ++it)
    {
      MessagePtr& message = *it;

      // If this message is about to push us past our queue size, erase the oldest message
      if (queue_size_ != 0 && message_count_ + 1 > queue_size_)
      {
        ++dropped_message_count_;
        NOTIFIER_DEBUG("Removed oldest message because buffer is full, count now %d (frame_id=%s, stamp=%f)", message_count_, messages_.front()->header.frame_id.c_str(), messages_.front()->header.stamp.toSec());
        messages_.pop_front();
        --message_count_;


      }

      // Add the message to our list
      messages_.push_back(message);
      ++message_count_;

      NOTIFIER_DEBUG("Added message in frame %s at time %.3f, count now %d", message->header.frame_id.c_str(), message->header.stamp.toSec(), message_count_);
    }
  }

  /**
   * \brief Entry point into the worker thread that does all our actual work, including calling the notification callback
   */
  void workerThread()
  {
    V_Message to_notify;
    while (!destructing_)
    {
      V_Message local_queue;

      {
        boost::mutex::scoped_lock lock(queue_mutex_);

        // Wait for new data to be available
        while (!destructing_ && ((message_count_ == 0 && new_message_queue_.size() == 0) || (!new_transforms_ && !new_messages_)))
        {
          new_data_.wait(lock);
        }

        // If we're destructing, break out of the loop
        if (destructing_)
        {
          break;
        }

        local_queue.swap(new_message_queue_);

        new_messages_ = false;
      }

      {
        // Outside the queue lock, gather and notify that the messages are ready
        // Need to lock the list mutex because clear() can modify the message list
        boost::mutex::scoped_lock lock(list_mutex_);
        processNewMessages(local_queue);

        local_queue.clear();

        gatherReadyMessages(to_notify);

        new_transforms_ = false;

        notify(to_notify);
        to_notify.clear();

        checkFailures();
      }
    }
  }


  /**
   * \class MessageDeleter
   * \brief Since we allocate with our own special functions, we need to also delete using them.  This provides a deletion interface for the boost::shared_ptr
   */
  class MessageDeleter
  {
  public:
    void operator()(MessageT* m)
    {
      m->~MessageT();
      notifierDeallocate(m);
    }
  };

  /**
   * \brief Callback that happens when we receive a message on the message topic
   */
  void incomingMessage(typename MessageT::ConstPtr msg)
  {
    // Allocate our new message and placement-new it
    MessageT* mem = (MessageT*) notifierAllocate(sizeof(MessageT));
    new (mem) MessageT();

    // Create a boost::shared_ptr from the message, with our custom deleter
    MessagePtr message(mem, MessageDeleter());
    // Copy the message
    *message = *msg;

    enqueueMessage(message);
  }

  /**
   * \brief Callback that happens when we receive a message on the TF message topic
   */
  void incomingTFMessage(const tf::tfMessage::ConstPtr msg)
  {
    // Notify the worker thread that there is new data available
    new_data_.notify_all();
    new_transforms_ = true;
    ++transform_message_count_;
  }

  void checkFailures()
  {
    if (next_failure_warning_.isZero())
    {
      next_failure_warning_ = ros::Time::now() + ros::Duration(15);
    }

    if (ros::Time::now() >= next_failure_warning_)
    {
      if (incoming_message_count_ - message_count_ == 0)
      {
        return;
      }

      double dropped_pct = (double)dropped_message_count_ / (double)(incoming_message_count_ - message_count_);
      if (dropped_pct > 0.95)
      {
        NOTIFIER_WARN("Dropped %.2f%% of messages so far. Please turn the [%s.message_notifier] rosconsole logger to DEBUG for more information.", dropped_pct*100, ROSCONSOLE_DEFAULT_NAME);
        next_failure_warning_ = ros::Time::now() + ros::Duration(60);

        if ((double)failed_out_the_back_count_ / (double)dropped_message_count_ > 0.5)
        {
          NOTIFIER_WARN("  The majority of dropped messages were due to messages growing older than the TF cache time.  The last message's timestamp was: %f, and the last frame_id was: %s", last_out_the_back_stamp_.toSec(), last_out_the_back_frame_.c_str());
        }
      }
    }
  }

  tf::Transformer& tf_; ///< The Transformer used to determine if transformation data is available
  ros::NodeHandle node_; ///< The node used to subscribe to the topic
  ros::Subscriber subscriber_;
  ros::Subscriber tf_subscriber_1_, tf_subscriber_2_;
  Callback callback_; ///< The callback to call when a message is ready
  std::vector<std::string> target_frames_; ///< The frames we need to be able to transform to before a message is ready
  std::string target_frames_string_;
  boost::mutex target_frames_string_mutex_;
  std::string topic_; ///< The topic to listen on
  uint32_t queue_size_; ///< The maximum number of messages we queue up

  L_Message messages_; ///< The message list
  uint32_t message_count_; ///< The number of messages in the list.  Used because messages_.size() has linear cost
  boost::mutex list_mutex_; ///< The mutex used for locking message list operations

  bool destructing_; ///< Used to notify the worker thread that it needs to shutdown
  boost::thread thread_handle_; ///< Thread handle for the worker thread
  boost::condition_variable new_data_; ///< Condition variable used for waking the worker thread
  bool new_messages_; ///< Used to skip waiting on new_data_ if new messages have come in while calling back
  volatile bool new_transforms_; ///< Used to skip waiting on new_data_ if new transforms have come in while calling back or transforming data
  V_Message new_message_queue_; ///< Queues messages to later be processed by the worker thread
  boost::mutex queue_mutex_; ///< The mutex used for locking message queue operations

  uint64_t successful_transform_count_;
  uint64_t failed_transform_count_;
  uint64_t failed_out_the_back_count_;
  uint64_t transform_message_count_;
  uint64_t incoming_message_count_;
  uint64_t dropped_message_count_;

  ros::Time last_out_the_back_stamp_;
  std::string last_out_the_back_frame_;

  ros::Time next_failure_warning_;

  ros::Duration time_tolerance_; ///< Provide additional tolerance on time for messages which are stamped but can have associated duration
};

} // namespace tf

#endif
