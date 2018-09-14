/*
 * (c) Copyright 2015-2016 6 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <unistd.h>
#include <sched.h>
#include <sys/types.h>
#include <sys/syscall.h>
#include <iostream>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <sys/types.h>

#ifndef _GNU_SOURCE
	#define _GNU_SOURCE
#endif

bool setThreadAffinity(int core_id)
{
   int num_cores = sysconf(_SC_NPROCESSORS_ONLN);
   if (core_id < 0 || core_id >= num_cores)
   {
      return false;
   }

   cpu_set_t cpuset;
   CPU_ZERO(&cpuset);
   CPU_SET(core_id, &cpuset);

   ROS_INFO_STREAM("Setting core: " << core_id << " for thread " <<  syscall(SYS_gettid));

   pthread_t current_thread = pthread_self();

   int res = pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
   return res == 0;
}

void niceThread(std::string name, int priority)
{
    pid_t tid = syscall(SYS_gettid);

    ROS_INFO_STREAM("Setting thread priority: " << priority << " for " << name << " on thread " << tid);

    if (::setpriority(PRIO_PROCESS, tid, priority))
    {
        ROS_ERROR("Could not set %s thread nice value to %d", name.c_str(), priority);
    }
}
