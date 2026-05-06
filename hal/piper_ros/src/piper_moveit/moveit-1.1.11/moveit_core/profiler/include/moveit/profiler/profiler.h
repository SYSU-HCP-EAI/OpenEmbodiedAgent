/*********************************************************************
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
 *   * Neither the name of Willow Garage nor the names of its
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
 *********************************************************************/

/* Author: Ioan Sucan */

#pragma once

#define MOVEIT_ENABLE_PROFILING 1

#ifndef MOVEIT_ENABLE_PROFILING

/** The ENABLE_PROFILING macro can be set externally. If it is not,
    profiling is enabled by default, unless NDEBUG is defined. */

#ifdef NDEBUG
#define MOVEIT_ENABLE_PROFILING 0
#else
#define MOVEIT_ENABLE_PROFILING 1
#endif

#endif

#if MOVEIT_ENABLE_PROFILING

#include <map>
#include <string>
#include <iostream>
#include <boost/thread.hpp>
#include <boost/noncopyable.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace moveit
{
namespace tools
{
/** This is a simple thread-safe tool for counting time
    spent in various chunks of code. This is different from
    external profiling tools in that it allows the user to count
    time spent in various bits of code (sub-function granularity)
    or count how many times certain pieces of code are executed.*/
class Profiler : private boost::noncopyable
{
public:
  /** \brief This instance will call Profiler::begin() when constructed and Profiler::end() when it goes out of scope.
   */
  class ScopedBlock
  {
  public:
    /** \brief Start counting time for the block named \e name of the profiler \e prof */
    ScopedBlock(const std::string& name, Profiler& prof = Profiler::instance()) : name_(name), prof_(prof)
    {
      prof_.begin(name);
    }

    ~ScopedBlock()
    {
      prof_.end(name_);
    }

  private:
    std::string name_;
    Profiler& prof_;
  };

  /** \brief This instance will call Profiler::start() when constructed and Profiler::stop() when it goes out of scope.
      If the profiler was already started, this block's constructor and destructor take no action */
  class ScopedStart
  {
  public:
    /** \brief Take as argument the profiler instance to operate on (\e prof) */
    ScopedStart(Profiler& prof = Profiler::instance()) : prof_(prof), wasRunning_(prof_.running())
    {
      if (!wasRunning_)
        prof_.start();
    }

    ~ScopedStart()
    {
      if (!wasRunning_)
        prof_.stop();
    }

  private:
    Profiler& prof_;
    bool wasRunning_;
  };

  /** \brief Return an instance of the class */
  static Profiler& instance();

  /** \brief Constructor. It is allowed to separately instantiate this
      class (not only as a singleton) */
  Profiler(bool printOnDestroy = false, bool autoStart = false) : running_(false), printOnDestroy_(printOnDestroy)
  {
    if (autoStart)
      start();
  }

  /** \brief Destructor */
  ~Profiler()
  {
    if (printOnDestroy_ && !data_.empty())
      status();
  }

  /** \brief Start counting time */
  static void Start()  // NOLINT(readability-identifier-naming)
  {
    instance().start();
  }

  /** \brief Stop counting time */
  static void Stop()  // NOLINT(readability-identifier-naming)
  {
    instance().stop();
  }

  /** \brief Clear counted time and events */
  static void Clear()  // NOLINT(readability-identifier-naming)
  {
    instance().clear();
  }

  /** \brief Start counting time */
  void start();

  /** \brief Stop counting time */
  void stop();

  /** \brief Clear counted time and events */
  void clear();

  /** \brief Count a specific event for a number of times */
  static void Event(const std::string& name, const unsigned int times = 1)  // NOLINT(readability-identifier-naming)
  {
    instance().event(name, times);
  }

  /** \brief Count a specific event for a number of times */
  void event(const std::string& name, const unsigned int times = 1);

  /** \brief Maintain the average of a specific value */
  static void Average(const std::string& name, const double value)  // NOLINT(readability-identifier-naming)
  {
    instance().average(name, value);
  }

  /** \brief Maintain the average of a specific value */
  void average(const std::string& name, const double value);

  /** \brief Begin counting time for a specific chunk of code */
  static void Begin(const std::string& name)  // NOLINT(readability-identifier-naming)
  {
    instance().begin(name);
  }

  /** \brief Stop counting time for a specific chunk of code */
  static void End(const std::string& name)  // NOLINT(readability-identifier-naming)
  {
    instance().end(name);
  }

  /** \brief Begin counting time for a specific chunk of code */
  void begin(const std::string& name);

  /** \brief Stop counting time for a specific chunk of code */
  void end(const std::string& name);

  /** \brief Print the status of the profiled code chunks and
      events. Optionally, computation done by different threads
      can be printed separately. */
  static void Status(std::ostream& out = std::cout, bool merge = true)  // NOLINT(readability-identifier-naming)
  {
    instance().status(out, merge);
  }

  /** \brief Print the status of the profiled code chunks and
      events. Optionally, computation done by different threads
      can be printed separately. */
  void status(std::ostream& out = std::cout, bool merge = true);

  /** \brief Print the status of the profiled code chunks and
      events to the console (using msg::Console) */
  static void Console()  // NOLINT(readability-identifier-naming)
  {
    instance().console();
  }

  /** \brief Print the status of the profiled code chunks and
      events to the console (using msg::Console) */
  void console();

  /** \brief Check if the profiler is counting time or not */
  bool running() const
  {
    return running_;
  }

  /** \brief Check if the profiler is counting time or not */
  static bool Running()  // NOLINT(readability-identifier-naming)
  {
    return instance().running();
  }

private:
  /** \brief Information about time spent in a section of the code */
  struct TimeInfo
  {
    TimeInfo()
      : total(0, 0, 0, 0), shortest(boost::posix_time::pos_infin), longest(boost::posix_time::neg_infin), parts(0)
    {
    }

    /** \brief Total time counted. */
    boost::posix_time::time_duration total;

    /** \brief The shortest counted time interval */
    boost::posix_time::time_duration shortest;

    /** \brief The longest counted time interval */
    boost::posix_time::time_duration longest;

    /** \brief Number of times a chunk of time was added to this structure */
    unsigned long int parts;

    /** \brief The point in time when counting time started */
    boost::posix_time::ptime start;

    /** \brief Begin counting time */
    void set()
    {
      start = boost::posix_time::microsec_clock::universal_time();
    }

    /** \brief Add the counted time to the total time */
    void update()
    {
      const boost::posix_time::time_duration& dt = boost::posix_time::microsec_clock::universal_time() - start;
      if (dt > longest)
        longest = dt;
      if (dt < shortest)
        shortest = dt;
      total = total + dt;
      ++parts;
    }
  };

  /** \brief Information maintained about averaged values */
  struct AvgInfo
  {
    /** \brief The sum of the values to average */
    double total;

    /** \brief The sub of squares of the values to average */
    double totalSqr;

    /** \brief Number of times a value was added to this structure */
    unsigned long int parts;
  };

  /** \brief Information to be maintained for each thread */
  struct PerThread
  {
    /** \brief The stored events */
    std::map<std::string, unsigned long int> events;

    /** \brief The stored averages */
    std::map<std::string, AvgInfo> avg;

    /** \brief The amount of time spent in various places */
    std::map<std::string, TimeInfo> time;
  };

  void printThreadInfo(std::ostream& out, const PerThread& data);

  boost::mutex lock_;
  std::map<boost::thread::id, PerThread> data_;
  TimeInfo tinfo_;
  bool running_;
  bool printOnDestroy_;
};
}  // namespace tools
}  // namespace moveit

#else

#include <string>
#include <iostream>

/* If profiling is disabled, provide empty implementations for the
   public functions */
namespace moveit
{
namespace tools
{
class Profiler
{
public:
  class ScopedBlock
  {
  public:
    ScopedBlock(const std::string&, Profiler& = Profiler::instance())
    {
    }

    ~ScopedBlock(void)
    {
    }
  };

  class ScopedStart
  {
  public:
    ScopedStart(Profiler& = Profiler::instance())
    {
    }

    ~ScopedStart(void)
    {
    }
  };

  static Profiler& instance(void);

  Profiler(bool = true, bool = true)
  {
  }

  ~Profiler(void)
  {
  }

  static void Start(void)
  {
  }

  static void Stop(void)
  {
  }

  static void Clear(void)
  {
  }

  void start(void)
  {
  }

  void stop(void)
  {
  }

  void clear(void)
  {
  }

  static void Event(const std::string&, const unsigned int = 1)
  {
  }

  void event(const std::string&, const unsigned int = 1)
  {
  }

  static void Average(const std::string&, const double)
  {
  }

  void average(const std::string&, const double)
  {
  }

  static void Begin(const std::string&)
  {
  }

  static void End(const std::string&)
  {
  }

  void begin(const std::string&)
  {
  }

  void end(const std::string&)
  {
  }

  static void Status(std::ostream& = std::cout, bool = true)
  {
  }

  void status(std::ostream& = std::cout, bool = true)
  {
  }

  static void Console(void)
  {
  }

  void console(void)
  {
  }

  bool running(void) const
  {
    return false;
  }

  static bool Running(void)
  {
    return false;
  }
};
}  // namespace tools
}  // namespace moveit

#endif
