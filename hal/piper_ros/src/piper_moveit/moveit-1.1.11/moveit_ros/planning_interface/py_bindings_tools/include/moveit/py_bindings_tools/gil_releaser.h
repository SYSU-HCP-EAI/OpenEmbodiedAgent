/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, RWTH Aachen University
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
 *   * Neither the name of RWTH Aachen University nor the names of its
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

/* Author: Bjarne von Horn */

#pragma once

#include <utility>
#include <Python.h>

namespace moveit
{
namespace py_bindings_tools
{
/** \brief RAII Helper to release the Global Interpreter Lock (GIL)
 *
 * Use this helper class to release the GIL before doing long computations
 * or blocking calls. Note that without the GIL Python-related functions <b>must not</b> be called.
 * So, before releasing the GIL all \c boost::python variables have to be converted to e.g. \c std::vector<std::string>.
 * Before converting the result back to e.g. a moveit::py_bindings_tools::ByteString instance, the GIL has to be
 * reacquired.
 */
class GILReleaser
{
  PyThreadState* m_thread_state;

public:
  /** \brief Release the GIL on construction  */
  GILReleaser() noexcept
  {
    m_thread_state = PyEval_SaveThread();
  }
  /** \brief Reacquire the GIL on destruction  */
  ~GILReleaser() noexcept
  {
    if (m_thread_state)
    {
      PyEval_RestoreThread(m_thread_state);
      m_thread_state = nullptr;
    }
  }

  GILReleaser(const GILReleaser&) = delete;
  GILReleaser(GILReleaser&& other) noexcept
  {
    m_thread_state = other.m_thread_state;
    other.m_thread_state = nullptr;
  }

  GILReleaser& operator=(const GILReleaser&) = delete;
  GILReleaser& operator=(GILReleaser&& other) noexcept
  {
    GILReleaser copy(std::move(other));
    this->swap(copy);
    return *this;
  }

  void swap(GILReleaser& other) noexcept
  {
    std::swap(other.m_thread_state, m_thread_state);
  }
};

}  // namespace py_bindings_tools
}  // namespace moveit
