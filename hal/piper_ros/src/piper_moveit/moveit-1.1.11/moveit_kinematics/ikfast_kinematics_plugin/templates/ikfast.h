// -*- coding: utf-8 -*-
// Copyright (C) 2012 Rosen Diankov <rosen.diankov@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/** \brief  Header file for all ikfast c++ files/shared objects.

    The ikfast inverse kinematics compiler is part of OpenRAVE.

    The file is divided into two sections:
    - <b>Common</b> - the abstract classes section that all ikfast share regardless of their settings
    - <b>Library Specific</b> - the library-specific definitions, which depends on the precision/settings that the
   library was compiled with

    The defines are as follows, they are also used for the ikfast C++ class:

   - IKFAST_HEADER_COMMON - common classes
   - IKFAST_HAS_LIBRARY - if defined, will include library-specific functions. by default this is off
   - IKFAST_CLIBRARY - Define this linking statically or dynamically to get correct visibility.
   - IKFAST_NO_MAIN - Remove the ``main`` function, usually used with IKFAST_CLIBRARY
   - IKFAST_ASSERT - Define in order to get a custom assert called when NaNs, divides by zero, and other invalid
   conditions are detected.
   - IKFAST_REAL - Use to force a custom real number type for IkReal.
   - IKFAST_NAMESPACE - Enclose all functions and classes in this namespace, the ``main`` function is excluded.

 */
#include <vector>
#include <list>
#include <stdexcept>

#ifndef IKFAST_HEADER_COMMON
#define IKFAST_HEADER_COMMON

/// should be the same as ikfast.__version__
#define IKFAST_VERSION 61

namespace ikfast
{
/// \brief holds the solution for a single dof
template <typename T>
class IkSingleDOFSolutionBase
{
public:
  IkSingleDOFSolutionBase() : fmul(0), foffset(0), freeind(-1), maxsolutions(1)
  {
    indices[0] = indices[1] = indices[2] = indices[3] = indices[4] = -1;
  }
  T fmul, foffset;             ///< joint value is fmul*sol[freeind]+foffset
  signed char freeind;         ///< if >= 0, mimics another joint
  unsigned char jointtype;     ///< joint type, 0x01 is revolute, 0x11 is slider
  unsigned char maxsolutions;  ///< max possible indices, 0 if controlled by free index or a free joint itself
  unsigned char indices[5];  ///< unique index of the solution used to keep track on what part it came from. sometimes a
  /// solution can be repeated for different indices. store at least another repeated root
};

/// \brief The discrete solutions are returned in this structure.
///
/// Sometimes the joint axes of the robot can align allowing an infinite number of solutions.
/// Stores all these solutions in the form of free variables that the user has to set when querying the solution. Its
/// prototype is:
template <typename T>
class IkSolutionBase
{
public:
  virtual ~IkSolutionBase()
  {
  }
  /// \brief gets a concrete solution
  ///
  /// \param[out] solution the result
  /// \param[in] freevalues values for the free parameters \se GetFree
  virtual void GetSolution(T* solution, const T* freevalues) const = 0;

  /// \brief std::vector version of \ref GetSolution
  virtual void GetSolution(std::vector<T>& solution, const std::vector<T>& freevalues) const
  {
    solution.resize(GetDOF());
    GetSolution(&solution.at(0), freevalues.size() > 0 ? &freevalues.at(0) : NULL);
  }

  /// \brief Gets the indices of the configuration space that have to be preset before a full solution can be returned
  ///
  /// \return vector of indices indicating the free parameters
  virtual const std::vector<int>& GetFree() const = 0;

  /// \brief the dof of the solution
  virtual int GetDOF() const = 0;
};

/// \brief manages all the solutions
template <typename T>
class IkSolutionListBase
{
public:
  virtual ~IkSolutionListBase()
  {
  }

  /// \brief add one solution and return its index for later retrieval
  ///
  /// \param vinfos Solution data for each degree of freedom of the manipulator
  /// \param vfree If the solution represents an infinite space, holds free parameters of the solution that users can
  /// freely set.
  virtual size_t AddSolution(const std::vector<IkSingleDOFSolutionBase<T> >& vinfos, const std::vector<int>& vfree) = 0;

  /// \brief returns the solution pointer
  virtual const IkSolutionBase<T>& GetSolution(size_t index) const = 0;

  /// \brief returns the number of solutions stored
  virtual size_t GetNumSolutions() const = 0;

  /// \brief clears all current solutions, note that any memory addresses returned from \ref GetSolution will be
  /// invalidated.
  virtual void Clear() = 0;
};

/// \brief holds function pointers for all the exported functions of ikfast
template <typename T>
class IkFastFunctions
{
public:
  IkFastFunctions()
    : _ComputeIk(NULL)
    , _ComputeFk(NULL)
    , _GetNumFreeParameters(NULL)
    , _GetFreeParameters(NULL)
    , _GetNumJoints(NULL)
    , _GetIkRealSize(NULL)
    , _GetIkFastVersion(NULL)
    , _GetIkType(NULL)
    , _GetKinematicsHash(NULL)
  {
  }
  virtual ~IkFastFunctions()
  {
  }
  typedef bool (*ComputeIkFn)(const T*, const T*, const T*, IkSolutionListBase<T>&);
  ComputeIkFn _ComputeIk;
  typedef void (*ComputeFkFn)(const T*, T*, T*);
  ComputeFkFn _ComputeFk;
  typedef int (*GetNumFreeParametersFn)();
  GetNumFreeParametersFn _GetNumFreeParameters;
  typedef int* (*GetFreeParametersFn)();
  GetFreeParametersFn _GetFreeParameters;
  typedef int (*GetNumJointsFn)();
  GetNumJointsFn _GetNumJoints;
  typedef int (*GetIkRealSizeFn)();
  GetIkRealSizeFn _GetIkRealSize;
  typedef const char* (*GetIkFastVersionFn)();
  GetIkFastVersionFn _GetIkFastVersion;
  typedef int (*GetIkTypeFn)();
  GetIkTypeFn _GetIkType;
  typedef const char* (*GetKinematicsHashFn)();
  GetKinematicsHashFn _GetKinematicsHash;
};

// Implementations of the abstract classes, user doesn't need to use them

/// \brief Default implementation of \ref IkSolutionBase
template <typename T>
class IkSolution : public IkSolutionBase<T>
{
public:
  IkSolution(const std::vector<IkSingleDOFSolutionBase<T> >& vinfos, const std::vector<int>& vfree)
  {
    _vbasesol = vinfos;
    _vfree = vfree;
  }

  virtual void GetSolution(T* solution, const T* freevalues) const
  {
    for (std::size_t i = 0; i < _vbasesol.size(); ++i)
    {
      if (_vbasesol[i].freeind < 0)
        solution[i] = _vbasesol[i].foffset;
      else
      {
        solution[i] = freevalues[_vbasesol[i].freeind] * _vbasesol[i].fmul + _vbasesol[i].foffset;
        if (solution[i] > T(3.14159265358979))
        {
          solution[i] -= T(6.28318530717959);
        }
        else if (solution[i] < T(-3.14159265358979))
        {
          solution[i] += T(6.28318530717959);
        }
      }
    }
  }

  virtual void GetSolution(std::vector<T>& solution, const std::vector<T>& freevalues) const
  {
    solution.resize(GetDOF());
    GetSolution(&solution.at(0), freevalues.size() > 0 ? &freevalues.at(0) : NULL);
  }

  virtual const std::vector<int>& GetFree() const
  {
    return _vfree;
  }
  virtual int GetDOF() const
  {
    return static_cast<int>(_vbasesol.size());
  }

  virtual void Validate() const
  {
    for (size_t i = 0; i < _vbasesol.size(); ++i)
    {
      if (_vbasesol[i].maxsolutions == (unsigned char)-1)
      {
        throw std::runtime_error("max solutions for joint not initialized");
      }
      if (_vbasesol[i].maxsolutions > 0)
      {
        if (_vbasesol[i].indices[0] >= _vbasesol[i].maxsolutions)
        {
          throw std::runtime_error("index >= max solutions for joint");
        }
        if (_vbasesol[i].indices[1] != (unsigned char)-1 && _vbasesol[i].indices[1] >= _vbasesol[i].maxsolutions)
        {
          throw std::runtime_error("2nd index >= max solutions for joint");
        }
      }
    }
  }

  virtual void GetSolutionIndices(std::vector<unsigned int>& v) const
  {
    v.resize(0);
    v.push_back(0);
    for (int i = (int)_vbasesol.size() - 1; i >= 0; --i)
    {
      if (_vbasesol[i].maxsolutions != (unsigned char)-1 && _vbasesol[i].maxsolutions > 1)
      {
        for (size_t j = 0; j < v.size(); ++j)
        {
          v[j] *= _vbasesol[i].maxsolutions;
        }
        size_t orgsize = v.size();
        if (_vbasesol[i].indices[1] != (unsigned char)-1)
        {
          for (size_t j = 0; j < orgsize; ++j)
          {
            v.push_back(v[j] + _vbasesol[i].indices[1]);
          }
        }
        if (_vbasesol[i].indices[0] != (unsigned char)-1)
        {
          for (size_t j = 0; j < orgsize; ++j)
          {
            v[j] += _vbasesol[i].indices[0];
          }
        }
      }
    }
  }

  std::vector<IkSingleDOFSolutionBase<T> > _vbasesol;  ///< solution and their offsets if joints are mimiced
  std::vector<int> _vfree;
};

/// \brief Default implementation of \ref IkSolutionListBase
template <typename T>
class IkSolutionList : public IkSolutionListBase<T>
{
public:
  virtual size_t AddSolution(const std::vector<IkSingleDOFSolutionBase<T> >& vinfos, const std::vector<int>& vfree)
  {
    size_t index = _listsolutions.size();
    _listsolutions.push_back(IkSolution<T>(vinfos, vfree));
    return index;
  }

  virtual const IkSolutionBase<T>& GetSolution(size_t index) const
  {
    if (index >= _listsolutions.size())
    {
      throw std::runtime_error("GetSolution index is invalid");
    }
    typename std::list<IkSolution<T> >::const_iterator it = _listsolutions.begin();
    std::advance(it, index);
    return *it;
  }

  virtual size_t GetNumSolutions() const
  {
    return _listsolutions.size();
  }

  virtual void Clear()
  {
    _listsolutions.clear();
  }

protected:
  std::list<IkSolution<T> > _listsolutions;
};
}  // namespace ikfast

#endif  // OPENRAVE_IKFAST_HEADER

// The following code is dependent on the C++ library linking with.
#ifdef IKFAST_HAS_LIBRARY

// defined when creating a shared object/dll
#ifdef IKFAST_CLIBRARY
#ifdef _MSC_VER
#define IKFAST_API extern "C" __declspec(dllexport)
#else
#define IKFAST_API extern "C"
#endif
#else
#define IKFAST_API
#endif

#ifdef IKFAST_NAMESPACE
namespace IKFAST_NAMESPACE
{
#endif

#ifdef IKFAST_REAL
typedef IKFAST_REAL IkReal;
#else
typedef double IkReal;
#endif

/** \brief Computes all IK solutions given a end effector coordinates and the free joints.

   - ``eetrans`` - 3 translation values. For iktype **TranslationXYOrientation3D**, the z-axis is the orientation.
   - ``eerot``
   - For **Transform6D** it is 9 values for the 3x3 rotation matrix.
   - For **Direction3D**, **Ray4D**, and **TranslationDirection5D**, the first 3 values represent the target direction.
   - For **TranslationXAxisAngle4D**, **TranslationYAxisAngle4D**, and **TranslationZAxisAngle4D** the first value
   represents the angle.
   - For **TranslationLocalGlobal6D**, the diagonal elements ([0],[4],[8]) are the local translation inside the end
   effector coordinate system.
 */
IKFAST_API bool ComputeIk(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree,
                          ikfast::IkSolutionListBase<IkReal>& solutions);

/// \brief Computes the end effector coordinates given the joint values. This function is used to double check ik.
IKFAST_API void ComputeFk(const IkReal* joints, IkReal* eetrans, IkReal* eerot);

/// \brief returns the number of free parameters users has to set apriori
IKFAST_API int GetNumFreeParameters();

/// \brief the indices of the free parameters indexed by the chain joints
IKFAST_API int* GetFreeParameters();

/// \brief the total number of indices of the chain
IKFAST_API int GetNumJoints();

/// \brief the size in bytes of the configured number type
IKFAST_API int GetIkRealSize();

/// \brief the ikfast version used to generate this file
IKFAST_API const char* GetIkFastVersion();

/// \brief the ik type ID
IKFAST_API int GetIkType();

/// \brief a hash of all the chain values used for double checking that the correct IK is used.
IKFAST_API const char* GetKinematicsHash();

#ifdef IKFAST_NAMESPACE
}
#endif

#endif  // IKFAST_HAS_LIBRARY
