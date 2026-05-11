/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Maxim Likhachev
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
 *   * Neither the name of Maxim Likhachev nor the names of its
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

#pragma once

class DummyEnvironment : public DiscreteSpaceInformation
{
public:
  /** \brief initialization environment from file (see .cfg files for examples)
   */
  virtual bool InitializeEnv(const char* sEnvFile){};

  /** \brief initialization of MDP data structure
   */
  virtual bool InitializeMDPCfg(MDPConfig* MDPCfg){};

  /** \brief heuristic estimate from state FromStateID to state ToStateID
   */
  virtual int GetFromToHeuristic(int FromStateID, int ToStateID){};
  /** \brief heuristic estimate from state with stateID to goal state
   */
  virtual int GetGoalHeuristic(int stateID){};
  /** \brief heuristic estimate from start state to state with stateID
   */
  virtual int GetStartHeuristic(int stateID){};

  /** \brief depending on the search used, it may call GetSuccs function (for forward search) or GetPreds function (for
     backward search)
      or both (for incremental search). At least one of this functions should be implemented (otherwise, there will be
     no search to run)
      Some searches may also use SetAllActionsandAllOutcomes or SetAllPreds functions if they keep the pointers to
     successors (predecessors) but
      most searches do not require this, so it is not necessary to support this
  */
  virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV){};
  /** \brief see comments for GetSuccs functon
   */
  virtual void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV){};
  /** \brief see comments for GetSuccs functon
   */
  virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state){};
  /** \brief see comments for GetSuccs functon
   */
  virtual void SetAllPreds(CMDPSTATE* state){};

  /** \brief returns the number of states (hashentries) created
   */
  virtual int SizeofCreatedEnv(){};
  /** \brief prints the state variables for a state with stateID
   */
  virtual void PrintState(int stateID, bool bVerbose, FILE* fOut = NULL){};
  /** \brief prints environment config file
   */
  virtual void PrintEnv_Config(FILE* fOut){};

  /** \brief sets a parameter to a value. The set of supported parameters depends on the particular environment
   */

  /** \brief destructor
   */
  virtual ~DummyEnvironment()
  {
  }

  /** \brief constructor
   */
  DummyEnvironment()
  {
  }
};
