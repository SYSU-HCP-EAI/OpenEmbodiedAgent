/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
 *  Copyright (c) 2014, SRI International
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

/* Author: Acorn Pooley, Ioan Sucan */

#include <gtest/gtest.h>
#include <moveit/robot_interaction/locked_robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <urdf_parser/urdf_parser.h>

static const char* URDF_STR = "<?xml version=\"1.0\" ?>"
                              "<robot name=\"one_robot\">"
                              "<link name=\"base_link\">"
                              "  <inertial>"
                              "    <mass value=\"2.81\"/>"
                              "    <origin rpy=\"0 0 0\" xyz=\"0.0 0.0 .0\"/>"
                              "    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
                              "  </inertial>"
                              "  <collision name=\"my_collision\">"
                              "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
                              "    <geometry>"
                              "      <box size=\"1 2 1\" />"
                              "    </geometry>"
                              "  </collision>"
                              "  <visual>"
                              "    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
                              "    <geometry>"
                              "      <box size=\"1 2 1\" />"
                              "    </geometry>"
                              "  </visual>"
                              "</link>"
                              "<joint name=\"joint_a\" type=\"continuous\">"
                              "   <axis xyz=\"0 0 1\"/>"
                              "   <parent link=\"base_link\"/>"
                              "   <child link=\"link_a\"/>"
                              "   <origin rpy=\" 0.0 0 0 \" xyz=\"0.0 0 0 \"/>"
                              "</joint>"
                              "<link name=\"link_a\">"
                              "  <inertial>"
                              "    <mass value=\"1.0\"/>"
                              "    <origin rpy=\"0 0 0\" xyz=\"0.0 0.0 .0\"/>"
                              "    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
                              "  </inertial>"
                              "  <collision>"
                              "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
                              "    <geometry>"
                              "      <box size=\"1 2 1\" />"
                              "    </geometry>"
                              "  </collision>"
                              "  <visual>"
                              "    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
                              "    <geometry>"
                              "      <box size=\"1 2 1\" />"
                              "    </geometry>"
                              "  </visual>"
                              "</link>"
                              "<joint name=\"joint_b\" type=\"fixed\">"
                              "  <parent link=\"link_a\"/>"
                              "  <child link=\"link_b\"/>"
                              "  <origin rpy=\" 0.0 -0.42 0 \" xyz=\"0.0 0.5 0 \"/>"
                              "</joint>"
                              "<link name=\"link_b\">"
                              "  <inertial>"
                              "    <mass value=\"1.0\"/>"
                              "    <origin rpy=\"0 0 0\" xyz=\"0.0 0.0 .0\"/>"
                              "    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
                              "  </inertial>"
                              "  <collision>"
                              "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
                              "    <geometry>"
                              "      <box size=\"1 2 1\" />"
                              "    </geometry>"
                              "  </collision>"
                              "  <visual>"
                              "    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
                              "    <geometry>"
                              "      <box size=\"1 2 1\" />"
                              "    </geometry>"
                              "  </visual>"
                              "</link>"
                              "  <joint name=\"joint_c\" type=\"prismatic\">"
                              "    <axis xyz=\"1 0 0\"/>"
                              "    <limit effort=\"100.0\" lower=\"0.0\" upper=\"0.09\" velocity=\"0.2\"/>"
                              "    <safety_controller k_position=\"20.0\" k_velocity=\"500.0\" "
                              "soft_lower_limit=\"0.0\" soft_upper_limit=\"0.089\"/>"
                              "    <parent link=\"link_b\"/>"
                              "    <child link=\"link_c\"/>"
                              "    <origin rpy=\" 0.0 0.42 0.0 \" xyz=\"0.0 -0.1 0 \"/>"
                              "  </joint>"
                              "<link name=\"link_c\">"
                              "  <inertial>"
                              "    <mass value=\"1.0\"/>"
                              "    <origin rpy=\"0 0 0\" xyz=\"0.0 0 .0\"/>"
                              "    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
                              "  </inertial>"
                              "  <collision>"
                              "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
                              "    <geometry>"
                              "      <box size=\"1 2 1\" />"
                              "    </geometry>"
                              "  </collision>"
                              "  <visual>"
                              "    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
                              "    <geometry>"
                              "      <box size=\"1 2 1\" />"
                              "    </geometry>"
                              "  </visual>"
                              "</link>"
                              "  <joint name=\"mim_f\" type=\"prismatic\">"
                              "    <axis xyz=\"1 0 0\"/>"
                              "    <limit effort=\"100.0\" lower=\"0.0\" upper=\"0.19\" velocity=\"0.2\"/>"
                              "    <parent link=\"link_c\"/>"
                              "    <child link=\"link_d\"/>"
                              "    <origin rpy=\" 0.0 0.1 0.0 \" xyz=\"0.1 0.1 0 \"/>"
                              "    <mimic joint=\"joint_f\" multiplier=\"1.5\" offset=\"0.1\"/>"
                              "  </joint>"
                              "  <joint name=\"joint_f\" type=\"prismatic\">"
                              "    <axis xyz=\"1 0 0\"/>"
                              "    <limit effort=\"100.0\" lower=\"0.0\" upper=\"0.19\" velocity=\"0.2\"/>"
                              "    <parent link=\"link_d\"/>"
                              "    <child link=\"link_e\"/>"
                              "    <origin rpy=\" 0.0 0.1 0.0 \" xyz=\"0.1 0.1 0 \"/>"
                              "  </joint>"
                              "<link name=\"link_d\">"
                              "  <collision>"
                              "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
                              "    <geometry>"
                              "      <box size=\"1 2 1\" />"
                              "    </geometry>"
                              "  </collision>"
                              "  <visual>"
                              "    <origin rpy=\"0 1 0\" xyz=\"0 0.1 0\"/>"
                              "    <geometry>"
                              "      <box size=\"1 2 1\" />"
                              "    </geometry>"
                              "  </visual>"
                              "</link>"
                              "<link name=\"link_e\">"
                              "  <collision>"
                              "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
                              "    <geometry>"
                              "      <box size=\"1 2 1\" />"
                              "    </geometry>"
                              "  </collision>"
                              "  <visual>"
                              "    <origin rpy=\"0 1 0\" xyz=\"0 0.1 0\"/>"
                              "    <geometry>"
                              "      <box size=\"1 2 1\" />"
                              "    </geometry>"
                              "  </visual>"
                              "</link>"
                              "</robot>";

static const char* SRDF_STR =
    "<?xml version=\"1.0\" ?>"
    "<robot name=\"one_robot\">"
    "<virtual_joint name=\"base_joint\" child_link=\"base_link\" parent_frame=\"odom_combined\" type=\"planar\"/>"
    "<group name=\"base_from_joints\">"
    "<joint name=\"base_joint\"/>"
    "<joint name=\"joint_a\"/>"
    "<joint name=\"joint_c\"/>"
    "</group>"
    "<group name=\"mim_joints\">"
    "<joint name=\"joint_f\"/>"
    "<joint name=\"mim_f\"/>"
    "</group>"
    "<group name=\"base_with_subgroups\">"
    "<group name=\"base_from_base_to_tip\"/>"
    "<joint name=\"joint_c\"/>"
    "</group>"
    "<group name=\"base_from_base_to_tip\">"
    "<chain base_link=\"base_link\" tip_link=\"link_b\"/>"
    "<joint name=\"base_joint\"/>"
    "</group>"
    "</robot>";

// index of joints from URDF
enum
{
  JOINT_A = 3,
  JOINT_C = 4,
  MIM_F = 5,
  JOINT_F = 6
};

static moveit::core::RobotModelPtr getModel()
{
  static moveit::core::RobotModelPtr model;
  if (!model)
  {
    urdf::ModelInterfaceSharedPtr urdf(urdf::parseURDF(URDF_STR));
    srdf::ModelSharedPtr srdf(new srdf::Model());
    srdf->initString(*urdf, SRDF_STR);
    model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  }
  return model;
}

// Test constructors and robot model loading
TEST(LockedRobotState, load)
{
  moveit::core::RobotModelPtr model = getModel();

  robot_interaction::LockedRobotState ls1(model);

  moveit::core::RobotState state2(model);
  state2.setToDefaultValues();
  robot_interaction::LockedRobotState ls2(state2);

  robot_interaction::LockedRobotStatePtr ls4(new robot_interaction::LockedRobotState(model));
}

// sanity test the URDF and enum
TEST(LockedRobotState, URDF_sanity)
{
  moveit::core::RobotModelPtr model = getModel();
  robot_interaction::LockedRobotState ls1(model);

  EXPECT_EQ(ls1.getState()->getVariableNames()[JOINT_A], "joint_a");
}

// A superclass to test the robotStateChanged() virtual method
class Super1 : public robot_interaction::LockedRobotState
{
public:
  Super1(const moveit::core::RobotModelPtr& model) : LockedRobotState(model), cnt_(0)
  {
  }

  void robotStateChanged() override
  {
    cnt_++;
  }

  int cnt_;
};

static void modify1(moveit::core::RobotState* state)
{
  state->setVariablePosition(JOINT_A, 0.00006);
}

TEST(LockedRobotState, robotStateChanged)
{
  moveit::core::RobotModelPtr model = getModel();

  Super1 ls1(model);

  EXPECT_EQ(ls1.cnt_, 0);

  moveit::core::RobotState cp1(*ls1.getState());
  cp1.setVariablePosition(JOINT_A, 0.00001);
  cp1.setVariablePosition(JOINT_C, 0.00002);
  cp1.setVariablePosition(JOINT_F, 0.00003);
  ls1.setState(cp1);

  EXPECT_EQ(ls1.cnt_, 1);

  ls1.modifyState(modify1);
  EXPECT_EQ(ls1.cnt_, 2);

  ls1.modifyState(modify1);
  EXPECT_EQ(ls1.cnt_, 3);
}

// Class for testing LockedRobotState in multithreaded environment.
// Contains thread functions for modifying/checking a LockedRobotState.
class MyInfo
{
public:
  MyInfo() : quit_(false)
  {
  }

  // Thread that repeatedly sets state to different values
  void setThreadFunc(robot_interaction::LockedRobotState* locked_state, int* counter, double offset);

  // Thread that repeatedly modifies  state with different values
  void modifyThreadFunc(robot_interaction::LockedRobotState* locked_state, int* counter, double offset);

  // Thread that repeatedly checks that state is valid (not half-updated)
  void checkThreadFunc(robot_interaction::LockedRobotState* locked_state, int* counter);

  // Thread that waits for other threads to complete
  void waitThreadFunc(robot_interaction::LockedRobotState* locked_state, int** counters, int max);

private:
  // helper function for modifyThreadFunc
  void modifyFunc(moveit::core::RobotState& state, double val);

  // Checks state for validity and self-consistancy.
  void checkState(robot_interaction::LockedRobotState& locked_state);

  // mutex protects quit_ and counter variables
  boost::mutex cnt_lock_;

  // set to true by waitThreadFunc() when all counters have reached at least
  // max.
  bool quit_;
};

// Check the state.  It should always be valid.
void MyInfo::checkState(robot_interaction::LockedRobotState& locked_state)
{
  moveit::core::RobotStateConstPtr s = locked_state.getState();

  moveit::core::RobotState cp1(*s);

  // take some time
  cnt_lock_.lock();
  cnt_lock_.unlock();
  cnt_lock_.lock();
  cnt_lock_.unlock();
  cnt_lock_.lock();
  cnt_lock_.unlock();

  // check mim_f == joint_f
  EXPECT_EQ(s->getVariablePositions()[MIM_F], s->getVariablePositions()[JOINT_F] * 1.5 + 0.1);

  moveit::core::RobotState cp2(*s);

  EXPECT_NE(cp1.getVariablePositions(), cp2.getVariablePositions());
  EXPECT_NE(cp1.getVariablePositions(), s->getVariablePositions());

  int cnt = cp1.getVariableCount();
  for (int i = 0; i < cnt; ++i)
  {
    EXPECT_EQ(cp1.getVariablePositions()[i], cp2.getVariablePositions()[i]);
    EXPECT_EQ(cp1.getVariablePositions()[i], s->getVariablePositions()[i]);
  }

  // check mim_f == joint_f
  EXPECT_EQ(s->getVariablePositions()[MIM_F], s->getVariablePositions()[JOINT_F] * 1.5 + 0.1);
}

// spin, checking the state
void MyInfo::checkThreadFunc(robot_interaction::LockedRobotState* locked_state, int* counter)
{
  bool go = true;
  while (go)
  {
    for (int loops = 0; loops < 100; ++loops)
    {
      checkState(*locked_state);
    }

    cnt_lock_.lock();
    go = !quit_;
    ++*counter;
    cnt_lock_.unlock();
  }
}

// spin, setting the state to different values
void MyInfo::setThreadFunc(robot_interaction::LockedRobotState* locked_state, int* counter, double offset)
{
  bool go = true;
  while (go)
  {
    double val = offset;
    for (int loops = 0; loops < 100; ++loops)
    {
      val += 0.0001;
      moveit::core::RobotState cp1(*locked_state->getState());

      cp1.setVariablePosition(JOINT_A, val + 0.00001);
      cp1.setVariablePosition(JOINT_C, val + 0.00002);
      cp1.setVariablePosition(JOINT_F, val + 0.00003);

      locked_state->setState(cp1);
    }

    cnt_lock_.lock();
    go = !quit_;
    ++*counter;
    cnt_lock_.unlock();

    checkState(*locked_state);

    val += 0.000001;
  }
}

// modify the state in place.  Used by MyInfo::modifyThreadFunc()
void MyInfo::modifyFunc(moveit::core::RobotState& state, double val)
{
  state.setVariablePosition(JOINT_A, val + 0.00001);
  state.setVariablePosition(JOINT_C, val + 0.00002);
  state.setVariablePosition(JOINT_F, val + 0.00003);
}

// spin, modifying the state to different values
void MyInfo::modifyThreadFunc(robot_interaction::LockedRobotState* locked_state, int* counter, double offset)
{
  bool go = true;
  while (go)
  {
    double val = offset;
    for (int loops = 0; loops < 100; ++loops)
    {
      val += 0.0001;

      locked_state->modifyState([this, val](moveit::core::RobotState* state) { modifyFunc(*state, val); });
    }

    cnt_lock_.lock();
    go = !quit_;
    ++*counter;
    cnt_lock_.unlock();

    checkState(*locked_state);

    val += 0.000001;
  }
}

// spin until all counters reach at least max
void MyInfo::waitThreadFunc(robot_interaction::LockedRobotState* locked_state, int** counters, int max)
{
  bool go = true;
  while (go)
  {
    go = false;
    cnt_lock_.lock();
    for (int i = 0; counters[i]; ++i)
    {
      if (counters[i][0] < max)
        go = true;
    }
    cnt_lock_.unlock();

    checkState(*locked_state);
    checkState(*locked_state);
  }
  cnt_lock_.lock();
  quit_ = true;
  cnt_lock_.unlock();
}

// Run several threads and ensure they modify the state consistantly
//   ncheck - # of checkThreadFunc threads to run
//   nset   - # of setThreadFunc threads to run
//   nmod   - # of modifyThreadFunc threads to run
static void runThreads(int ncheck, int nset, int nmod)
{
  MyInfo info;

  moveit::core::RobotModelPtr model = getModel();
  robot_interaction::LockedRobotState ls1(model);

  int num = ncheck + nset + nmod;

  typedef int* int_ptr;
  typedef boost::thread* thread_ptr;
  int* cnt = new int[num];
  int_ptr* counters = new int_ptr[num + 1];
  thread_ptr* threads = new thread_ptr[num];

  int p = 0;
  double val = 0.1;

  // These threads check the validity of the RobotState
  for (int i = 0; i < ncheck; ++i)
  {
    cnt[p] = 0;
    counters[p] = &cnt[p];
    threads[p] = new boost::thread(&MyInfo::checkThreadFunc, &info, &ls1, &cnt[p]);
    val += 0.1;
    p++;
  }

  // These threads set the RobotState to new values
  for (int i = 0; i < nset; ++i)
  {
    cnt[p] = 0;
    counters[p] = &cnt[p];
    threads[p] = new boost::thread(&MyInfo::setThreadFunc, &info, &ls1, &cnt[p], val);
    val += 0.1;
    p++;
  }

  // These threads modify the RobotState in place
  for (int i = 0; i < nmod; ++i)
  {
    cnt[p] = 0;
    counters[p] = &cnt[p];
    threads[p] = new boost::thread(&MyInfo::modifyThreadFunc, &info, &ls1, &cnt[p], val);
    val += 0.1;
    p++;
  }

  ASSERT_EQ(p, num);
  counters[p] = nullptr;

  // this thread waits for all the other threads to make progress, then stops
  // everything.
  boost::thread wthread(&MyInfo::waitThreadFunc, &info, &ls1, counters, 1000);

  // wait for all threads to finish
  for (int i = 0; i < p; ++i)
  {
    threads[i]->join();
    wthread.join();
  }

  // clean up
  for (int i = 0; i < p; ++i)
  {
    delete threads[i];
  }
  delete[] cnt;
  delete[] counters;
  delete[] threads;
}

TEST(LockedRobotState, set1)
{
  runThreads(1, 1, 0);
}

// skip all more complex locking checks when optimizations are disabled
// they can easily outrun 20 minutes with Debug builds
#ifdef NDEBUG
#define OPT_TEST(F, N) TEST(F, N)
#else
#define OPT_TEST(F, N) TEST(F, DISABLED_##N)
#endif

OPT_TEST(LockedRobotState, set2)
{
  runThreads(1, 2, 0);
}

OPT_TEST(LockedRobotState, set3)
{
  runThreads(1, 3, 0);
}

OPT_TEST(LockedRobotState, mod1)
{
  runThreads(1, 0, 1);
}

OPT_TEST(LockedRobotState, mod2)
{
  runThreads(1, 0, 1);
}

OPT_TEST(LockedRobotState, mod3)
{
  runThreads(1, 0, 1);
}

OPT_TEST(LockedRobotState, set1mod1)
{
  runThreads(1, 1, 1);
}

OPT_TEST(LockedRobotState, set2mod1)
{
  runThreads(1, 2, 1);
}

OPT_TEST(LockedRobotState, set1mod2)
{
  runThreads(1, 1, 2);
}

OPT_TEST(LockedRobotState, set3mod1)
{
  runThreads(1, 3, 1);
}

OPT_TEST(LockedRobotState, set1mod3)
{
  runThreads(1, 1, 3);
}

OPT_TEST(LockedRobotState, set3mod3)
{
  runThreads(1, 3, 3);
}

OPT_TEST(LockedRobotState, set3mod3c3)
{
  runThreads(3, 3, 3);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
