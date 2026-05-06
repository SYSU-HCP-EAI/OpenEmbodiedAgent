/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Dave Coleman */

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/setup_assistant/tools/compute_default_collisions.h>
#include <boost/math/special_functions/binomial.hpp>  // for statistics at end
#include <boost/thread.hpp>
#include <boost/unordered_map.hpp>
#include <boost/assign.hpp>
#include <ros/console.h>

namespace moveit_setup_assistant
{
// ******************************************************************************************
// Custom Types, Enums and Structs
// ******************************************************************************************

// Boost mapping of reasons for disabling a link pair to strings
const boost::unordered_map<DisabledReason, std::string> REASONS_TO_STRING = boost::assign::map_list_of(NEVER, "Never")(
    DEFAULT, "Default")(ADJACENT, "Adjacent")(ALWAYS, "Always")(USER, "User")(NOT_DISABLED, "Not Disabled");

const boost::unordered_map<std::string, DisabledReason> REASONS_FROM_STRING =
    boost::assign::map_list_of("Never", NEVER)("Default", DEFAULT)("Adjacent", ADJACENT)("Always", ALWAYS)(
        "User", USER)("Not Disabled", NOT_DISABLED);

// Unique set of pairs of links in string-based form
typedef std::set<std::pair<std::string, std::string> > StringPairSet;

// Struct for passing parameters to threads, for cleaner code
struct ThreadComputation
{
  ThreadComputation(planning_scene::PlanningScene& scene, const collision_detection::CollisionRequest& req,
                    int thread_id, int num_trials, StringPairSet* links_seen_colliding, boost::mutex* lock,
                    unsigned int* progress)
    : scene_(scene)
    , req_(req)
    , thread_id_(thread_id)
    , num_trials_(num_trials)
    , links_seen_colliding_(links_seen_colliding)
    , lock_(lock)
    , progress_(progress)
  {
  }
  planning_scene::PlanningScene& scene_;
  const collision_detection::CollisionRequest& req_;
  int thread_id_;
  unsigned int num_trials_;
  StringPairSet* links_seen_colliding_;
  boost::mutex* lock_;
  unsigned int* progress_;  // only to be updated by thread 0
};

// LinkGraph defines a Link's model and a set of unique links it connects
typedef std::map<const moveit::core::LinkModel*, std::set<const moveit::core::LinkModel*> > LinkGraph;

// ******************************************************************************************
// Static Prototypes
// ******************************************************************************************

/**
 * \brief Helper function for adding two links to the disabled links data structure
 * \param linkA Name of first link
 * \param linkB Name of second link
 * \param reason Enum reason of why the link pair is disabled from C.C., if it is
 * \param link_pairs List of all unique link pairs and each pair's properties
 * \return bool Was link pair already disabled from C.C.
 */
static bool setLinkPair(const std::string& linkA, const std::string& linkB, const DisabledReason reason,
                        LinkPairMap& link_pairs);

/**
 * \brief Build the robot links connection graph and then check for links with no geomotry
 * \param link The root link to begin a breadth first search on
 * \param link_graph A representation of all bi-direcitonal joint connections between links in robot_description
 */
static void computeConnectionGraph(const moveit::core::LinkModel* link, LinkGraph& link_graph);

/**
 * \brief Recursively build the adj list of link connections
 * \param link The root link to begin a breadth first search on
 * \param link_graph A representation of all bi-direcitonal joint connections between links in robot_description
 */
static void computeConnectionGraphRec(const moveit::core::LinkModel* link, LinkGraph& link_graph);

/**
 * \brief Disable collision checking for adjacent links, or adjacent with no geometry links between them
 * \param link_graph A representation of all bi-direcitonal joint connections between links in robot_description
 * \param scene A reference to the robot in the planning scene
 * \param link_pairs List of all unique link pairs and each pair's properties
 * \return number of adjacent links found and disabled
 */
static unsigned int disableAdjacentLinks(planning_scene::PlanningScene& scene, LinkGraph& link_graph,
                                         LinkPairMap& link_pairs);

/**
 * \brief Disable all collision checks that occur when the robot is started in its default state
 * \param scene A reference to the robot in the planning scene
 * \param link_pairs List of all unique link pairs and each pair's properties
 * \param req A reference to a collision request that is already initialized
 * \return number of default collision links found and disabled
 */
static unsigned int disableDefaultCollisions(planning_scene::PlanningScene& scene, LinkPairMap& link_pairs,
                                             collision_detection::CollisionRequest& req);

/**
 * \brief Compute the links that are always in collision
 * \param scene A reference to the robot in the planning scene
 * \param link_pairs List of all unique link pairs and each pair's properties
 * \param req A reference to a collision request that is already initialized
 * \param links_seen_colliding Set of links that have at some point been seen in collision
 * \param min_collision_fraction If collisions are found between a pair of links >= this fraction, the are assumed
 * "always" in collision
 * \return number of always in collision links found and disabled
 */
static unsigned int disableAlwaysInCollision(planning_scene::PlanningScene& scene, LinkPairMap& link_pairs,
                                             collision_detection::CollisionRequest& req,
                                             StringPairSet& links_seen_colliding, double min_collision_faction = 0.95);

/**
 * \brief Get the pairs of links that are never in collision
 * \param scene A reference to the robot in the planning scene
 * \param link_pairs List of all unique link pairs and each pair's properties
 * \param req A reference to a collision request that is already initialized
 * \param links_seen_colliding Set of links that have at some point been seen in collision
 * \return number of never in collision links found and disabled
 */
static unsigned int disableNeverInCollision(const unsigned int num_trials, planning_scene::PlanningScene& scene,
                                            LinkPairMap& link_pairs, const collision_detection::CollisionRequest& req,
                                            StringPairSet& links_seen_colliding, unsigned int* progress);

/**
 * \brief Thread for getting the pairs of links that are never in collision
 * \param tc Struct that encapsulates all the data each thread needs
 */
static void disableNeverInCollisionThread(ThreadComputation tc);

// ******************************************************************************************
// Generates an adjacency list of links that are always and never in collision, to speed up collision detection
// ******************************************************************************************
LinkPairMap computeDefaultCollisions(const planning_scene::PlanningSceneConstPtr& parent_scene, unsigned int* progress,
                                     const bool include_never_colliding, const unsigned int num_trials,
                                     const double min_collision_fraction, const bool verbose)
{
  // Create new instance of planning scene using pointer
  planning_scene::PlanningScenePtr scene = parent_scene->diff();

  // Map of disabled collisions that contains a link as a key and an ordered list of links that are connected.
  LinkPairMap link_pairs;

  // Track unique edges that have been found to be in collision in some state
  StringPairSet links_seen_colliding;

  // LinkGraph is a custom type of a map with a LinkModel as key and a set of LinkModels as second
  LinkGraph link_graph;

  // ROS_INFO_STREAM("Initial allowed Collision Matrix Size = " << scene.getAllowedCollisions().getSize() );

  // 0. GENERATE ALL POSSIBLE LINK PAIRS -------------------------------------------------------------
  // Generate a list of unique link pairs for all links with geometry. Order pairs alphabetically.
  // There should be n choose 2 pairs
  computeLinkPairs(*scene, link_pairs);
  *progress = 1;

  // 1. FIND CONNECTING LINKS ------------------------------------------------------------------------
  // For each link, compute the set of other links it connects to via a single joint (adjacent links)
  // or via a chain of joints with intermediate links with no geometry (like a socket joint)

  // Create Connection Graph
  computeConnectionGraph(scene->getRobotModel()->getRootLink(), link_graph);
  *progress = 2;  // Progress bar feedback
  boost::this_thread::interruption_point();

  // 2. DISABLE ALL ADJACENT LINK COLLISIONS ---------------------------------------------------------
  // if 2 links are adjacent, or adjacent with a zero-shape between them, disable collision checking for them
  unsigned int num_adjacent = disableAdjacentLinks(*scene, link_graph, link_pairs);
  *progress = 4;  // Progress bar feedback
  boost::this_thread::interruption_point();

  // 3. INITIAL CONTACTS TO CONSIDER GUESS -----------------------------------------------------------
  // Create collision detection request object
  collision_detection::CollisionRequest req;
  req.contacts = true;
  // max number of contacts to compute. initial guess is number of links on robot
  req.max_contacts = int(link_graph.size());
  req.max_contacts_per_pair = 1;
  req.verbose = false;

  // 4. DISABLE "DEFAULT" COLLISIONS --------------------------------------------------------
  // Disable all collision checks that occur when the robot is started in its default state
  unsigned int num_default = disableDefaultCollisions(*scene, link_pairs, req);
  *progress = 6;  // Progress bar feedback
  boost::this_thread::interruption_point();

  // 5. ALWAYS IN COLLISION --------------------------------------------------------------------
  // Compute the links that are always in collision
  unsigned int num_always =
      disableAlwaysInCollision(*scene, link_pairs, req, links_seen_colliding, min_collision_fraction);
  // ROS_INFO("Links seen colliding total = %d", int(links_seen_colliding.size()));
  *progress = 8;  // Progress bar feedback
  boost::this_thread::interruption_point();

  // 6. NEVER IN COLLISION -------------------------------------------------------------------
  // Get the pairs of links that are never in collision
  unsigned int num_never = 0;
  if (include_never_colliding)  // option of function
  {
    num_never = disableNeverInCollision(num_trials, *scene, link_pairs, req, links_seen_colliding, progress);
  }

  // ROS_INFO("Link pairs seen colliding ever: %d", int(links_seen_colliding.size()));

  if (verbose)
  {
    // Calculate number of disabled links:
    unsigned int num_disabled = 0;
    for (LinkPairMap::const_iterator pair_it = link_pairs.begin(); pair_it != link_pairs.end(); ++pair_it)
    {
      if (pair_it->second.disable_check)  // has a reason to be disabled
        ++num_disabled;
    }

    ROS_INFO("-------------------------------------------------------------------------------");
    ROS_INFO("Statistics:");
    unsigned int num_links = int(link_graph.size());
    double num_possible = boost::math::binomial_coefficient<double>(num_links, 2);  // n choose 2
    unsigned int num_sometimes = num_possible - num_disabled;

    ROS_INFO("%6d : %s", num_links, "Total Links");
    ROS_INFO("%6.0f : %s", num_possible, "Total possible collisions");
    ROS_INFO("%6d : %s", num_always, "Always in collision");
    ROS_INFO("%6d : %s", num_never, "Never in collision");
    ROS_INFO("%6d : %s", num_default, "Default in collision");
    ROS_INFO("%6d : %s", num_adjacent, "Adjacent links disabled");
    ROS_INFO("%6d : %s", num_sometimes, "Sometimes in collision");
    ROS_INFO("%6d : %s", num_disabled, "TOTAL DISABLED");

    /*ROS_INFO("Copy to Spreadsheet:");
    ROS_INFO_STREAM(num_links << "\t" << num_possible << "\t" << num_always << "\t" << num_never
                    << "\t" << num_default << "\t" << num_adjacent << "\t" << num_sometimes
                    << "\t" << num_disabled);
    */
  }

  *progress = 100;  // end the status bar

  return link_pairs;
}

// ******************************************************************************************
// Helper function for adding two links to the disabled links data structure
// ******************************************************************************************
bool setLinkPair(const std::string& linkA, const std::string& linkB, const DisabledReason reason,
                 LinkPairMap& link_pairs)
{
  bool is_unique = false;  // determine if this link pair had already existsed in the link_pairs datastructure

  // Determine order of the 2 links in the pair
  std::pair<std::string, std::string> link_pair;

  // compare the string names of the two links and add the lesser alphabetically, s.t. the pair is only added once
  if (linkA < linkB)
  {
    link_pair = std::pair<std::string, std::string>(linkA, linkB);
  }
  else
  {
    link_pair = std::pair<std::string, std::string>(linkB, linkA);
  }

  // Update properties of this link pair using only 1 search
  LinkPairData* link_pair_ptr = &link_pairs[link_pair];

  // Check if link pair was already disabled. It also creates the entry if none existed
  if (!link_pairs[link_pair].disable_check)  // it was not previously disabled
  {
    is_unique = true;
    link_pair_ptr->reason = reason;  // only change the reason if the pair was previously enabled
  }

  // Only disable collision checking if there is a reason to disable it. This func is also used for initializing pairs
  link_pair_ptr->disable_check = (reason != NOT_DISABLED);

  return is_unique;
}

// ******************************************************************************************
// Generate a list of unique link pairs for all links with geometry. Order pairs alphabetically. n choose 2 pairs
// ******************************************************************************************
void computeLinkPairs(const planning_scene::PlanningScene& scene, LinkPairMap& link_pairs)
{
  // Get the names of the link models that have some collision geometry associated to themselves
  const std::vector<std::string>& names = scene.getRobotModel()->getLinkModelNamesWithCollisionGeometry();

  std::pair<std::string, std::string> temp_pair;

  // Loop through every combination of name pairs, AB and BA, n^2
  for (std::size_t i = 0; i < names.size(); ++i)
  {
    for (std::size_t j = i + 1; j < names.size(); ++j)
    {
      // Add to link pairs list
      setLinkPair(names[i], names[j], NOT_DISABLED, link_pairs);
    }
  }
}
// ******************************************************************************************
// Build the robot links connection graph and then check for links with no geomotry
// ******************************************************************************************
void computeConnectionGraph(const moveit::core::LinkModel* start_link, LinkGraph& link_graph)
{
  link_graph.clear();  // make sure the edges structure is clear

  // Recurively build adj list of link connections
  computeConnectionGraphRec(start_link, link_graph);

  // Repeatidly check for links with no geometry and remove them, then re-check until no more removals are detected
  bool update = true;  // track if a no geometry link was found
  while (update)
  {
    update = false;  // default

    // Check if each edge has a shape
    for (LinkGraph::const_iterator edge_it = link_graph.begin(); edge_it != link_graph.end(); ++edge_it)
    {
      if (edge_it->first->getShapes().empty())  // link in adjList "link_graph" does not have shape, remove!
      {
        // Temporary list for connected links
        std::vector<const moveit::core::LinkModel*> temp_list;

        // Copy link's parent and child links to temp_list
        for (const moveit::core::LinkModel* adj_it : edge_it->second)
        {
          temp_list.push_back(adj_it);
        }

        // Make all preceeding and succeeding links to the no-shape link fully connected
        // so that they don't collision check with themselves
        for (std::size_t i = 0; i < temp_list.size(); ++i)
        {
          for (std::size_t j = i + 1; j < temp_list.size(); ++j)
          {
            // for each LinkModel in temp_list, find its location in the link_graph structure and insert the rest
            // of the links into its unique set.
            // if the LinkModel is not already in the set, update is set to true so that we keep searching
            if (link_graph[temp_list[i]].insert(temp_list[j]).second)
              update = true;
            if (link_graph[temp_list[j]].insert(temp_list[i]).second)
              update = true;
          }
        }
      }
    }
  }
  // ROS_INFO("Generated connection graph with %d links", int(link_graph.size()));
}

// ******************************************************************************************
// Recursively build the adj list of link connections
// ******************************************************************************************
void computeConnectionGraphRec(const moveit::core::LinkModel* start_link, LinkGraph& link_graph)
{
  if (start_link)  // check that the link is a valid pointer
  {
    // Loop through every link attached to start_link
    for (std::size_t i = 0; i < start_link->getChildJointModels().size(); ++i)
    {
      const moveit::core::LinkModel* next = start_link->getChildJointModels()[i]->getChildLinkModel();

      // Bi-directional connection
      link_graph[next].insert(start_link);
      link_graph[start_link].insert(next);

      // Iterate with subsequent link
      computeConnectionGraphRec(next, link_graph);
    }
  }
  else
  {
    ROS_ERROR("Joint exists in URDF with no link!");
  }
}

// ******************************************************************************************
// Disable collision checking for adjacent links, or adjacent with no geometry links between them
// ******************************************************************************************
unsigned int disableAdjacentLinks(planning_scene::PlanningScene& scene, LinkGraph& link_graph, LinkPairMap& link_pairs)
{
  int num_disabled = 0;
  for (LinkGraph::const_iterator link_graph_it = link_graph.begin(); link_graph_it != link_graph.end(); ++link_graph_it)
  {
    // disable all connected links to current link by looping through them
    for (std::set<const moveit::core::LinkModel*>::const_iterator adj_it = link_graph_it->second.begin();
         adj_it != link_graph_it->second.end(); ++adj_it)
    {
      // ROS_INFO("Disabled %s to %s", link_graph_it->first->getName().c_str(), (*adj_it)->getName().c_str() );

      // Check if either of the links have no geometry. If so, do not add (are we sure?)
      if (!link_graph_it->first->getShapes().empty() && !(*adj_it)->getShapes().empty())  // both links have geometry
      {
        num_disabled += setLinkPair(link_graph_it->first->getName(), (*adj_it)->getName(), ADJACENT, link_pairs);

        // disable link checking in the collision matrix
        scene.getAllowedCollisionMatrixNonConst().setEntry(link_graph_it->first->getName(), (*adj_it)->getName(), true);
      }
    }
  }
  // ROS_INFO("Disabled %d adjancent link pairs from collision checking", num_disabled);

  return num_disabled;
}

// ******************************************************************************************
// Disable all collision checks that occur when the robot is started in its default state
// ******************************************************************************************
unsigned int disableDefaultCollisions(planning_scene::PlanningScene& scene, LinkPairMap& link_pairs,
                                      collision_detection::CollisionRequest& req)
{
  // Setup environment
  collision_detection::CollisionResult res;
  scene.getCurrentStateNonConst().setToDefaultValues();  // set to default values of 0 OR half between low and high
                                                         // joint values
  scene.checkSelfCollision(req, res);

  // For each collision in default state, always add to disabled links set
  int num_disabled = 0;
  for (collision_detection::CollisionResult::ContactMap::const_iterator it = res.contacts.begin();
       it != res.contacts.end(); ++it)
  {
    num_disabled += setLinkPair(it->first.first, it->first.second, DEFAULT, link_pairs);

    // disable link checking in the collision matrix
    scene.getAllowedCollisionMatrixNonConst().setEntry(it->first.first, it->first.second, true);
  }

  // ROS_INFO("Disabled %d link pairs that are in collision in default state from collision checking", num_disabled);

  return num_disabled;
}

// ******************************************************************************************
// Compute the links that are always in collision
// ******************************************************************************************
unsigned int disableAlwaysInCollision(planning_scene::PlanningScene& scene, LinkPairMap& link_pairs,
                                      collision_detection::CollisionRequest& req, StringPairSet& links_seen_colliding,
                                      double min_collision_faction)
{
  // Trial count variables
  static const unsigned int SMALL_TRIAL_COUNT = 200;
  static const unsigned int SMALL_TRIAL_LIMIT = (unsigned int)((double)SMALL_TRIAL_COUNT * min_collision_faction);

  bool done = false;
  unsigned int num_disabled = 0;

  while (!done)
  {
    // DO 'SMALL_TRIAL_COUNT' COLLISION CHECKS AND RECORD STATISTICS ---------------------------------------
    std::map<std::pair<std::string, std::string>, unsigned int> collision_count;

    // Do a large number of tests
    for (unsigned int i = 0; i < SMALL_TRIAL_COUNT; ++i)
    {
      // Check for collisions
      collision_detection::CollisionResult res;
      scene.getCurrentStateNonConst().setToRandomPositions();
      scene.checkSelfCollision(req, res);

      // Sum the number of collisions
      unsigned int nc = 0;
      for (collision_detection::CollisionResult::ContactMap::const_iterator it = res.contacts.begin();
           it != res.contacts.end(); ++it)
      {
        collision_count[it->first]++;
        links_seen_colliding.insert(it->first);
        nc += it->second.size();
      }

      // Check if the number of contacts is greater than the max count
      if (nc >= req.max_contacts)
      {
        req.max_contacts *= 2;  // double the max contacts that the CollisionRequest checks for
        // ROS_INFO("Doubling max_contacts to %d", int(req.max_contacts));
      }
    }

    // >= XX% OF TIME IN COLLISION DISABLE -----------------------------------------------------
    // Disable collision checking for links that are >= XX% of the time in collision (XX% = 95% by default)
    int found = 0;

    // Loop through every pair of link collisions and disable if it meets the threshold
    for (std::map<std::pair<std::string, std::string>, unsigned int>::const_iterator it = collision_count.begin();
         it != collision_count.end(); ++it)
    {
      // Disable these two links permanently
      if (it->second > SMALL_TRIAL_LIMIT)
      {
        num_disabled += setLinkPair(it->first.first, it->first.second, ALWAYS, link_pairs);

        // disable link checking in the collision matrix
        scene.getAllowedCollisionMatrixNonConst().setEntry(it->first.first, it->first.second, true);

        found++;
      }
    }

    // if no updates were made to the collision matrix, we stop
    if (found == 0)
      done = true;

    // ROS_INFO("Disabled %u link pairs that are always in collision from collision checking", found);
  }

  return num_disabled;
}

// ******************************************************************************************
// Get the pairs of links that are never in collision
// ******************************************************************************************
unsigned int disableNeverInCollision(const unsigned int num_trials, planning_scene::PlanningScene& scene,
                                     LinkPairMap& link_pairs, const collision_detection::CollisionRequest& req,
                                     StringPairSet& links_seen_colliding, unsigned int* progress)
{
  unsigned int num_disabled = 0;

  boost::thread_group bgroup;  // create a group of threads
  boost::mutex lock;           // used for sharing the same data structures

  int num_threads = boost::thread::hardware_concurrency();  // how many cores does this computer have?
  // ROS_INFO_STREAM("Performing " << num_trials << " trials for 'always in collision' checking on " <<
  //   num_threads << " threads...");

  for (int i = 0; i < num_threads; ++i)
  {
    ThreadComputation tc(scene, req, i, num_trials / num_threads, &links_seen_colliding, &lock, progress);
    bgroup.create_thread([tc] { return disableNeverInCollisionThread(tc); });
  }

  try
  {
    bgroup.join_all();  // wait for all threads to finish
  }
  catch (boost::thread_interrupted)
  {
    ROS_WARN("disableNeverInCollision interrupted");
    bgroup.interrupt_all();
    bgroup.join_all();  // wait for all threads to interrupt
    throw;
  }

  // Loop through every possible link pair and check if it has ever been seen in collision
  for (std::pair<const std::pair<std::string, std::string>, LinkPairData>& link_pair : link_pairs)
  {
    if (!link_pair.second.disable_check)  // is not disabled yet
    {
      // Check if current pair has been seen colliding ever. If it has never been seen colliding, add it to disabled
      // list
      if (links_seen_colliding.find(link_pair.first) == links_seen_colliding.end())
      {
        // Add to disabled list using pair ordering
        link_pair.second.reason = NEVER;
        link_pair.second.disable_check = true;

        // Count it
        ++num_disabled;
      }
    }
  }
  // ROS_INFO("Disabled %d link pairs that are never in collision", num_disabled);

  return num_disabled;
}

// ******************************************************************************************
// Thread for getting the pairs of links that are never in collision
// ******************************************************************************************
void disableNeverInCollisionThread(ThreadComputation tc)
{
  // ROS_INFO_STREAM("Thread " << tc.thread_id_ << " running " << tc.num_trials_ << " trials");

  // User feedback vars
  const unsigned int progress_interval = tc.num_trials_ / 20;  // show progress update every 5%

  // Create a new kinematic state for this thread to work on
  moveit::core::RobotState robot_state(tc.scene_.getRobotModel());

  // Do a large number of tests
  for (unsigned int i = 0; i < tc.num_trials_; ++i)
  {
    boost::this_thread::interruption_point();

    // Status update at intervals and only for 0 thread
    if (i % progress_interval == 0 && tc.thread_id_ == 0)
    {
      (*tc.progress_) = i * 92 / tc.num_trials_ + 8;  // 8 is the amount of progress already completed in prev steps
    }

    collision_detection::CollisionResult res;
    robot_state.setToRandomPositions();
    tc.scene_.checkSelfCollision(tc.req_, res, robot_state);

    // Check all contacts
    for (collision_detection::CollisionResult::ContactMap::const_iterator it = res.contacts.begin();
         it != res.contacts.end(); ++it)
    {
      // Check if this collision pair is unique before doing a thread lock
      if (tc.links_seen_colliding_->find(it->first) == tc.links_seen_colliding_->end())
      {
        // Collision Matrix and links_seen_colliding is modified only if needed, based on above if statement

        boost::mutex::scoped_lock slock(*tc.lock_);
        tc.links_seen_colliding_->insert(it->first);

        tc.scene_.getAllowedCollisionMatrixNonConst().setEntry(it->first.first, it->first.second,
                                                               true);  // disable link checking in the collision matrix
      }
    }
  }
}

// ******************************************************************************************
// Converts a reason for disabling a link pair into a string
// ******************************************************************************************
const std::string disabledReasonToString(DisabledReason reason)
{
  return REASONS_TO_STRING.at(reason);
}

// ******************************************************************************************
// Converts a string reason for disabling a link pair into a struct data type
// ******************************************************************************************
DisabledReason disabledReasonFromString(const std::string& reason)
{
  DisabledReason r;
  try
  {
    r = REASONS_FROM_STRING.at(reason);
  }
  catch (const std::out_of_range&)
  {
    r = USER;
  }

  return r;
}

}  // namespace moveit_setup_assistant
