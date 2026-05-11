# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Ioan Sucan

import rospy
from moveit_commander import (
    RobotCommander,
    MoveGroupCommander,
    PlanningSceneInterface,
    MoveItCommanderException,
)
from geometry_msgs.msg import Pose, PoseStamped
import tf
import re
import time
import os.path


class MoveGroupInfoLevel:
    FAIL = 1
    WARN = 2
    SUCCESS = 3
    INFO = 4
    DEBUG = 5


class MoveGroupCommandInterpreter(object):
    """
    Interpreter for simple commands
    """

    DEFAULT_FILENAME = "move_group.cfg"
    GO_DIRS = {
        "up": (2, 1),
        "down": (2, -1),
        "z": (2, 1),
        "left": (1, 1),
        "right": (1, -1),
        "y": (1, 1),
        "forward": (0, 1),
        "backward": (0, -1),
        "back": (0, -1),
        "x": (0, 1),
    }

    def __init__(self):
        self._gdict = {}
        self._group_name = ""
        self._prev_group_name = ""
        self._planning_scene_interface = PlanningSceneInterface()
        self._robot = RobotCommander()
        self._last_plan = None
        self._db_host = None
        self._db_port = 33829
        self._trace = False

    def get_active_group(self):
        if len(self._group_name) > 0:
            return self._gdict[self._group_name]
        else:
            return None

    def get_loaded_groups(self):
        return self._gdict.keys()

    def execute(self, cmd):
        cmd = self.resolve_command_alias(cmd)
        result = self.execute_generic_command(cmd)
        if result != None:
            return result
        else:
            if len(self._group_name) > 0:
                return self.execute_group_command(self._gdict[self._group_name], cmd)
            else:
                return (
                    MoveGroupInfoLevel.INFO,
                    self.get_help()
                    + "\n\nNo groups initialized yet. You must call the 'use' or the 'load' command first.\n",
                )

    def execute_generic_command(self, cmd):
        if os.path.isfile("cmd/" + cmd):
            cmd = "load cmd/" + cmd
        cmdlow = cmd.lower()
        if cmdlow.startswith("use"):
            if cmdlow == "use":
                return (MoveGroupInfoLevel.INFO, "\n".join(self.get_loaded_groups()))
            clist = cmd.split()
            clist[0] = clist[0].lower()
            if len(clist) == 2:
                if clist[0] == "use":
                    if clist[1] == "previous":
                        clist[1] = self._prev_group_name
                        if len(clist[1]) == 0:
                            return (MoveGroupInfoLevel.DEBUG, "OK")
                    if clist[1] in self._gdict:
                        self._prev_group_name = self._group_name
                        self._group_name = clist[1]
                        return (MoveGroupInfoLevel.DEBUG, "OK")
                    else:
                        try:
                            mg = MoveGroupCommander(clist[1])
                            self._gdict[clist[1]] = mg
                            self._group_name = clist[1]
                            return (MoveGroupInfoLevel.DEBUG, "OK")
                        except MoveItCommanderException as e:
                            return (
                                MoveGroupInfoLevel.FAIL,
                                "Initializing " + clist[1] + ": " + str(e),
                            )
                        except:
                            return (
                                MoveGroupInfoLevel.FAIL,
                                "Unable to initialize " + clist[1],
                            )
        elif cmdlow.startswith("trace"):
            if cmdlow == "trace":
                return (
                    MoveGroupInfoLevel.INFO,
                    "trace is on" if self._trace else "trace is off",
                )
            clist = cmdlow.split()
            if clist[0] == "trace" and clist[1] == "on":
                self._trace = True
                return (MoveGroupInfoLevel.DEBUG, "OK")
            if clist[0] == "trace" and clist[1] == "off":
                self._trace = False
                return (MoveGroupInfoLevel.DEBUG, "OK")
        elif cmdlow.startswith("load"):
            filename = self.DEFAULT_FILENAME
            clist = cmd.split()
            if len(clist) > 2:
                return (MoveGroupInfoLevel.WARN, "Unable to parse load command")
            if len(clist) == 2:
                filename = clist[1]
                if not os.path.isfile(filename):
                    filename = "cmd/" + filename
            line_num = 0
            line_content = ""
            try:
                f = open(filename, "r")
                for line in f:
                    line_num += 1
                    line = line.rstrip()
                    line_content = line
                    if self._trace:
                        print("%s:%d:  %s" % (filename, line_num, line_content))
                    comment = line.find("#")
                    if comment != -1:
                        line = line[0:comment].rstrip()
                    if line != "":
                        self.execute(line.rstrip())
                    line_content = ""
                return (MoveGroupInfoLevel.DEBUG, "OK")
            except MoveItCommanderException as e:
                if line_num > 0:
                    return (
                        MoveGroupInfoLevel.WARN,
                        "Error at %s:%d:  %s\n%s"
                        % (filename, line_num, line_content, str(e)),
                    )
                else:
                    return (
                        MoveGroupInfoLevel.WARN,
                        "Processing " + filename + ": " + str(e),
                    )
            except:
                if line_num > 0:
                    return (
                        MoveGroupInfoLevel.WARN,
                        "Error at %s:%d:  %s" % (filename, line_num, line_content),
                    )
                else:
                    return (MoveGroupInfoLevel.WARN, "Unable to load " + filename)
        elif cmdlow.startswith("save"):
            filename = self.DEFAULT_FILENAME
            clist = cmd.split()
            if len(clist) > 2:
                return (MoveGroupInfoLevel.WARN, "Unable to parse save command")
            if len(clist) == 2:
                filename = clist[1]
            try:
                f = open(filename, "w")
                for gr in self._gdict.keys():
                    f.write("use " + gr + "\n")
                    known = self._gdict[gr].get_remembered_joint_values()
                    for v in known.keys():
                        f.write(
                            v + " = [" + " ".join([str(x) for x in known[v]]) + "]\n"
                        )
                if self._db_host != None:
                    f.write(
                        "database " + self._db_host + " " + str(self._db_port) + "\n"
                    )
                return (MoveGroupInfoLevel.DEBUG, "OK")
            except:
                return (MoveGroupInfoLevel.WARN, "Unable to save " + filename)
        else:
            return None

    def execute_group_command(self, g, cmdorig):
        cmd = cmdorig.lower()

        if cmd == "stop":
            g.stop()
            return (MoveGroupInfoLevel.DEBUG, "OK")

        if cmd == "id":
            return (MoveGroupInfoLevel.INFO, g.get_name())

        if cmd == "help":
            return (MoveGroupInfoLevel.INFO, self.get_help())

        if cmd == "vars":
            known = g.get_remembered_joint_values()
            return (MoveGroupInfoLevel.INFO, "[" + " ".join(known.keys()) + "]")

        if cmd == "joints":
            joints = g.get_joints()
            return (
                MoveGroupInfoLevel.INFO,
                "\n"
                + "\n".join([str(i) + " = " + joints[i] for i in range(len(joints))])
                + "\n",
            )

        if cmd == "show":
            return self.command_show(g)

        if cmd == "current":
            return self.command_current(g)

        if cmd == "ground":
            pose = PoseStamped()
            pose.pose.position.x = 0
            pose.pose.position.y = 0
            # offset such that the box is 0.1 mm below ground (to prevent collision with the robot itself)
            pose.pose.position.z = -0.0501
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            pose.header.stamp = rospy.get_rostime()
            pose.header.frame_id = self._robot.get_root_link()
            self._planning_scene_interface.attach_box(
                self._robot.get_root_link(), "ground", pose, (3, 3, 0.1)
            )
            return (MoveGroupInfoLevel.INFO, "Added ground")

        if cmd == "eef":
            if len(g.get_end_effector_link()) > 0:
                return (MoveGroupInfoLevel.INFO, g.get_end_effector_link())
            else:
                return (MoveGroupInfoLevel.INFO, "There is no end effector defined")

        if cmd == "database":
            if self._db_host == None:
                return (MoveGroupInfoLevel.INFO, "Not connected to a database")
            else:
                return (
                    MoveGroupInfoLevel.INFO,
                    "Connected to " + self._db_host + ":" + str(self._db_port),
                )
        if cmd == "plan":
            if self._last_plan != None:
                return (MoveGroupInfoLevel.INFO, str(self._last_plan))
            else:
                return (MoveGroupInfoLevel.INFO, "No previous plan")

        if cmd == "constrain":
            g.clear_path_constraints()
            return (MoveGroupInfoLevel.SUCCESS, "Cleared path constraints")

        if cmd == "tol" or cmd == "tolerance":
            return (
                MoveGroupInfoLevel.INFO,
                "Joint = %0.6g, Position = %0.6g, Orientation = %0.6g"
                % g.get_goal_tolerance(),
            )

        if cmd == "time":
            return (MoveGroupInfoLevel.INFO, str(g.get_planning_time()))

        if cmd == "execute":
            if self._last_plan != None:
                if g.execute(self._last_plan):
                    return (MoveGroupInfoLevel.SUCCESS, "Plan submitted for execution")
                else:
                    return (
                        MoveGroupInfoLevel.WARN,
                        "Failed to submit plan for execution",
                    )
            else:
                return (MoveGroupInfoLevel.WARN, "No motion plan computed")

        # see if we have assignment between variables
        assign_match = re.match(r"^(\w+)\s*=\s*(\w+)$", cmd)
        if assign_match:
            known = g.get_remembered_joint_values()
            if assign_match.group(2) in known:
                g.remember_joint_values(
                    assign_match.group(1), known[assign_match.group(2)]
                )
                return (
                    MoveGroupInfoLevel.SUCCESS,
                    assign_match.group(1)
                    + " is now the same as "
                    + assign_match.group(2),
                )
            else:
                return (MoveGroupInfoLevel.WARN, "Unknown command: '" + cmd + "'")

        # see if we have assignment of matlab-like vector syntax
        set_match = re.match(r"^(\w+)\s*=\s*\[([\d\.e\-\+\s]+)\]$", cmd)
        if set_match:
            try:
                g.remember_joint_values(
                    set_match.group(1), [float(x) for x in set_match.group(2).split()]
                )
                return (
                    MoveGroupInfoLevel.SUCCESS,
                    "Remembered joint values ["
                    + set_match.group(2)
                    + "] under the name "
                    + set_match.group(1),
                )
            except:
                return (
                    MoveGroupInfoLevel.WARN,
                    "Unable to parse joint value [" + set_match.group(2) + "]",
                )

        # see if we have assignment of matlab-like element update
        component_match = re.match(
            r"^(\w+)\s*\[\s*(\d+)\s*\]\s*=\s*([\d\.e\-\+]+)$", cmd
        )
        if component_match:
            known = g.get_remembered_joint_values()
            if component_match.group(1) in known:
                try:
                    val = known[component_match.group(1)]
                    val[int(component_match.group(2))] = float(component_match.group(3))
                    g.remember_joint_values(component_match.group(1), val)
                    return (
                        MoveGroupInfoLevel.SUCCESS,
                        "Updated "
                        + component_match.group(1)
                        + "["
                        + component_match.group(2)
                        + "]",
                    )
                except:
                    return (
                        MoveGroupInfoLevel.WARN,
                        "Unable to parse index or value in '" + cmd + "'",
                    )
            else:
                return (MoveGroupInfoLevel.WARN, "Unknown command: '" + cmd + "'")

        clist = cmdorig.split()
        clist[0] = clist[0].lower()

        # if this is an unknown one-word command, it is probably a variable
        if len(clist) == 1:
            known = g.get_remembered_joint_values()
            if cmd in known:
                return (
                    MoveGroupInfoLevel.INFO,
                    "[" + " ".join([str(x) for x in known[cmd]]) + "]",
                )
            else:
                return (MoveGroupInfoLevel.WARN, "Unknown command: '" + cmd + "'")

        # command with one argument
        if len(clist) == 2:
            if clist[0] == "go":
                self._last_plan = None
                if clist[1] == "rand" or clist[1] == "random":
                    vals = g.get_random_joint_values()
                    g.set_joint_value_target(vals)
                    if g.go():
                        return (
                            MoveGroupInfoLevel.SUCCESS,
                            "Moved to random target ["
                            + " ".join([str(x) for x in vals])
                            + "]",
                        )
                    else:
                        return (
                            MoveGroupInfoLevel.FAIL,
                            "Failed while moving to random target ["
                            + " ".join([str(x) for x in vals])
                            + "]",
                        )
                else:
                    try:
                        g.set_named_target(clist[1])
                        if g.go():
                            return (MoveGroupInfoLevel.SUCCESS, "Moved to " + clist[1])
                        else:
                            return (
                                MoveGroupInfoLevel.FAIL,
                                "Failed while moving to " + clist[1],
                            )
                    except MoveItCommanderException as e:
                        return (
                            MoveGroupInfoLevel.WARN,
                            "Going to " + clist[1] + ": " + str(e),
                        )
                    except:
                        return (MoveGroupInfoLevel.WARN, clist[1] + " is unknown")
            if clist[0] == "plan":
                self._last_plan = None
                vals = None
                if clist[1] == "rand" or clist[1] == "random":
                    vals = g.get_random_joint_values()
                    g.set_joint_value_target(vals)
                    self._last_plan = g.plan()[1]  # The trajectory msg
                else:
                    try:
                        g.set_named_target(clist[1])
                        self._last_plan = g.plan()[1]  # The trajectory msg
                    except MoveItCommanderException as e:
                        return (
                            MoveGroupInfoLevel.WARN,
                            "Planning to " + clist[1] + ": " + str(e),
                        )
                    except:
                        return (MoveGroupInfoLevel.WARN, clist[1] + " is unknown")
                if self._last_plan != None:
                    if (
                        len(self._last_plan.joint_trajectory.points) == 0
                        and len(self._last_plan.multi_dof_joint_trajectory.points) == 0
                    ):
                        self._last_plan = None
                dest_str = clist[1]
                if vals != None:
                    dest_str = "[" + " ".join([str(x) for x in vals]) + "]"
                if self._last_plan != None:
                    return (MoveGroupInfoLevel.SUCCESS, "Planned to " + dest_str)
                else:
                    return (
                        MoveGroupInfoLevel.FAIL,
                        "Failed while planning to " + dest_str,
                    )
            elif clist[0] == "pick":
                self._last_plan = None
                if g.pick(clist[1]):
                    return (MoveGroupInfoLevel.SUCCESS, "Picked object " + clist[1])
                else:
                    return (
                        MoveGroupInfoLevel.FAIL,
                        "Failed while trying to pick object " + clist[1],
                    )
            elif clist[0] == "place":
                self._last_plan = None
                if g.place(clist[1]):
                    return (MoveGroupInfoLevel.SUCCESS, "Placed object " + clist[1])
                else:
                    return (
                        MoveGroupInfoLevel.FAIL,
                        "Failed while trying to place object " + clist[1],
                    )
            elif clist[0] == "planner":
                g.set_planner_id(clist[1])
                return (MoveGroupInfoLevel.SUCCESS, "Planner is now " + clist[1])
            elif clist[0] == "record" or clist[0] == "rec":
                g.remember_joint_values(clist[1])
                return (
                    MoveGroupInfoLevel.SUCCESS,
                    "Remembered current joint values under the name " + clist[1],
                )
            elif clist[0] == "rand" or clist[0] == "random":
                g.remember_joint_values(clist[1], g.get_random_joint_values())
                return (
                    MoveGroupInfoLevel.SUCCESS,
                    "Remembered random joint values under the name " + clist[1],
                )
            elif clist[0] == "del" or clist[0] == "delete":
                g.forget_joint_values(clist[1])
                return (
                    MoveGroupInfoLevel.SUCCESS,
                    "Forgot joint values under the name " + clist[1],
                )
            elif clist[0] == "show":
                known = g.get_remembered_joint_values()
                if clist[1] in known:
                    return (
                        MoveGroupInfoLevel.INFO,
                        "[" + " ".join([str(x) for x in known[clist[1]]]) + "]",
                    )
                else:
                    return (
                        MoveGroupInfoLevel.WARN,
                        "Joint values for " + clist[1] + " are not known",
                    )
            elif clist[0] == "tol" or clist[0] == "tolerance":
                try:
                    g.set_goal_tolerance(float(clist[1]))
                    return (MoveGroupInfoLevel.SUCCESS, "OK")
                except:
                    return (
                        MoveGroupInfoLevel.WARN,
                        "Unable to parse tolerance value '" + clist[1] + "'",
                    )
            elif clist[0] == "time":
                try:
                    g.set_planning_time(float(clist[1]))
                    return (MoveGroupInfoLevel.SUCCESS, "OK")
                except:
                    return (
                        MoveGroupInfoLevel.WARN,
                        "Unable to parse planning duration value '" + clist[1] + "'",
                    )
            elif clist[0] == "constrain":
                try:
                    g.set_path_constraints(clist[1])
                    return (MoveGroupInfoLevel.SUCCESS, "OK")
                except:
                    if self._db_host != None:
                        return (
                            MoveGroupInfoLevel.WARN,
                            "Constraint " + clist[1] + " is not known.",
                        )
                    else:
                        return (MoveGroupInfoLevel.WARN, "Not connected to a database.")
            elif clist[0] == "wait":
                try:
                    time.sleep(float(clist[1]))
                    return (MoveGroupInfoLevel.SUCCESS, clist[1] + " seconds passed")
                except:
                    return (
                        MoveGroupInfoLevel.WARN,
                        "Unable to wait '" + clist[1] + "' seconds",
                    )
            elif clist[0] == "database":
                try:
                    g.set_constraints_database(clist[1], self._db_port)
                    self._db_host = clist[1]
                    return (
                        MoveGroupInfoLevel.SUCCESS,
                        "Connected to " + self._db_host + ":" + str(self._db_port),
                    )
                except:
                    return (
                        MoveGroupInfoLevel.WARN,
                        "Unable to connect to '"
                        + clist[1]
                        + ":"
                        + str(self._db_port)
                        + "'",
                    )
            else:
                return (MoveGroupInfoLevel.WARN, "Unknown command: '" + cmd + "'")

        if len(clist) == 3:
            if clist[0] == "go" and clist[1] in self.GO_DIRS:
                self._last_plan = None
                try:
                    offset = float(clist[2])
                    (axis, factor) = self.GO_DIRS[clist[1]]
                    return self.command_go_offset(g, offset, factor, axis, clist[1])
                except MoveItCommanderException as e:
                    return (
                        MoveGroupInfoLevel.WARN,
                        "Going " + clist[1] + ": " + str(e),
                    )
                except:
                    return (
                        MoveGroupInfoLevel.WARN,
                        "Unable to parse distance '" + clist[2] + "'",
                    )
            elif clist[0] == "allow" and (clist[1] == "look" or clist[1] == "looking"):
                if (
                    clist[2] == "1"
                    or clist[2] == "true"
                    or clist[2] == "T"
                    or clist[2] == "True"
                ):
                    g.allow_looking(True)
                else:
                    g.allow_looking(False)
                return (MoveGroupInfoLevel.DEBUG, "OK")
            elif clist[0] == "allow" and (
                clist[1] == "replan" or clist[1] == "replanning"
            ):
                if (
                    clist[2] == "1"
                    or clist[2] == "true"
                    or clist[2] == "T"
                    or clist[2] == "True"
                ):
                    g.allow_replanning(True)
                else:
                    g.allow_replanning(False)
                return (MoveGroupInfoLevel.DEBUG, "OK")
            elif clist[0] == "database":
                try:
                    g.set_constraints_database(clist[1], int(clist[2]))
                    self._db_host = clist[1]
                    self._db_port = int(clist[2])
                    return (
                        MoveGroupInfoLevel.SUCCESS,
                        "Connected to " + self._db_host + ":" + str(self._db_port),
                    )
                except:
                    self._db_host = None
                    return (
                        MoveGroupInfoLevel.WARN,
                        "Unable to connect to '" + clist[1] + ":" + clist[2] + "'",
                    )
        if len(clist) == 4:
            if clist[0] == "rotate":
                try:
                    g.set_rpy_target([float(x) for x in clist[1:]])
                    if g.go():
                        return (MoveGroupInfoLevel.SUCCESS, "Rotation complete")
                    else:
                        return (
                            MoveGroupInfoLevel.FAIL,
                            "Failed while rotating to " + " ".join(clist[1:]),
                        )
                except MoveItCommanderException as e:
                    return (MoveGroupInfoLevel.WARN, str(e))
                except:
                    return (
                        MoveGroupInfoLevel.WARN,
                        "Unable to parse X-Y-Z rotation  values '"
                        + " ".join(clist[1:])
                        + "'",
                    )
        if len(clist) >= 7:
            if clist[0] == "go":
                self._last_plan = None
                approx = False
                if len(clist) > 7:
                    if clist[7] == "approx" or clist[7] == "approximate":
                        approx = True
                try:
                    p = Pose()
                    p.position.x = float(clist[1])
                    p.position.y = float(clist[2])
                    p.position.z = float(clist[3])
                    q = tf.transformations.quaternion_from_euler(
                        float(clist[4]), float(clist[5]), float(clist[6])
                    )
                    p.orientation.x = q[0]
                    p.orientation.y = q[1]
                    p.orientation.z = q[2]
                    p.orientation.w = q[3]
                    if approx:
                        g.set_joint_value_target(p, True)
                    else:
                        g.set_pose_target(p)
                    if g.go():
                        return (
                            MoveGroupInfoLevel.SUCCESS,
                            "Moved to pose target\n%s\n" % (str(p)),
                        )
                    else:
                        return (
                            MoveGroupInfoLevel.FAIL,
                            "Failed while moving to pose \n%s\n" % (str(p)),
                        )
                except MoveItCommanderException as e:
                    return (
                        MoveGroupInfoLevel.WARN,
                        "Going to pose target: %s" % (str(e)),
                    )
                except:
                    return (
                        MoveGroupInfoLevel.WARN,
                        "Unable to parse pose '" + " ".join(clist[1:]) + "'",
                    )

        return (MoveGroupInfoLevel.WARN, "Unknown command: '" + cmd + "'")

    def command_show(self, g):
        known = g.get_remembered_joint_values()
        res = []
        for k in known.keys():
            res.append(k + " = [" + " ".join([str(x) for x in known[k]]) + "]")
        return (MoveGroupInfoLevel.INFO, "\n".join(res))

    def command_current(self, g):
        res = (
            "joints = ["
            + ", ".join([str(x) for x in g.get_current_joint_values()])
            + "]"
        )
        if len(g.get_end_effector_link()) > 0:
            res = (
                res
                + "\n\n"
                + g.get_end_effector_link()
                + " pose = [\n"
                + str(g.get_current_pose())
                + " ]"
            )
            res = (
                res
                + "\n"
                + g.get_end_effector_link()
                + " RPY = "
                + str(g.get_current_rpy())
            )
        return (MoveGroupInfoLevel.INFO, res)

    def command_go_offset(self, g, offset, factor, dimension_index, direction_name):
        if g.has_end_effector_link():
            try:
                g.shift_pose_target(dimension_index, factor * offset)
                if g.go():
                    return (
                        MoveGroupInfoLevel.SUCCESS,
                        "Moved " + direction_name + " by " + str(offset) + " m",
                    )
                else:
                    return (
                        MoveGroupInfoLevel.FAIL,
                        "Failed while moving " + direction_name,
                    )
            except MoveItCommanderException as e:
                return (MoveGroupInfoLevel.WARN, str(e))
            except:
                return (MoveGroupInfoLevel.WARN, "Unable to process pose update")
        else:
            return (
                MoveGroupInfoLevel.WARN,
                "No known end effector. Cannot move " + direction_name,
            )

    def resolve_command_alias(self, cmdorig):
        cmd = cmdorig.lower()
        if cmd == "which":
            return "id"
        if cmd == "groups":
            return "use"
        return cmdorig

    def get_help(self):
        res = []
        res.append("Known commands:")
        res.append("  help                show this screen")
        res.append("  allow looking <true|false>       enable/disable looking around")
        res.append("  allow replanning <true|false>    enable/disable replanning")
        res.append("  constrain           clear path constraints")
        res.append(
            "  constrain <name>    use the constraint <name> as a path constraint"
        )
        res.append("  current             show the current state of the active group")
        res.append(
            "  database            display the current database connection (if any)"
        )
        res.append(
            "  delete <name>       forget the joint values under the name <name>"
        )
        res.append(
            "  eef                 print the name of the end effector attached to the current group"
        )
        res.append("  execute             execute a previously computed motion plan")
        res.append(
            "  go <name>           plan and execute a motion to the state <name>"
        )
        res.append("  go rand             plan and execute a motion to a random state")
        res.append(
            "  go <dir> <dx>|      plan and execute a motion in direction up|down|left|right|forward|backward for distance <dx>"
        )
        res.append("  ground              add a ground plane to the planning scene")
        res.append(
            "  id|which            display the name of the group that is operated on"
        )
        res.append(
            "  joints              display names of the joints in the active group"
        )
        res.append(
            "  load [<file>]       load a set of interpreted commands from a file"
        )
        res.append("  pick <name>         pick up object <name>")
        res.append("  place <name>        place object <name>")
        res.append("  plan <name>         plan a motion to the state <name>")
        res.append("  plan rand           plan a motion to a random state")
        res.append("  planner <name>      use planner <name> to plan next motion")
        res.append(
            "  record <name>       record the current joint values under the name <name>"
        )
        res.append(
            "  rotate <x> <y> <z>  plan and execute a motion to a specified orientation (about the X,Y,Z axes)"
        )
        res.append(
            "  save [<file>]       save the currently known variables as a set of commands"
        )
        res.append(
            "  show                display the names and values of the known states"
        )
        res.append("  show <name>         display the value of a state")
        res.append("  stop                stop the active group")
        res.append("  time                show the configured allowed planning time")
        res.append("  time <val>          set the allowed planning time")
        res.append(
            "  tolerance           show the tolerance for reaching the goal region"
        )
        res.append(
            "  tolerance <val>     set the tolerance for reaching the goal region"
        )
        res.append("  trace <on|off>      enable/disable replanning or looking around")
        res.append(
            "  use <name>          switch to using the group named <name> (and load it if necessary)"
        )
        res.append("  use|groups          show the group names that are already loaded")
        res.append("  vars                display the names of the known states")
        res.append("  wait <dt>           sleep for <dt> seconds")
        res.append("  x = y               assign the value of y to x")
        res.append("  x = [v1 v2...]      assign a vector of values to x")
        res.append("  x[idx] = val        assign a value to dimension idx of x")
        return "\n".join(res)

    def get_keywords(self):
        known_vars = []
        known_constr = []
        if self.get_active_group() != None:
            known_vars = self.get_active_group().get_remembered_joint_values().keys()
            known_constr = self.get_active_group().get_known_constraints()
        groups = self._robot.get_group_names()
        return {
            "go": ["up", "down", "left", "right", "backward", "forward", "random"]
            + list(known_vars),
            "help": [],
            "record": known_vars,
            "show": known_vars,
            "wait": [],
            "delete": known_vars,
            "database": [],
            "current": [],
            "use": groups,
            "load": [],
            "save": [],
            "pick": [],
            "place": [],
            "plan": known_vars,
            "allow": ["replanning", "looking"],
            "constrain": known_constr,
            "vars": [],
            "joints": [],
            "tolerance": [],
            "time": [],
            "eef": [],
            "execute": [],
            "ground": [],
            "id": [],
        }
