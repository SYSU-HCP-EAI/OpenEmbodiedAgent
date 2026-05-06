#!/usr/bin/env python

from __future__ import print_function

import roslib
import rospy

try:
    import readline
except ImportError:
    import pyreadline as readline  # for Windows
import sys
import os
import signal

import argparse
from moveit_commander import (
    MoveGroupCommandInterpreter,
    MoveGroupInfoLevel,
    roscpp_initialize,
    roscpp_shutdown,
)

# python3 has renamed raw_input to input: https://www.python.org/dev/peps/pep-3111
# Here, we use the new input(). Hence, for python2, we redirect raw_input to input
try:
    import __builtin__  # This is named builtin in python3

    input = getattr(__builtin__, "raw_input")
except (ImportError, AttributeError):
    pass


class bcolors:
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"


class SimpleCompleter(object):
    def __init__(self, options):
        self.options = options

    def set_options(self, options):
        self.options = options

    def complete(self, text, state):
        response = None
        cmds = readline.get_line_buffer().split()
        prefix = ""
        if len(cmds) > 0:
            prefix = cmds[0]
            if not prefix in self.options:
                prefix = ""

        if state == 0:
            # This is the first time for this text, so build a match list.
            if text:
                if len(prefix) == 0:
                    self.matches = sorted(
                        [s for s in self.options.keys() if s and s.startswith(text)]
                    )
                else:
                    self.matches = sorted(
                        [s for s in self.options[prefix] if s and s.startswith(text)]
                    )
            else:
                if len(prefix) == 0:
                    self.matches = sorted(self.options.keys())
                else:
                    self.matches = self.options[prefix]

        # Return the state'th item from the match list,
        # if we have that many.
        try:
            response = self.matches[state] + " "
        except IndexError:
            response = None
        return response


def print_message(level, msg):
    if level == MoveGroupInfoLevel.FAIL:
        print(bcolors.FAIL + msg + bcolors.ENDC)
    elif level == MoveGroupInfoLevel.WARN:
        print(bcolors.WARNING + msg + bcolors.ENDC)
    elif level == MoveGroupInfoLevel.SUCCESS:
        print(bcolors.OKGREEN + msg + bcolors.ENDC)
    elif level == MoveGroupInfoLevel.DEBUG:
        print(bcolors.OKBLUE + msg + bcolors.ENDC)
    else:
        print(msg)


def get_context_keywords(interpreter):
    kw = interpreter.get_keywords()
    kw["quit"] = []
    return kw


def run_interactive(group_name):
    c = MoveGroupCommandInterpreter()
    if len(group_name) > 0:
        c.execute("use " + group_name)
    completer = SimpleCompleter(get_context_keywords(c))
    readline.set_completer(completer.complete)

    print()
    print(
        bcolors.HEADER
        + "Waiting for commands. Type 'help' to get a list of known commands."
        + bcolors.ENDC
    )
    print()
    readline.parse_and_bind("tab: complete")

    while not rospy.is_shutdown():
        cmd = ""
        try:
            name = ""
            ag = c.get_active_group()
            if ag != None:
                name = ag.get_name()
            cmd = input(bcolors.OKBLUE + name + "> " + bcolors.ENDC)
        except:
            break
        cmdorig = cmd.strip()
        if cmdorig == "":
            continue
        cmd = cmdorig.lower()

        if cmd == "q" or cmd == "quit" or cmd == "exit":
            break
        if cmd == "host":
            print_message(
                MoveGroupInfoLevel.INFO,
                "Master is '" + os.environ["ROS_MASTER_URI"] + "'",
            )
            continue

        (level, msg) = c.execute(cmdorig)
        print_message(level, msg)
        # update the set of keywords
        completer.set_options(get_context_keywords(c))


def run_service(group_name):
    c = MoveGroupCommandInterpreter()
    if len(group_name) > 0:
        c.execute("use " + group_name)
    # add service stuff
    print("Running ROS service")
    rospy.spin()


def stop_ros(reason):
    rospy.signal_shutdown(reason)
    roscpp_shutdown()


def sigint_handler(signal, frame):
    stop_ros("Ctrl+C pressed")
    # this won't actually exit, but trigger an exception to terminate input
    sys.exit(0)


if __name__ == "__main__":

    signal.signal(signal.SIGINT, sigint_handler)

    roscpp_initialize(sys.argv)

    rospy.init_node(
        "move_group_interface_cmdline", anonymous=True, disable_signals=True
    )

    parser = argparse.ArgumentParser(
        usage="""%(prog)s [options] [<group_name>]""",
        description="Command Line Interface to MoveIt",
    )
    parser.add_argument(
        "-i",
        "--interactive",
        action="store_true",
        dest="interactive",
        default=True,
        help="Run the command processing script in interactive mode (default)",
    )
    parser.add_argument(
        "-s",
        "--service",
        action="store_true",
        dest="service",
        default=False,
        help="Run the command processing script as a ROS service",
    )
    parser.add_argument(
        "group_name",
        type=str,
        default="",
        nargs="?",
        help="Group name to initialize the CLI for.",
    )

    opt = parser.parse_args(rospy.myargv()[1:])

    if opt.service:
        run_service(opt.group_name)
    else:
        run_interactive(opt.group_name)

    stop_ros("Done")

    print("Bye bye!")
