#!/usr/bin/env python

from __future__ import print_function

import unittest
import rostest
import subprocess
import rospkg
import roslib.packages

PKGNAME = "moveit_ros_planning_interface"
NODENAME = "moveit_cleanup_tests"

# As issue #592 is related to a crash during program exit,
# we cannot perform a standard unit test.
# We have to check the return value of the called program.
# As rostest doesn't do this automatically, we do it ourselves
# and call the actual test program here.
class CleanupTest(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(CleanupTest, self).__init__(*args, **kwargs)
        self._rospack = rospkg.RosPack()

    def run_cmd(self, cmd, num=5):
        failures = 0
        for i in range(num):
            if subprocess.call(cmd) != 0:
                failures += 1
        self.assertEqual(failures, 0, "%d of %d runs failed" % (failures, num))

    def test_py(self):
        self.run_cmd(
            roslib.packages.find_node(PKGNAME, "test_cleanup.py", self._rospack)
        )

    def test_cpp(self):
        self.run_cmd(roslib.packages.find_node(PKGNAME, "test_cleanup", self._rospack))


if __name__ == "__main__":
    rostest.rosrun(PKGNAME, NODENAME, CleanupTest)
