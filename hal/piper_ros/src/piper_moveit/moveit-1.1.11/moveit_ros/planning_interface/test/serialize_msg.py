#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2020, RWTH Aachen University
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
#  * Neither the name of RWTH Aachen University nor the names of its
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
# Author: Bjarne von Horn
#

from moveit_ros_planning_interface._moveit_planning_interface_test_serialize_msg_cpp_helper import (
    ByteStringTestHelper,
)
from geometry_msgs.msg import Vector3
import unittest

try:
    # Try Python 2.7 behaviour first
    from StringIO import StringIO

    py_version_maj = 2
except ImportError:
    # Use Python 3.x behaviour as fallback and choose the non-unicode version
    from io import BytesIO as StringIO

    py_version_maj = 3


class PythonMsgSerializeTest(unittest.TestCase):
    def setUp(self):
        self.helper = ByteStringTestHelper()

    def test_EmbeddedZeros(self):
        self.assertTrue(self.helper.compareEmbeddedZeros(b"\xff\xef\x00\x10"))

    def test_ByteStringFromPchar(self):
        # ByteString(const char*) constructor
        self.assertEqual(self.helper.getBytesPChar(), b"abcdef")

    def test_ByteStringFromStdString(self):
        # ByteString(const std::string&) constructor
        self.assertEqual(self.helper.getBytesStdString(), b"\xff\xfe\x10\x00\x00")

    def test_ByteStringDefaultCtor(self):
        self.assertEqual(self.helper.getDefaultBytes(), b"")

    def test_CxxTupleToPy(self):
        # sending a tuple from C++ to Python
        ans = self.helper.getTuple()
        self.assertIsInstance(ans, tuple)
        self.assertEqual(len(ans), 1)
        self.assertEqual(ans[0], b"abcdef")

    def test_PyTupleToCxx(self):
        # sending a tuple from Python to C++
        self.assertTrue(self.helper.compareTuple((b"mno",)))

    def test_sendMessage(self):
        tmp = StringIO()
        Vector3(x=1.0, y=-2.0, z=0.25).serialize(tmp)
        self.assertTrue(self.helper.compareVector(tmp.getvalue()))

    def test_recieveMessage(self):
        tmp = Vector3()
        tmp.deserialize(self.helper.getVector())
        self.assertEqual(tmp, Vector3(1.0, -2.0, 0.25))

    def test_rejectInt(self):
        with self.assertRaisesRegexp(Exception, "Python argument types in"):
            self.helper.compareEmbeddedZeros(4711)

    def test_rejectIntTuple(self):
        with self.assertRaisesRegexp(Exception, "Python argument types in"):
            self.helper.compareEmbeddedZeros((4711,))

    def test_rejectUnicode(self):
        with self.assertRaisesRegexp(Exception, "Python argument types in"):
            self.helper.compareEmbeddedZeros(u"kdasd")  # fmt: skip

    @unittest.skipIf(py_version_maj == 2, "does not trigger with python 2.7")
    def test_rejectUnicodeTuple(self):
        with self.assertRaisesRegexp(
            RuntimeError, "Underlying python object is not a Bytes/String instance"
        ):
            self.helper.compareVectorTuple((u"kdasd",))  # fmt: skip


if __name__ == "__main__":
    unittest.main()
