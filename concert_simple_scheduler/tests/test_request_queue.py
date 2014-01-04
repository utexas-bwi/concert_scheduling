#!/usr/bin/env python

# enable some python3 compatibility options:
# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

import copy
import uuid
import unittest

# ROS dependencies
import unique_id
from scheduler_msgs.msg import Request, Resource
from rocon_scheduler_requests.transitions import ResourceReply

# module being tested:
from concert_simple_scheduler.request_queue import *

# some resources for testing
RQ1_UUID = uuid.uuid4()
RQ2_UUID = uuid.uuid4()
EXAMPLE_RAPP = 'tests/example_rapp'
MARVIN_NAME = 'rocon:///linux/precise/ros/turtlebot/marvin'
MARVIN = Resource(platform_info=MARVIN_NAME, name=EXAMPLE_RAPP)
ROBERTO_NAME = 'rocon:///linux/precise/ros/turtlebot/roberto'
ROBERTO = Resource(platform_info=ROBERTO_NAME, name=EXAMPLE_RAPP)

# some useful Resource and Request messages
MARVIN_RESOURCE = Resource(name=EXAMPLE_RAPP, platform_info=MARVIN_NAME)
MARVIN_REQUEST = ResourceReply(Request(
    id=unique_id.toMsg(RQ1_UUID),
    resources=[MARVIN_RESOURCE]))
ROBERTO_RESOURCE = Resource(name=EXAMPLE_RAPP, platform_info=ROBERTO_NAME)
ROBERTO_REQUEST = ResourceReply(Request(
    id=unique_id.toMsg(RQ2_UUID),
    resources=[ROBERTO_RESOURCE]))


class TestRequestQueue(unittest.TestCase):
    """Unit tests for simple scheduler request queue class.

    These tests do not require a running ROS core.
    """

    ###############################
    # scheduler request queue tests
    ###############################

    def test_append_one_resource(self):
        rqq1 = RequestQueue()
        self.assertEqual(len(rqq1), 0)
        rqq1.append(ROBERTO_REQUEST)
        self.assertEqual(len(rqq1), 1)

    def test_empty_constructor(self):
        rqq0 = RequestQueue()
        self.assertIsNotNone(rqq0)
        self.assertEqual(len(rqq0), 0)
        self.assertRaises(IndexError, rqq0.popleft)

    def test_one_request_constructor(self):
        rqq1 = RequestQueue([ROBERTO_REQUEST])
        self.assertEqual(len(rqq1), 1)
        rq1 = rqq1.popleft()
        self.assertEqual(len(rqq1), 0)
        self.assertMultiLineEqual(str(rq1), str(ROBERTO_REQUEST))

    def test_pop_one_request(self):
        rqq1 = RequestQueue([MARVIN_REQUEST, ROBERTO_REQUEST])
        self.assertEqual(len(rqq1), 2)
        self.assertMultiLineEqual(str(rqq1.pop()), str(ROBERTO_REQUEST))

    def test_popleft_one_request(self):
        rqq1 = RequestQueue([MARVIN_REQUEST, ROBERTO_REQUEST])
        self.assertEqual(len(rqq1), 2)
        self.assertMultiLineEqual(str(rqq1.popleft()), str(MARVIN_REQUEST))

    def test_two_request_constructor(self):
        rqq2 = RequestQueue([MARVIN_REQUEST, ROBERTO_REQUEST])
        self.assertEqual(len(rqq2), 2)

        rq1 = rqq2.popleft()
        self.assertEqual(len(rqq2), 1)
        self.assertMultiLineEqual(str(rq1), str(MARVIN_REQUEST))

        rq2 = rqq2.popleft()
        self.assertEqual(len(rqq2), 0)
        self.assertMultiLineEqual(str(rq2), str(ROBERTO_REQUEST))

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('concert_simple_scheduler',
                    'test_request_queue',
                    TestRequestQueue)
