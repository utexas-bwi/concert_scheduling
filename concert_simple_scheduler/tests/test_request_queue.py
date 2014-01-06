#!/usr/bin/env python

# enable some python3 compatibility options:
# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

import copy
import heapq
import uuid
import unittest

# ROS dependencies
import unique_id
from scheduler_msgs.msg import Request, Resource
from rocon_scheduler_requests.transitions import ResourceReply

# module being tested:
from concert_simple_scheduler.request_queue import *

# some resources for testing
RQR_ID = uuid.uuid4()
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


###############################
# queue element tests
###############################


class TestQueueElement(unittest.TestCase):
    """Unit tests for queue element class.

    These tests do not require a running ROS core.
    """
    def test_constructor(self):
        qe1 = QueueElement(ROBERTO_REQUEST, RQR_ID)
        qe2 = QueueElement(MARVIN_REQUEST, RQR_ID)
        self.assertNotEqual(qe1, qe2)

    def test_hash(self):
        qe1 = QueueElement(ROBERTO_REQUEST, RQR_ID)
        qe2 = QueueElement(MARVIN_REQUEST, RQR_ID)
        self.assertNotEqual(qe1, qe2)
        self.assertTrue(qe1 != qe2)
        self.assertNotEqual(hash(qe1), hash(qe2))
        qe3 = QueueElement(ROBERTO_REQUEST, RQR_ID)
        self.assertEqual(qe1, qe3)      # same request ID
        self.assertFalse(qe1 != qe3)
        self.assertEqual(hash(qe1), hash(qe3))
        dict = {qe1: qe1}
        self.assertIn(qe1, dict)
        self.assertNotIn(qe2, dict)
        self.assertIn(qe3, dict)        # because hashes are equal
        dict[qe2] = qe2
        self.assertIn(qe2, dict)

    def test_heap_queue(self):
        qe1 = QueueElement(ResourceReply(
                Request(id=unique_id.toMsg(RQ1_UUID),
                        resources=[ROBERTO_RESOURCE],
                        priority=10)
                ), RQR_ID)
        qe2 = QueueElement(ResourceReply(
                Request(id=unique_id.toMsg(RQ1_UUID),
                        resources=[MARVIN_RESOURCE],
                        priority=0)
                ), RQR_ID)
        self.assertLess(qe1, qe2)       # due to higher priority
        h = []
        heapq.heappush(h, qe2)
        heapq.heappush(h, qe1)
        self.assertEqual(len(h), 2)
        self.assertEqual(heapq.heappop(h), qe1)
        self.assertEqual(len(h), 1)
        heapq.heappush(h, qe1)
        self.assertEqual(len(h), 2)
        self.assertEqual(heapq.heappop(h), qe1)

        qe3 = QueueElement(ResourceReply(
                Request(id=unique_id.toMsg(RQ1_UUID),
                        resources=[ROBERTO_RESOURCE])
                ), RQR_ID)
        qe4 = QueueElement(ResourceReply(
                Request(id=unique_id.toMsg(RQ1_UUID),
                        resources=[MARVIN_RESOURCE])
                ), RQR_ID)
        self.assertLess(qe3, qe4)       # due to sequence number
        heapq.heappush(h, qe4)
        heapq.heappush(h, qe3)
        heapq.heappush(h, qe1)
        self.assertEqual(len(h), 4)
        self.assertEqual(heapq.heappop(h), qe1)
        self.assertEqual(heapq.heappop(h), qe2)
        self.assertEqual(heapq.heappop(h), qe3)
        self.assertEqual(heapq.heappop(h), qe4)
        self.assertEqual(len(h), 0)
        self.assertRaises(IndexError, heapq.heappop, h)

    def test_sort_diff_priority(self):
        qe1 = QueueElement(ResourceReply(
                Request(id=unique_id.toMsg(RQ1_UUID),
                        resources=[ROBERTO_RESOURCE],
                        priority=10)
                ), RQR_ID)
        qe2 = QueueElement(ResourceReply(
                Request(id=unique_id.toMsg(RQ1_UUID),
                        resources=[MARVIN_RESOURCE],
                        priority=0)
                ), RQR_ID)
        self.assertLess(qe1, qe2)
        self.assertEqual(sorted([qe2, qe1]), [qe1, qe2])
        qe3 = QueueElement(ResourceReply(
                Request(id=unique_id.toMsg(RQ1_UUID),
                        resources=[ROBERTO_RESOURCE])
                ), RQR_ID)
        qe4 = QueueElement(ResourceReply(
                Request(id=unique_id.toMsg(RQ1_UUID),
                        resources=[MARVIN_RESOURCE])
                ), RQR_ID)
        self.assertEqual(sorted([qe4, qe3]), [qe3, qe4])
        self.assertEqual(sorted([qe4, qe1, qe3, qe2]),
                         [qe1, qe2, qe3, qe4])

    def test_sort_same_priority(self):
        qe1 = QueueElement(ROBERTO_REQUEST, RQR_ID)
        qe2 = QueueElement(MARVIN_REQUEST, RQR_ID)
        self.assertLess(qe1, qe2)
        list1 = [qe1]
        self.assertEqual(sorted(list1), [qe1])
        list2 = [qe2, qe1]
        list2.sort()                    # sort in-place
        self.assertEqual(list2, [qe1, qe2])


###############################
# scheduler request queue tests
###############################


class TestRequestQueue(unittest.TestCase):
    """Unit tests for simple scheduler FIFO request queue class.

    These tests do not require a running ROS core.
    """
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
                    'test_queue_element',
                    TestQueueElement)
    rosunit.unitrun('concert_simple_scheduler',
                    'test_request_queue',
                    TestRequestQueue)
