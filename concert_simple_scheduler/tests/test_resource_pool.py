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
try:
    from scheduler_msgs.msg import CurrentStatus
except ImportError:
    from rocon_scheduler_requests.resources import CurrentStatus
from rocon_scheduler_requests.transitions import ResourceReply
from rocon_scheduler_requests.resources import ResourceSet

# module being tested:
from concert_simple_scheduler.resource_pool import *

# some resources for testing
RQ_UUID = uuid.UUID('01234567-89ab-cdef-0123-456789abcdef')
EXAMPLE_RAPP = 'tests/example_rapp'
TELEOP_RAPP = 'rocon_apps/teleop'
TEST_RAPPS = set((TELEOP_RAPP, EXAMPLE_RAPP))
MARVIN_NAME = 'rocon:///linux/precise/ros/turtlebot/marvin'
MARVIN = CurrentStatus(platform_info=MARVIN_NAME, rapps=TEST_RAPPS)
ROBERTO_NAME = 'rocon:///linux/precise/ros/turtlebot/roberto'
ROBERTO = CurrentStatus(platform_info=ROBERTO_NAME, rapps=TEST_RAPPS)
SINGLETON_POOL = KnownResources(resources=[ROBERTO])
DOUBLETON_POOL = KnownResources(resources=[MARVIN, ROBERTO])

# some useful Resource and Request messages
MARVIN_RESOURCE = Resource(name=TELEOP_RAPP, platform_info=MARVIN_NAME)
ROBERTO_RESOURCE = Resource(name=TELEOP_RAPP, platform_info=ROBERTO_NAME)
ROBERTO_REQUEST = ResourceReply(Request(
    id=unique_id.toMsg(RQ_UUID),
    resources=[ROBERTO_RESOURCE]))


class TestResourcePool(unittest.TestCase):
    """Unit tests for simple scheduler resource pool class.

    These tests do not require a running ROS core.
    """

    ####################
    # ROCON resource pool tests
    ####################

    def test_empty_constructor(self):
        rp0 = ResourcePool()
        self.assertIsNotNone(rp0)
        self.assertEqual(len(rp0.pool), 0)
        self.assertNotIn(MARVIN_NAME, rp0.pool)

    def test_one_resource_constructor(self):
        rp1 = ResourcePool(ResourceSet(SINGLETON_POOL))
        self.assertEqual(len(rp1.pool), 1)
        self.assertIn(ROBERTO_NAME, rp1.pool)
        self.assertNotIn(MARVIN_NAME, rp1.pool)
        self.assertMultiLineEqual(str(rp1.pool),
                                  str(ResourceSet(SINGLETON_POOL)))

    def test_two_resource_constructor(self):
        rp2 = ResourcePool(ResourceSet(DOUBLETON_POOL))
        self.assertEqual(len(rp2.pool), 2)
        self.assertIn(ROBERTO_NAME, rp2.pool)
        self.assertIn(MARVIN_NAME, rp2.pool)
        self.assertMultiLineEqual(str(rp2.pool),
                                  str(ResourceSet(DOUBLETON_POOL)))

    def test_exact_resource_allocation(self):
        rp2 = ResourcePool(ResourceSet(DOUBLETON_POOL))
        res = copy.deepcopy(ROBERTO_RESOURCE)
        subset = rp2.match_subset(res)
        self.assertIn(ROBERTO_NAME, subset)
        self.assertEqual(subset, set([ROBERTO_NAME]))
        self.assertEqual(rp2.match_list([ROBERTO_RESOURCE]),
                         [set([ROBERTO_NAME])])
        rq = copy.deepcopy(ROBERTO_REQUEST)
        alloc = rp2.allocate(rq)
        self.assertEqual(alloc[0], ROBERTO_RESOURCE)
        self.assertEqual(rp2.pool[ROBERTO_NAME].status, CurrentStatus.ALLOCATED)
        self.assertEqual(rp2.pool[ROBERTO_NAME].owner, RQ_UUID)

    def test_deallocate_one_resource(self):
        rp2 = ResourcePool(ResourceSet(DOUBLETON_POOL))
        self.assertEqual(rp2.pool[ROBERTO_NAME].status, CurrentStatus.AVAILABLE)

        rq = copy.deepcopy(ROBERTO_REQUEST)
        alloc = rp2.allocate(rq)
        self.assertEqual(rp2.pool[ROBERTO_NAME].status, CurrentStatus.ALLOCATED)
        self.assertEqual(rp2.pool[ROBERTO_NAME].owner, RQ_UUID)

        rp2.deallocate(alloc)
        self.assertEqual(rp2.pool[ROBERTO_NAME].status, CurrentStatus.AVAILABLE)
        self.assertEqual(rp2.pool[ROBERTO_NAME].owner, None)

    def test_release_one_resource(self):
        rp2 = ResourcePool(ResourceSet(DOUBLETON_POOL))
        self.assertEqual(rp2.pool[ROBERTO_NAME].status, CurrentStatus.AVAILABLE)

        rq = copy.deepcopy(ROBERTO_REQUEST)
        alloc = rp2.allocate(rq)
        rq.grant(alloc)
        self.assertEqual(rp2.pool[ROBERTO_NAME].status, CurrentStatus.ALLOCATED)
        self.assertEqual(rp2.pool[ROBERTO_NAME].owner, RQ_UUID)

        rp2.release(rq)
        self.assertEqual(rp2.pool[ROBERTO_NAME].status, CurrentStatus.AVAILABLE)
        self.assertEqual(rp2.pool[ROBERTO_NAME].owner, None)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('concert_simple_scheduler',
                    'test_resource_pool',
                    TestResourcePool)
