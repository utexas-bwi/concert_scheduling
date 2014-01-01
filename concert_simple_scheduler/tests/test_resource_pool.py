#!/usr/bin/env python

# enable some python3 compatibility options:
# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

import copy
import uuid
import unittest

# ROS dependencies
import unique_id
from scheduler_msgs.msg import Resource
try:
    from scheduler_msgs.msg import CurrentStatus
except ImportError:
    from rocon_scheduler_requests.resources import CurrentStatus
from rocon_scheduler_requests.resources import ResourceSet

# module being tested:
from concert_simple_scheduler.resource_pool import *

# some resources for testing
EXAMPLE_RAPP = 'tests/example_rapp'
TEST_RAPPS = set(('rocon_apps/teleop', EXAMPLE_RAPP))
MARVIN = CurrentStatus(
    platform_info='rocon:///linux/precise/ros/turtlebot/marvin',
    rapps=TEST_RAPPS)
ROBERTO = CurrentStatus(
    platform_info='rocon:///linux/precise/ros/turtlebot/roberto',
    rapps=TEST_RAPPS)
SINGLETON_POOL = KnownResources(resources=[ROBERTO])
DOUBLETON_POOL = KnownResources(resources=[MARVIN, ROBERTO])


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
        self.assertNotIn(MARVIN.platform_info, rp0.pool)

    def test_one_resource_constructor(self):
        rp1 = ResourcePool(ResourceSet(SINGLETON_POOL))
        self.assertEqual(len(rp1.pool), 1)
        self.assertIn(ROBERTO.platform_info, rp1.pool)
        self.assertNotIn(MARVIN.platform_info, rp1.pool)
        self.assertMultiLineEqual(str(rp1.pool),
                                  str(ResourceSet(SINGLETON_POOL)))

    def test_two_resource_constructor(self):
        rp2 = ResourcePool(ResourceSet(DOUBLETON_POOL))
        self.assertEqual(len(rp2.pool), 2)
        self.assertIn(ROBERTO.platform_info, rp2.pool)
        self.assertIn(MARVIN.platform_info, rp2.pool)
        self.assertMultiLineEqual(str(rp2.pool),
                                  str(ResourceSet(DOUBLETON_POOL)))

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('concert_simple_scheduler',
                    'test_resource_pool',
                    TestResourcePool)
