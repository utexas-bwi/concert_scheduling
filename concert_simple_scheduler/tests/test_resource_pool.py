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
    from scheduler_msgs.msg import CurrentStatus, KnownResources
except ImportError:
    from .resource_pool import CurrentStatus, KnownResources
from rocon_scheduler_requests.transitions import ActiveRequest

# module being tested:
from concert_simple_scheduler.resource_pool import *

# some definitions for testing
RQ_UUID = uuid.UUID('01234567-89ab-cdef-0123-456789abcdef')
TEST_UUID = unique_id.fromURL('package://concert_simple_scheduler/test_uuid')
DIFF_UUID = unique_id.fromURL('package://concert_simple_scheduler/diff_uuid')

EXAMPLE_RAPP = 'tests/example_rapp'
TELEOP_RAPP = 'rocon_apps/teleop'
TEST_RAPPS = set((TELEOP_RAPP, EXAMPLE_RAPP))

TEST_STATUS = CurrentStatus(
    platform_info='rocon:///linux/precise/ros/segbot/roberto',
    rapps=[EXAMPLE_RAPP])
TEST_RESOURCE = Resource(
    platform_info='rocon:///linux/precise/ros/segbot/roberto',
    name=EXAMPLE_RAPP)
TEST_RESOURCE_NAME = 'rocon:///linux/precise/ros/segbot/roberto'
TEST_RESOURCE_STRING = (
    """rocon:///linux/precise/ros/segbot/roberto, status: 0
  owner: None
  rapps:
    """ + EXAMPLE_RAPP)

# this Resource has an old-format platform_info string:
TEST_ANOTHER = Resource(
    platform_info='linux.precise.ros.segbot.marvin',
    name=EXAMPLE_RAPP)
TEST_ANOTHER_NAME = 'rocon:///linux/precise/ros/segbot/marvin'
TEST_ANOTHER_STRING = (
    """rocon:///linux/precise/ros/segbot/marvin, status: 0
  owner: None
  rapps:
    """ + EXAMPLE_RAPP)

ANY_NAME = 'rocon:///linux/precise/ros/turtlebot/\.*'
NOT_TURTLEBOT_NAME = 'rocon:///linux/precise/ros/pr2/farnsworth'
MARVIN_NAME = 'rocon:///linux/precise/ros/turtlebot/marvin'
ROBERTO_NAME = 'rocon:///linux/precise/ros/turtlebot/roberto'
MARVIN = CurrentStatus(platform_info=MARVIN_NAME, rapps=TEST_RAPPS)
ROBERTO = CurrentStatus(platform_info=ROBERTO_NAME, rapps=TEST_RAPPS)

SINGLETON_POOL = KnownResources(resources=[ROBERTO])
DOUBLETON_POOL = KnownResources(resources=[MARVIN, ROBERTO])

# some useful Resource and Request messages
ANY_RESOURCE = Resource(name=TELEOP_RAPP, platform_info=ANY_NAME)
ANY_REQUEST = ActiveRequest(Request(
    id=unique_id.toMsg(RQ_UUID),
    resources=[ANY_RESOURCE]))
MARVIN_RESOURCE = Resource(name=TELEOP_RAPP, platform_info=MARVIN_NAME)
ROBERTO_RESOURCE = Resource(name=TELEOP_RAPP, platform_info=ROBERTO_NAME)
ROBERTO_REQUEST = ActiveRequest(Request(
    id=unique_id.toMsg(RQ_UUID),
    resources=[ROBERTO_RESOURCE]))
NOT_TURTLEBOT_RESOURCE = Resource(
    name=TELEOP_RAPP,
    platform_info=NOT_TURTLEBOT_NAME)
NOT_TURTLEBOT_REQUEST = ActiveRequest(Request(
    id=unique_id.toMsg(RQ_UUID),
    resources=[NOT_TURTLEBOT_RESOURCE]))


class TestResourcePool(unittest.TestCase):
    """Unit tests for simple scheduler resource pool class.

    These tests do not require a running ROS core.
    """

    ####################
    # resource pool tests
    ####################

    def test_allocate_permutation_two_resources(self):
        # Request a regexp allocation followed by an exact allocation.
        # Initially the exact resource gets assigned to the regexp, so
        # the second part of the request fails.  The allocator must
        # try the other permutation for it to succeed.
        pool = ResourcePool(KnownResources(resources=[
                    CurrentStatus(platform_info=MARVIN_NAME,
                                  rapps={TELEOP_RAPP, EXAMPLE_RAPP}),
                    CurrentStatus(platform_info=ROBERTO_NAME,
                                  rapps={TELEOP_RAPP})]))
        rq = ActiveRequest(Request(
                id=unique_id.toMsg(RQ_UUID),
                resources=[Resource(name=TELEOP_RAPP,
                                    platform_info=ANY_NAME),
                           Resource(name=EXAMPLE_RAPP,
                                    platform_info=MARVIN_NAME)]))
        alloc = pool.allocate(rq)
        self.assertTrue(alloc)
        self.assertEqual(len(alloc), 2)
        self.assertEqual(pool[MARVIN_NAME].status, CurrentStatus.ALLOCATED)
        self.assertEqual(pool[MARVIN_NAME].owner, RQ_UUID)
        self.assertEqual(pool[ROBERTO_NAME].status, CurrentStatus.ALLOCATED)
        self.assertEqual(pool[ROBERTO_NAME].owner, RQ_UUID)
        self.assertEqual(alloc[0], Resource(name=TELEOP_RAPP,
                                            platform_info=ROBERTO_NAME))
        self.assertEqual(alloc[1], Resource(name=EXAMPLE_RAPP,
                                            platform_info=MARVIN_NAME))

    def test_empty_constructor(self):
        rp0 = ResourcePool()
        self.assertIsNotNone(rp0)
        self.assertEqual(len(rp0), 0)
        self.assertNotIn(MARVIN_NAME, rp0)

    def test_exact_resource_allocation(self):
        rp2 = ResourcePool(DOUBLETON_POOL)
        self.assertEqual(len(rp2), 2)
        res = copy.deepcopy(ROBERTO_RESOURCE)
        subset = rp2._match_subset(res)
        self.assertIn(ROBERTO_NAME, subset)
        self.assertEqual(subset, set([ROBERTO_NAME]))
        self.assertEqual(rp2._match_list([ROBERTO_RESOURCE]),
                         [set([ROBERTO_NAME])])
        rq = copy.deepcopy(ROBERTO_REQUEST)
        alloc = rp2.allocate(rq)
        self.assertTrue(alloc)
        self.assertEqual(alloc[0], ROBERTO_RESOURCE)
        self.assertEqual(rp2[ROBERTO_NAME].status, CurrentStatus.ALLOCATED)
        self.assertEqual(rp2[ROBERTO_NAME].owner, RQ_UUID)

    def test_get_method(self):
        rp2 = ResourcePool(DOUBLETON_POOL)
        self.assertEqual(rp2.get(ROBERTO_NAME), PoolResource(ROBERTO))
        self.assertEqual(rp2.get(MARVIN_NAME, 3.14), PoolResource(MARVIN))
        self.assertIsNone(rp2.get(ANY_NAME))
        self.assertEqual(rp2.get(ANY_NAME, 3.14), 3.14)

    def test_insufficient_resources(self):
        # ask for two when there's only one
        pool = ResourcePool(KnownResources(resources=[ROBERTO]))
        rq1 = ActiveRequest(Request(
                id=unique_id.toMsg(RQ_UUID),
                resources=[ANY_RESOURCE, ANY_RESOURCE]))
        alloc1 = pool.allocate(rq1)
        self.assertFalse(alloc1)
        self.assertEqual(pool[ROBERTO_NAME].status, CurrentStatus.AVAILABLE)
        self.assertIsNone(pool[ROBERTO_NAME].owner)

    def test_match_failures(self):
        rp1 = ResourcePool(SINGLETON_POOL)
        res = Resource(
            name=TELEOP_RAPP,
            platform_info=NOT_TURTLEBOT_NAME)
        subset = rp1._match_subset(res)
        self.assertEqual(len(subset), 0)
        self.assertNotIn(MARVIN_NAME, subset)
        self.assertNotIn(ROBERTO_NAME, subset)
        self.assertEqual(subset, set())
        matches = rp1._match_list([NOT_TURTLEBOT_RESOURCE])
        self.assertEqual(matches, [])
        self.assertFalse(matches)
        rq = copy.deepcopy(NOT_TURTLEBOT_REQUEST)
        alloc = rp1.allocate(rq)
        self.assertFalse(alloc)

    def test_matching_allocation_one_resource(self):
        rp1 = ResourcePool(SINGLETON_POOL)
        self.assertEqual(len(rp1), 1)
        res = Resource(
            name=TELEOP_RAPP,
            platform_info=ANY_NAME)
        subset = rp1._match_subset(res)
        self.assertNotIn(MARVIN_NAME, subset)
        self.assertIn(ROBERTO_NAME, subset)
        self.assertEqual(subset, set([ROBERTO_NAME]))
        self.assertEqual(rp1._match_list([ROBERTO_RESOURCE]),
                         [set([ROBERTO_NAME])])
        rq = copy.deepcopy(ANY_REQUEST)
        alloc = rp1.allocate(rq)
        self.assertTrue(alloc)
        self.assertEqual(alloc[0], ROBERTO_RESOURCE)
        self.assertEqual(rp1[ROBERTO_NAME].status, CurrentStatus.ALLOCATED)
        self.assertEqual(rp1[ROBERTO_NAME].owner, RQ_UUID)

    def test_matching_allocation_two_resources(self):
        rp2 = ResourcePool(DOUBLETON_POOL)
        self.assertEqual(len(rp2), 2)
        res = Resource(
            name=TELEOP_RAPP,
            platform_info=ANY_NAME)
        subset = rp2._match_subset(res)
        self.assertIn(MARVIN_NAME, subset)
        self.assertIn(ROBERTO_NAME, subset)
        self.assertEqual(subset, set([MARVIN_NAME, ROBERTO_NAME]))
        self.assertEqual(rp2._match_list([ROBERTO_RESOURCE]),
                         [set([ROBERTO_NAME])])
        rq = copy.deepcopy(ANY_REQUEST)
        alloc = rp2.allocate(rq)
        self.assertTrue(alloc)
        if alloc[0] == MARVIN_RESOURCE:
            self.assertEqual(rp2[MARVIN_NAME].status,
                             CurrentStatus.ALLOCATED)
            self.assertEqual(rp2[MARVIN_NAME].owner, RQ_UUID)
        elif alloc[0] == ROBERTO_RESOURCE:
            self.assertEqual(rp2[ROBERTO_NAME].status,
                             CurrentStatus.ALLOCATED)
            self.assertEqual(rp2[ROBERTO_NAME].owner, RQ_UUID)
        else:
            self.fail('allocation failed to yield any expected result')

    def test_one_resource_constructor(self):
        rp1 = ResourcePool(SINGLETON_POOL)
        self.assertEqual(len(rp1), 1)
        self.assertIn(ROBERTO_NAME, rp1)
        self.assertNotIn(MARVIN_NAME, rp1)
        #self.assertMultiLineEqual(str(rp1), str(SINGLETON_POOL))

    def test_release_one_resource(self):
        rp2 = ResourcePool(DOUBLETON_POOL)
        self.assertEqual(len(rp2), 2)
        self.assertEqual(rp2[ROBERTO_NAME].status, CurrentStatus.AVAILABLE)

        rq = copy.deepcopy(ROBERTO_REQUEST)
        alloc = rp2.allocate(rq)
        self.assertTrue(alloc)
        rq.grant(alloc)
        self.assertEqual(rp2[ROBERTO_NAME].status, CurrentStatus.ALLOCATED)
        self.assertEqual(rp2[ROBERTO_NAME].owner, RQ_UUID)

        rp2.release_request(rq)
        self.assertEqual(rp2[ROBERTO_NAME].status, CurrentStatus.AVAILABLE)
        self.assertEqual(rp2[ROBERTO_NAME].owner, None)

    def test_release_one_resource_list(self):
        rp2 = ResourcePool(DOUBLETON_POOL)
        self.assertEqual(rp2[ROBERTO_NAME].status, CurrentStatus.AVAILABLE)

        rq = copy.deepcopy(ROBERTO_REQUEST)
        alloc = rp2.allocate(rq)
        self.assertTrue(alloc)
        self.assertEqual(rp2[ROBERTO_NAME].status, CurrentStatus.ALLOCATED)
        self.assertEqual(rp2[ROBERTO_NAME].owner, RQ_UUID)

        rp2.release_resources(alloc)
        self.assertEqual(rp2[ROBERTO_NAME].status, CurrentStatus.AVAILABLE)
        self.assertEqual(rp2[ROBERTO_NAME].owner, None)

    def test_two_resource_constructor(self):
        rp2 = ResourcePool(DOUBLETON_POOL)
        self.assertEqual(len(rp2), 2)
        self.assertIn(ROBERTO_NAME, rp2)
        self.assertIn(MARVIN_NAME, rp2)
        #self.assertMultiLineEqual(str(rp2), str(DOUBLETON_POOL))


class TestPoolResource(unittest.TestCase):
    """Unit tests for pools resource class.

    These tests do not require a running ROS core.
    """

    #####################
    # pool resource tests
    #####################

    def test_constructor(self):
        res1 = PoolResource(TEST_ANOTHER)
        self.assertIsNotNone(res1)
        self.assertEqual(res1.platform_info, TEST_ANOTHER_NAME)
        self.assertMultiLineEqual(str(res1), TEST_ANOTHER_STRING)

        res2 = PoolResource(TEST_STATUS)
        self.assertEqual(res2.platform_info, TEST_RESOURCE_NAME)
        self.assertMultiLineEqual(str(res2), TEST_RESOURCE_STRING)
        self.assertNotEqual(str(res2), TEST_ANOTHER_STRING)

    def test_allocate(self):
        res1 = PoolResource(Resource(
            platform_info='rocon:///linux/precise/ros/segbot/roberto',
            name=EXAMPLE_RAPP))
        self.assertEqual(res1.status, CurrentStatus.AVAILABLE)
        self.assertEqual(res1.owner, None)
        res1.allocate(TEST_UUID)
        self.assertEqual(res1.status, CurrentStatus.ALLOCATED)
        self.assertEqual(res1.owner, TEST_UUID)
        self.assertRaises(ResourceNotAvailableError, res1.allocate, DIFF_UUID)
        self.assertRaises(ResourceNotAvailableError, res1.allocate, TEST_UUID)

    def test_equality(self):
        res1 = PoolResource(Resource(
            platform_info='linux.precise.ros.segbot.roberto',
            name='rocon_apps/teleop'))
        self.assertEqual(res1, PoolResource(Resource(
            platform_info='linux.precise.ros.segbot.roberto',
            name='rocon_apps/teleop')))

        # different platform_info
        self.assertNotEqual(res1, PoolResource(TEST_ANOTHER))

        # different rapp name
        self.assertNotEqual(res1, PoolResource(Resource(
            platform_info='linux.precise.ros.segbot.roberto',
            name='other_package/teleop')))

        # different owner
        res2 = PoolResource(Resource(
            platform_info='rocon:///linux/precise/ros/segbot/roberto',
            name='rocon_apps/teleop'))
        res2.allocate(TEST_UUID)
        self.assertNotEqual(res1, res2)

        # different status
        res3 = PoolResource(Resource(
            platform_info='rocon:///linux/precise/ros/segbot/roberto',
            name='rocon_apps/teleop'))
        res3.status = CurrentStatus.MISSING
        self.assertEqual(res1.owner, res3.owner)
        self.assertNotEqual(res1.status, res3.status)
        self.assertNotEqual(res1, res3)

    def test_match(self):
        res1 = PoolResource(TEST_STATUS)
        self.assertTrue(res1.match(Resource(
            name=EXAMPLE_RAPP,
            platform_info=r'rocon:///linux/precise/ros/segbot/.*')))
        self.assertTrue(res1.match(Resource(
            name=EXAMPLE_RAPP,
            platform_info='linux.precise.ros.segbot.*')))
        self.assertTrue(res1.match(TEST_RESOURCE))
        self.assertFalse(res1.match(Resource(
            name=EXAMPLE_RAPP,
            platform_info='linux.precise.ros.segbot.marvin')))
        self.assertTrue(res1.match(Resource(
            name=EXAMPLE_RAPP,
            platform_info=r'rocon:///linux/.*/ros/(segbot|turtlebot)/.*')))

        # different rapps:
        diff_rapp = Resource(
            name='different/rapp',
            platform_info=r'rocon:///linux/precise/ros/segbot/.*')
        self.assertFalse(res1.match(diff_rapp))
        res1.rapps.add('different/rapp')
        self.assertTrue(res1.match(diff_rapp))
        res1.rapps.remove('different/rapp')
        self.assertFalse(res1.match(diff_rapp))

    def test_match_pattern(self):
        res1 = PoolResource(TEST_RESOURCE)
        self.assertTrue(res1.match_pattern(
            'rocon:///linux/precise/ros/segbot/.*', EXAMPLE_RAPP))
        self.assertFalse(res1.match_pattern(
            'rocon:///linux/precise/ros/marvin/.*', EXAMPLE_RAPP))
        self.assertTrue(res1.match_pattern(
            'rocon:///linux/.*/ros/(segbot|turtlebot)/.*', EXAMPLE_RAPP))

        # different rapps:
        self.assertFalse(res1.match_pattern(
            'rocon:///linux/precise/ros/segbot/.*', 'different/rapp'))
        res1.rapps.add('different/rapp')
        self.assertTrue(res1.match_pattern(
            'rocon:///linux/precise/ros/segbot/.*', 'different/rapp'))
        res1.rapps.remove('different/rapp')
        self.assertFalse(res1.match_pattern(
            'rocon:///linux/precise/ros/segbot/.*', 'different/rapp'))

    def test_release(self):
        res1 = PoolResource(Resource(
            platform_info='rocon:///linux/precise/ros/segbot/roberto',
            name=EXAMPLE_RAPP))
        self.assertEqual(res1.status, CurrentStatus.AVAILABLE)
        self.assertEqual(res1.owner, None)
        res1.allocate(TEST_UUID)
        self.assertEqual(res1.status, CurrentStatus.ALLOCATED)
        self.assertEqual(res1.owner, TEST_UUID)
        self.assertRaises(ResourceNotOwnedError, res1.release, DIFF_UUID)
        self.assertEqual(res1.status, CurrentStatus.ALLOCATED)
        res1.release(TEST_UUID)
        self.assertEqual(res1.status, CurrentStatus.AVAILABLE)

        res2 = PoolResource(Resource(
            platform_info='rocon:///linux/precise/ros/segbot/roberto',
            name=EXAMPLE_RAPP))
        res2.allocate(TEST_UUID)
        self.assertEqual(res2.status, CurrentStatus.ALLOCATED)
        res2.status = CurrentStatus.MISSING    # resource now missing
        res2.release(TEST_UUID)
        self.assertEqual(res2.status, CurrentStatus.MISSING)

        res3 = PoolResource(Resource(
            platform_info='rocon:///linux/precise/ros/segbot/roberto',
            name=EXAMPLE_RAPP))
        res3.allocate(TEST_UUID)
        res3.release()
        self.assertEqual(res3.status, CurrentStatus.AVAILABLE)

        res4 = PoolResource(Resource(
            platform_info='rocon:///linux/precise/ros/segbot/roberto',
            name=EXAMPLE_RAPP))
        res4.allocate(TEST_UUID)
        res4.status = CurrentStatus.MISSING    # resource now missing
        res4.release()
        self.assertEqual(res4.status, CurrentStatus.MISSING)

    def test_rocon_name(self):
        self.assertEqual(rocon_name(TEST_RESOURCE_NAME), TEST_RESOURCE_NAME)
        self.assertEqual(rocon_name(TEST_ANOTHER_NAME), TEST_ANOTHER_NAME)
        self.assertNotEqual(rocon_name(TEST_ANOTHER_NAME), TEST_RESOURCE_NAME)


class TestResourceSet(unittest.TestCase):
    """Unit tests for resource set operations.

    These tests do not require a running ROS core.
    """

    ####################
    # resource set tests -- deprecated, but run them for now
    ####################

    def test_empty_resource_set(self):
        res_set = ResourceSet()
        self.assertIsNotNone(res_set)
        self.assertEqual(len(res_set), 0)
        self.assertFalse(PoolResource(TEST_STATUS) in res_set)
        self.assertTrue('arbitrary name' not in res_set)
        self.assertTrue(TEST_RESOURCE_NAME not in res_set)
        self.assertIsNone(res_set.get(TEST_RESOURCE_NAME))
        self.assertEqual(res_set.get(TEST_RESOURCE_NAME, 3.14), 3.14)

        # Test equality for empty res_sets.
        self.assertTrue(res_set == ResourceSet([]))
        self.assertFalse(not res_set == ResourceSet([]))
        self.assertTrue((res_set != ResourceSet([TEST_RESOURCE])))
        self.assertFalse(res_set == ResourceSet([TEST_STATUS]))
        self.assertEqual(str(res_set), 'ROCON resource set:')

    def test_one_resource_set(self):
        res_set = ResourceSet(KnownResources(resources=[TEST_RESOURCE]))
        self.assertEqual(len(res_set), 1)
        self.assertTrue(PoolResource(TEST_RESOURCE) in res_set)
        self.assertTrue(TEST_RESOURCE_NAME in res_set)
        self.assertEqual(res_set.get(TEST_RESOURCE_NAME),
                         res_set[TEST_RESOURCE_NAME])
        self.assertEqual(res_set.get(TEST_RESOURCE_NAME, 3.14),
                         res_set[TEST_RESOURCE_NAME])
        self.assertNotIn('', res_set)
        self.assertIn(TEST_RESOURCE_NAME, res_set)
        self.assertNotIn(TEST_ANOTHER_NAME, res_set)

        # Test equality for non-empty res_sets.
        self.assertNotEqual(res_set, ResourceSet([]))
        self.assertEqual(res_set, ResourceSet([TEST_RESOURCE]))
        self.assertMultiLineEqual(
            str(res_set), 'ROCON resource set:\n  ' + TEST_RESOURCE_STRING)
        self.assertNotEqual(res_set, ResourceSet([Resource(
            platform_info='rocon:///linux/precise/ros/segbot/roberto',
            name='other_package/teleop')]))

    def test_two_resource_set(self):
        res_set = ResourceSet()
        self.assertEqual(len(res_set), 0)
        self.assertNotIn(TEST_RESOURCE_NAME, res_set)
        self.assertNotIn(TEST_ANOTHER_NAME, res_set)

        res_set[TEST_RESOURCE_NAME] = PoolResource(TEST_STATUS)
        self.assertEqual(len(res_set), 1)
        self.assertIn(TEST_RESOURCE_NAME, res_set)
        self.assertNotIn(TEST_ANOTHER_NAME, res_set)

        res_set[TEST_ANOTHER_NAME] = TEST_ANOTHER
        self.assertEqual(len(res_set), 2)
        self.assertIn(TEST_RESOURCE_NAME, res_set)
        self.assertIn(TEST_ANOTHER_NAME, res_set)

        # Test equality for res_set.
        self.assertFalse(res_set == ResourceSet([]))
        self.assertTrue(not res_set == ResourceSet([]))
        self.assertFalse(res_set != ResourceSet([TEST_RESOURCE, TEST_ANOTHER]))
        self.assertTrue(res_set == ResourceSet([TEST_RESOURCE, TEST_ANOTHER]))

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('concert_simple_scheduler',
                    'test_resource_pool',
                    TestResourcePool)
    rosunit.unitrun('concert_simple_scheduler',
                    'test_pool_resource',
                    TestPoolResource)
    rosunit.unitrun('concert_simple_scheduler',
                    'test_resource_sets',
                    TestResourceSet)
