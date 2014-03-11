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
TEST_RAPPS = [TELEOP_RAPP, EXAMPLE_RAPP]

TEST_STATUS = CurrentStatus(uri='rocon:/segbot/roberto', rapps=[EXAMPLE_RAPP])
TEST_RESOURCE = Resource(uri='rocon:/segbot/roberto', rapp=EXAMPLE_RAPP)
TEST_RESOURCE_NAME = 'rocon:/segbot/roberto'
TEST_RESOURCE_STRING = (
    """rocon:/segbot/roberto, status: 0
  owner: None
  rapps:
    """ + EXAMPLE_RAPP)

# this Resource has an old-format platform_info string:
TEST_ANOTHER = Resource(uri='segbot.marvin', rapp=EXAMPLE_RAPP)
TEST_ANOTHER_NAME = 'rocon:/segbot/marvin'
TEST_ANOTHER_STRING = (
    """rocon:/segbot/marvin, status: 0
  owner: None
  rapps:
    """ + EXAMPLE_RAPP)

ANY_NAME = 'rocon:/turtlebot'
NOT_TURTLEBOT_NAME = 'rocon:/pr2/farnsworth'
DUDE1_NAME = 'rocon:/turtlebot/dude1'
DUDE2_NAME = 'rocon:/turtlebot/dude2'
DUDE3_NAME = 'rocon:/turtlebot/dude3'
DUDE4_NAME = 'rocon:/turtlebot/dude4'
MARVIN_NAME = 'rocon:/turtlebot/marvin'
ROBERTO_NAME = 'rocon:/turtlebot/roberto'
MARVIN = CurrentStatus(uri=MARVIN_NAME, rapps=TEST_RAPPS)
ROBERTO = CurrentStatus(uri=ROBERTO_NAME, rapps=TEST_RAPPS)

SINGLETON_POOL = KnownResources(resources=[ROBERTO])
DOUBLETON_POOL = KnownResources(resources=[MARVIN, ROBERTO])

# some useful Resource and Request messages
ANY_RESOURCE = Resource(rapp=TELEOP_RAPP, uri=ANY_NAME)
ANY_REQUEST = ActiveRequest(Request(
    id=unique_id.toMsg(RQ_UUID),
    resources=[ANY_RESOURCE]))
MARVIN_RESOURCE = Resource(rapp=TELEOP_RAPP, uri=MARVIN_NAME)
ROBERTO_RESOURCE = Resource(rapp=TELEOP_RAPP, uri=ROBERTO_NAME)
ROBERTO_REQUEST = ActiveRequest(Request(
    id=unique_id.toMsg(RQ_UUID),
    resources=[ROBERTO_RESOURCE]))
NOT_TURTLEBOT_RESOURCE = Resource(rapp=TELEOP_RAPP, uri=NOT_TURTLEBOT_NAME)
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

    @unittest.skip('allocate() is succeeding, and it should not')
    def test_allocate_four_resources_failure(self):
        """ Similar to test_allocate_permutation_two_resources(), but
        here there are more permutations, so the allocator gives up
        after the initial failure.
        """
        pool = ResourcePool(KnownResources(resources=[
                    CurrentStatus(uri=DUDE1_NAME, rapps={TELEOP_RAPP}),
                    CurrentStatus(uri=DUDE2_NAME, rapps={TELEOP_RAPP}),
                    CurrentStatus(uri=DUDE3_NAME, rapps={TELEOP_RAPP}),
                    CurrentStatus(uri=DUDE4_NAME,
                                  rapps={TELEOP_RAPP, EXAMPLE_RAPP})]))
        rq = ActiveRequest(Request(
                id=unique_id.toMsg(RQ_UUID),
                resources=[Resource(rapp=TELEOP_RAPP, uri=ANY_NAME),
                           Resource(rapp=EXAMPLE_RAPP, uri=DUDE4_NAME),
                           Resource(rapp=TELEOP_RAPP, uri=DUDE2_NAME),
                           Resource(rapp=TELEOP_RAPP, uri=DUDE3_NAME)]))
        self.assertRaises(InvalidRequestError, pool.allocate, rq)
        for name in [DUDE1_NAME, DUDE2_NAME, DUDE3_NAME, DUDE4_NAME]:
            self.assertEqual(pool[name].status, CurrentStatus.AVAILABLE)
            self.assertIsNone(pool[name].owner)

    def test_allocate_four_resources_success(self):
        """ Similar to test_allocate_four_resources_failure(), but the
        order of the request is different, so the allocator succeeds.
        """
        pool = ResourcePool(KnownResources(resources=[
                    CurrentStatus(uri=DUDE1_NAME, rapps={TELEOP_RAPP}),
                    CurrentStatus(uri=DUDE2_NAME, rapps={TELEOP_RAPP}),
                    CurrentStatus(uri=DUDE3_NAME, rapps={TELEOP_RAPP}),
                    CurrentStatus(uri=DUDE4_NAME,
                                  rapps={TELEOP_RAPP, EXAMPLE_RAPP})]))
        rq = ActiveRequest(Request(
                id=unique_id.toMsg(RQ_UUID),
                resources=[Resource(rapp=EXAMPLE_RAPP, uri=DUDE4_NAME),
                           Resource(rapp=TELEOP_RAPP, uri=DUDE2_NAME),
                           Resource(rapp=TELEOP_RAPP, uri=DUDE3_NAME),
                           Resource(rapp=TELEOP_RAPP, uri=ANY_NAME)]))
        alloc = pool.allocate(rq)
        self.assertTrue(alloc)
        bot_names = [DUDE4_NAME, DUDE2_NAME, DUDE3_NAME, DUDE1_NAME]
        for name, i in zip(bot_names, range(4)):
            self.assertEqual(pool[name].status, CurrentStatus.ALLOCATED)
            self.assertEqual(alloc[i].uri, name)

    def test_allocate_permutation_two_resources(self):
        """ Request a regexp allocation followed by an exact
        allocation.  Initially the exact resource gets assigned to the
        regexp, so the second part of the request fails.  The
        allocator must try the other permutation for it to succeed.
        """
        pool = ResourcePool(KnownResources(resources=[
                    CurrentStatus(uri=MARVIN_NAME,
                                  rapps={TELEOP_RAPP, EXAMPLE_RAPP}),
                    CurrentStatus(uri=ROBERTO_NAME,
                                  rapps={TELEOP_RAPP})]))
        rq = ActiveRequest(Request(
                id=unique_id.toMsg(RQ_UUID),
                resources=[Resource(rapp=TELEOP_RAPP, uri=ANY_NAME),
                           Resource(rapp=EXAMPLE_RAPP, uri=MARVIN_NAME)]))
        alloc = pool.allocate(rq)
        self.assertTrue(alloc)
        self.assertEqual(len(alloc), 2)
        self.assertEqual(pool[MARVIN_NAME].status, CurrentStatus.ALLOCATED)
        self.assertEqual(pool[MARVIN_NAME].owner, RQ_UUID)
        self.assertEqual(pool[ROBERTO_NAME].status, CurrentStatus.ALLOCATED)
        self.assertEqual(pool[ROBERTO_NAME].owner, RQ_UUID)
        self.assertEqual(alloc[0],
                         Resource(rapp=TELEOP_RAPP, uri=ROBERTO_NAME))
        self.assertEqual(alloc[1],
                         Resource(rapp=EXAMPLE_RAPP, uri=MARVIN_NAME))

    def test_empty_constructor(self):
        pool = ResourcePool()
        self.assertIsNotNone(pool)
        self.assertEqual(len(pool), 0)
        self.assertNotIn(MARVIN_NAME, pool)
        self.assertMultiLineEqual(str(pool), 'pool contents:')
        self.assertTrue(pool.changed)
        self.assertEqual(pool.known_resources(), KnownResources())
        self.assertFalse(pool.changed)

    def test_exact_resource_allocation(self):
        pool = ResourcePool(DOUBLETON_POOL)
        self.assertEqual(len(pool), 2)
        self.assertTrue(pool.changed)
        self.assertEqual(pool.known_resources(), DOUBLETON_POOL)
        self.assertFalse(pool.changed)

        res = copy.deepcopy(ROBERTO_RESOURCE)
        subset = pool._match_subset(res, {CurrentStatus.AVAILABLE})
        self.assertIn(ROBERTO_NAME, subset)
        self.assertEqual(subset, set([ROBERTO_NAME]))
        self.assertEqual(pool.match_list([ROBERTO_RESOURCE],
                                        {CurrentStatus.AVAILABLE}),
                         [set([ROBERTO_NAME])])

        self.assertFalse(pool.changed)
        rq = copy.deepcopy(ROBERTO_REQUEST)
        alloc = pool.allocate(rq)
        self.assertTrue(pool.changed)
        self.assertTrue(alloc)
        self.assertEqual(alloc[0], ROBERTO_RESOURCE)
        self.assertEqual(pool[ROBERTO_NAME].status, CurrentStatus.ALLOCATED)
        self.assertEqual(pool[ROBERTO_NAME].owner, RQ_UUID)

    def test_get_method(self):
        pool = ResourcePool(DOUBLETON_POOL)
        self.assertEqual(pool.get(ROBERTO_NAME), PoolResource(ROBERTO))
        self.assertEqual(pool.get(MARVIN_NAME, 3.14), PoolResource(MARVIN))
        self.assertIsNone(pool.get(ANY_NAME))
        self.assertEqual(pool.get(ANY_NAME, 3.14), 3.14)

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
        pool = ResourcePool(SINGLETON_POOL)
        res = Resource(rapp=TELEOP_RAPP, uri=NOT_TURTLEBOT_NAME)
        subset = pool._match_subset(res, {CurrentStatus.AVAILABLE})
        self.assertEqual(len(subset), 0)
        self.assertNotIn(MARVIN_NAME, subset)
        self.assertNotIn(ROBERTO_NAME, subset)
        self.assertEqual(subset, set())
        # test null resources list:
        match_null = pool.match_list([], {CurrentStatus.AVAILABLE})
        self.assertEqual(match_null, [])
        self.assertFalse(match_null)
        # test not matching resource:
        matches = pool.match_list([NOT_TURTLEBOT_RESOURCE],
                                 {CurrentStatus.AVAILABLE})
        self.assertEqual(matches, [])
        self.assertFalse(matches)
        rq = copy.deepcopy(NOT_TURTLEBOT_REQUEST)
        alloc = pool.allocate(rq)
        self.assertFalse(alloc)

    def test_matching_allocation_one_resource(self):
        pool = ResourcePool(SINGLETON_POOL)
        self.assertEqual(len(pool), 1)
        self.assertEqual(pool.known_resources(), SINGLETON_POOL)
        self.assertFalse(pool.changed)
        res = Resource(rapp=TELEOP_RAPP, uri=ANY_NAME)
        subset = pool._match_subset(res, {CurrentStatus.AVAILABLE})
        self.assertNotIn(MARVIN_NAME, subset)
        self.assertIn(ROBERTO_NAME, subset)
        self.assertEqual(subset, set([ROBERTO_NAME]))
        self.assertEqual(pool.match_list([ROBERTO_RESOURCE],
                                        {CurrentStatus.AVAILABLE}),
                         [set([ROBERTO_NAME])])
        rq = copy.deepcopy(ANY_REQUEST)
        alloc = pool.allocate(rq)
        self.assertTrue(alloc)
        self.assertTrue(pool.changed)
        self.assertEqual(alloc[0], ROBERTO_RESOURCE)
        self.assertEqual(pool[ROBERTO_NAME].status, CurrentStatus.ALLOCATED)
        self.assertEqual(pool[ROBERTO_NAME].owner, RQ_UUID)
        self.assertTrue(pool.changed)
        self.assertEqual(
            pool.known_resources(), 
            KnownResources(resources=[
                    CurrentStatus(uri=ROBERTO_NAME, rapps=TEST_RAPPS,
                                  status=CurrentStatus.ALLOCATED,
                                  owner=RQ_UUID)]))
        self.assertFalse(pool.changed)

    def test_matching_allocation_two_resources(self):
        pool = ResourcePool(DOUBLETON_POOL)
        self.assertEqual(len(pool), 2)
        res = Resource(rapp=TELEOP_RAPP, uri=ANY_NAME)
        subset = pool._match_subset(res, {CurrentStatus.AVAILABLE})
        self.assertIn(MARVIN_NAME, subset)
        self.assertIn(ROBERTO_NAME, subset)
        self.assertEqual(subset, set([MARVIN_NAME, ROBERTO_NAME]))
        self.assertEqual(pool.match_list([ROBERTO_RESOURCE],
                                        {CurrentStatus.AVAILABLE}),
                         [set([ROBERTO_NAME])])
        rq = copy.deepcopy(ANY_REQUEST)
        alloc = pool.allocate(rq)
        self.assertTrue(alloc)
        if alloc[0] == MARVIN_RESOURCE:
            self.assertEqual(pool[MARVIN_NAME].status,
                             CurrentStatus.ALLOCATED)
            self.assertEqual(pool[MARVIN_NAME].owner, RQ_UUID)
        elif alloc[0] == ROBERTO_RESOURCE:
            self.assertEqual(pool[ROBERTO_NAME].status,
                             CurrentStatus.ALLOCATED)
            self.assertEqual(pool[ROBERTO_NAME].owner, RQ_UUID)
        else:
            self.fail('allocation failed to yield any expected result')

    def test_one_resource_constructor(self):
        pool = ResourcePool(SINGLETON_POOL)
        self.assertEqual(len(pool), 1)
        self.assertIn(ROBERTO_NAME, pool)
        self.assertNotIn(MARVIN_NAME, pool)
        self.assertMultiLineEqual(
            str(pool),
            'pool contents:\n  ' + str(PoolResource(ROBERTO)))
        self.assertTrue(pool.changed)
        self.assertEqual(pool.known_resources(), SINGLETON_POOL)
        self.assertFalse(pool.changed)

    def test_release_one_resource(self):
        pool = ResourcePool(DOUBLETON_POOL)
        self.assertEqual(len(pool), 2)
        self.assertEqual(pool[ROBERTO_NAME].status, CurrentStatus.AVAILABLE)
        self.assertTrue(pool.changed)
        self.assertEqual(pool.known_resources(), DOUBLETON_POOL)
        self.assertFalse(pool.changed)

        rq = copy.deepcopy(ROBERTO_REQUEST)
        alloc = pool.allocate(rq)
        self.assertTrue(alloc)
        rq.grant(alloc)
        self.assertEqual(pool[ROBERTO_NAME].status, CurrentStatus.ALLOCATED)
        self.assertEqual(pool[ROBERTO_NAME].owner, RQ_UUID)

        self.assertTrue(pool.changed)
        x = pool.known_resources()
        self.assertFalse(pool.changed)

        pool.release_request(rq)
        self.assertEqual(pool[ROBERTO_NAME].status, CurrentStatus.AVAILABLE)
        self.assertEqual(pool[ROBERTO_NAME].owner, None)

        self.assertTrue(pool.changed)
        self.assertEqual(pool.known_resources(), DOUBLETON_POOL)
        self.assertFalse(pool.changed)

    def test_release_one_resource_list(self):
        pool = ResourcePool(DOUBLETON_POOL)
        self.assertEqual(pool[ROBERTO_NAME].status, CurrentStatus.AVAILABLE)

        rq = copy.deepcopy(ROBERTO_REQUEST)
        alloc = pool.allocate(rq)
        self.assertTrue(alloc)
        self.assertEqual(pool[ROBERTO_NAME].status, CurrentStatus.ALLOCATED)
        self.assertEqual(pool[ROBERTO_NAME].owner, RQ_UUID)

        pool.release_resources(alloc)
        self.assertEqual(pool[ROBERTO_NAME].status, CurrentStatus.AVAILABLE)
        self.assertEqual(pool[ROBERTO_NAME].owner, None)

    def test_two_resource_constructor(self):
        pool = ResourcePool(DOUBLETON_POOL)
        self.assertEqual(len(pool), 2)
        self.assertIn(ROBERTO_NAME, pool)
        self.assertIn(MARVIN_NAME, pool)
        self.assertTrue(pool.changed)
        self.assertEqual(pool.known_resources(), DOUBLETON_POOL)
        self.assertFalse(pool.changed)


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
        self.assertEqual(res1.uri, TEST_ANOTHER_NAME)
        self.assertMultiLineEqual(str(res1), TEST_ANOTHER_STRING)

        res2 = PoolResource(TEST_STATUS)
        self.assertEqual(res2.uri, TEST_RESOURCE_NAME)
        self.assertMultiLineEqual(str(res2), TEST_RESOURCE_STRING)
        self.assertNotEqual(str(res2), TEST_ANOTHER_STRING)

    def test_allocate(self):
        res1 = PoolResource(Resource(
            uri='rocon:/segbot/roberto',
            rapp=EXAMPLE_RAPP))
        self.assertEqual(res1.status, CurrentStatus.AVAILABLE)
        self.assertEqual(res1.owner, None)
        res1.allocate(TEST_UUID)
        self.assertEqual(res1.status, CurrentStatus.ALLOCATED)
        self.assertEqual(res1.owner, TEST_UUID)
        self.assertRaises(ResourceNotAvailableError, res1.allocate, DIFF_UUID)
        self.assertRaises(ResourceNotAvailableError, res1.allocate, TEST_UUID)

    def test_equality(self):
        res1 = PoolResource(Resource(
            uri='linux.precise.ros.segbot.roberto',
            rapp='rocon_apps/teleop'))
        self.assertEqual(res1, PoolResource(Resource(
            uri='linux.precise.ros.segbot.roberto',
            rapp='rocon_apps/teleop')))

        # different uri
        self.assertNotEqual(res1, PoolResource(TEST_ANOTHER))

        # different rapp name
        self.assertNotEqual(res1, PoolResource(Resource(
            uri='linux.precise.ros.segbot.roberto',
            rapp='other_package/teleop')))

        # different owner
        res2 = PoolResource(Resource(
            uri='rocon:/segbot/roberto', rapp='rocon_apps/teleop'))
        res2.allocate(TEST_UUID)
        self.assertNotEqual(res1, res2)

        # different status
        res3 = PoolResource(Resource(
            uri='rocon:/segbot/roberto', rapp='rocon_apps/teleop'))
        res3.status = CurrentStatus.MISSING
        self.assertEqual(res1.owner, res3.owner)
        self.assertNotEqual(res1.status, res3.status)
        self.assertNotEqual(res1, res3)

    def test_match(self):
        res1 = PoolResource(TEST_STATUS)
        self.assertTrue(res1.match(Resource(
                    rapp=EXAMPLE_RAPP, uri=r'rocon:/segbot')))
        self.assertTrue(res1.match(Resource(rapp=EXAMPLE_RAPP, uri='segbot')))
        self.assertTrue(res1.match(Resource(
                    rapp=EXAMPLE_RAPP, uri='rocon:/segbot/roberto')))
        self.assertFalse(res1.match(Resource(
            rapp=EXAMPLE_RAPP, uri='linux.precise.ros.segbot.marvin')))
        self.assertTrue(res1.match(Resource(
            rapp=EXAMPLE_RAPP, uri=r'rocon:/(segbot|turtlebot)/')))

        # different rapps:
        diff_rapp = Resource(rapp='different/rapp', uri=r'rocon:/segbot')
        self.assertFalse(res1.match(diff_rapp))
        res1.rapps.add('different/rapp')
        self.assertTrue(res1.match(diff_rapp))
        res1.rapps.remove('different/rapp')
        self.assertFalse(res1.match(diff_rapp))

    def test_match_pattern(self):
        res1 = PoolResource(TEST_RESOURCE)
        self.assertTrue(res1.match_pattern('rocon:/segbot', EXAMPLE_RAPP))
        self.assertFalse(res1.match_pattern('rocon:/*/marvin', EXAMPLE_RAPP))
        self.assertTrue(res1.match_pattern(
            'rocon:/(segbot|turtlebot)/', EXAMPLE_RAPP))

        # different rapps:
        self.assertFalse(res1.match_pattern('rocon:/segbot', 'different/rapp'))
        res1.rapps.add('different/rapp')
        self.assertTrue(res1.match_pattern('rocon:/segbot', 'different/rapp'))
        res1.rapps.remove('different/rapp')
        self.assertFalse(res1.match_pattern('rocon:/segbot', 'different/rapp'))

    def test_release(self):
        res1 = PoolResource(Resource(
            uri='rocon:/segbot/roberto', rapp=EXAMPLE_RAPP))
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
            uri='rocon:/segbot/roberto', rapp=EXAMPLE_RAPP))
        res2.allocate(TEST_UUID)
        self.assertEqual(res2.status, CurrentStatus.ALLOCATED)
        res2.status = CurrentStatus.MISSING    # resource now missing
        res2.release(TEST_UUID)
        self.assertEqual(res2.status, CurrentStatus.MISSING)

        res3 = PoolResource(Resource(
            uri='rocon:/segbot/roberto', rapp=EXAMPLE_RAPP))
        res3.allocate(TEST_UUID)
        res3.release()
        self.assertEqual(res3.status, CurrentStatus.AVAILABLE)

        res4 = PoolResource(Resource(
            uri='rocon:/segbot/roberto', rapp=EXAMPLE_RAPP))
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
            uri='rocon:/segbot/roberto', rapp='other_package/teleop')]))

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
