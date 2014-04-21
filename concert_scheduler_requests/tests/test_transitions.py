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

# module being tested:
from concert_scheduler_requests.transitions import *

RQR_UUID = uuid.UUID('01234567-89ab-cdef-0123-456789abcdef')
TEST_UUID = uuid.UUID('01234567-89ab-cdef-fedc-ba9876543210')
DIFF_UUID = uuid.UUID('01234567-cdef-fedc-89ab-ba9876543210')
TEST_RAPP = 'test_rapp'
TEST_RESOURCE = Resource(rapp=TEST_RAPP, uri='rocon:/segbot/roberto')
TEST_WILDCARD = Resource(rapp=TEST_RAPP, uri='rocon:/segbot')


class TestTransitions(unittest.TestCase):
    """Unit tests for scheduler request state transitions.

    These tests do not require a running ROS core.
    """

    ####################
    # utility methods
    ####################
    def assert_invalid(self, request_type, old_status,
                       operation, exception, *args):
        """
        Assert that *request_type* with *old_status* rejects named
        *operation*, raising *exception*.
        """
        rq = request_type(Request(id=unique_id.toMsg(TEST_UUID),
                                  resources=[TEST_WILDCARD],
                                  status=old_status))
        op_method = getattr(rq, operation)
        self.assertRaises(exception, op_method, *args)

    def assert_valid(self, request_type, old_status,
                     operation, new_status, *args):
        """
        Assert that *request_type* with *old_status* accepts named
        *operation*, yielding *new_status*.

        :returns: request contents after the *operation*.
        """
        rq = request_type(Request(id=unique_id.toMsg(TEST_UUID),
                                  resources=[TEST_WILDCARD],
                                  status=old_status))
        getattr(rq, operation)(*args)
        self.assertEqual(rq.msg.status, new_status)
        return rq

    ####################
    # request tests
    ####################
    def test_constructor(self):
        msg1 = Request(id=unique_id.toMsg(TEST_UUID),
                       resources=[TEST_WILDCARD],
                       status=Request.NEW)
        rq1 = ResourceRequest(msg1)
        self.assertIsNotNone(rq1)
        self.assertEqual(str(rq1),
                         """id: 01234567-89ab-cdef-fedc-ba9876543210
    priority: 0
    resources: 
      rocon:/segbot#test_rapp
    status: 0""")
        self.assertEqual(rq1.msg.status, Request.NEW)
        self.assertEqual(rq1.msg.resources, [TEST_WILDCARD])
        self.assertEqual(rq1.uuid, TEST_UUID)

        rq2 = ResourceRequest(Request(id=unique_id.toMsg(DIFF_UUID),
                                      resources=[TEST_RESOURCE],
                                      status=Request.NEW))
        self.assertEqual(rq2.msg.status, Request.NEW)
        self.assertEqual(rq2.msg.resources, [TEST_RESOURCE])
        self.assertEqual(rq2.uuid, DIFF_UUID)
        self.assertEqual(str(rq2),
                         """id: 01234567-cdef-fedc-89ab-ba9876543210
    priority: 0
    resources: 
      rocon:/segbot/roberto#test_rapp
    status: 0""")

    def test_cancel(self):
        # should be valid in every state:
        self.assert_valid(ResourceRequest, Request.CANCELING,
                          'cancel', Request.CANCELING)
        self.assert_valid(ResourceRequest, Request.CLOSED,
                          'cancel', Request.CLOSED)
        self.assert_valid(ResourceRequest, Request.GRANTED,
                          'cancel', Request.CANCELING)
        self.assert_valid(ResourceRequest, Request.NEW,
                          'cancel', Request.CANCELING)
        self.assert_valid(ResourceRequest, Request.PREEMPTING,
                          'cancel', Request.CANCELING)
        self.assert_valid(ResourceRequest, Request.RESERVED,
                          'cancel', Request.CANCELING)
        self.assert_valid(ResourceRequest, Request.WAITING,
                          'cancel', Request.CANCELING)

    def test_close(self):
        self.assert_valid(ActiveRequest, Request.CANCELING,
                          'close', Request.CLOSED)
        self.assert_invalid(ActiveRequest, Request.CLOSED,
                            'close', TransitionError)
        self.assert_invalid(ActiveRequest, Request.GRANTED,
                            'close', TransitionError)
        self.assert_invalid(ActiveRequest, Request.NEW,
                            'close', TransitionError)
        self.assert_valid(ActiveRequest, Request.PREEMPTING,
                          'close', Request.CLOSED)
        self.assert_invalid(ActiveRequest, Request.RESERVED,
                            'close', TransitionError)
        self.assert_invalid(ActiveRequest, Request.WAITING,
                            'close', TransitionError)

    def test_grant(self):
        self.assert_invalid(ActiveRequest, Request.CANCELING,
                            'grant', TransitionError, [TEST_RESOURCE])
        self.assert_invalid(ActiveRequest, Request.CLOSED,
                            'grant', TransitionError, [TEST_RESOURCE])
        self.assert_invalid(ActiveRequest, Request.GRANTED,
                            'grant', TransitionError, [TEST_RESOURCE])
        rq = self.assert_valid(ActiveRequest, Request.NEW,
                               'grant', Request.GRANTED, [TEST_RESOURCE])
        self.assertEqual(rq.msg.resources, [TEST_RESOURCE])
        self.assertEqual(rq.msg.reason, Request.NONE)
        self.assert_valid(ActiveRequest, Request.RESERVED,
                          'grant', Request.GRANTED, [TEST_RESOURCE])
        self.assert_invalid(ActiveRequest, Request.PREEMPTING,
                               'grant', TransitionError, [TEST_RESOURCE])
        self.assert_valid(ActiveRequest, Request.WAITING,
                          'grant', Request.GRANTED, [TEST_RESOURCE])

    def test_preempt(self):
        # valid in every state, but only affects GRANTED requests
        self.assert_valid(ActiveRequest, Request.CANCELING,
                          'preempt', Request.CANCELING)
        rq = self.assert_valid(ActiveRequest, Request.CLOSED,
                               'preempt', Request.CLOSED,
                               Request.PREEMPTED)
        self.assertNotEqual(rq.msg.reason, Request.PREEMPTED)
        self.assertEqual(rq.msg.reason, Request.NONE)
        rq = self.assert_valid(ActiveRequest, Request.GRANTED,
                               'preempt', Request.PREEMPTING,
                               Request.PREEMPTED)
        self.assertEqual(rq.msg.reason, Request.PREEMPTED)
        self.assert_valid(ActiveRequest, Request.NEW,
                          'preempt', Request.NEW)
        self.assert_valid(ActiveRequest, Request.PREEMPTING,
                          'preempt', Request.PREEMPTING)
        self.assert_valid(ActiveRequest, Request.RESERVED,
                          'preempt', Request.RESERVED)
        self.assert_valid(ActiveRequest, Request.WAITING,
                          'preempt', Request.WAITING)

    def test_state_names(self):
        # test that STATE_NAME list matches the actual values
        self.assertEqual(len(STATE_NAME), 7)
        self.assertEqual(STATE_NAME[Request.NEW], 'NEW')
        self.assertEqual(STATE_NAME[Request.RESERVED], 'RESERVED')
        self.assertEqual(STATE_NAME[Request.WAITING], 'WAITING')
        self.assertEqual(STATE_NAME[Request.GRANTED], 'GRANTED')
        self.assertEqual(STATE_NAME[Request.PREEMPTING], 'PREEMPTING')
        self.assertEqual(STATE_NAME[Request.CANCELING], 'CANCELING')
        self.assertEqual(STATE_NAME[Request.CLOSED], 'CLOSED')

    def test_validate(self):
        rq1 = ResourceRequest(Request(id=unique_id.toMsg(TEST_UUID),
                                      resources=[TEST_RESOURCE],
                                      status=Request.NEW))
        self.assertTrue(rq1._validate(Request.GRANTED))
        self.assertTrue(rq1._validate(Request.PREEMPTING))
        self.assertFalse(rq1._validate(Request.CLOSED))


    def test_wait(self):
        self.assert_invalid(ActiveRequest, Request.CANCELING,
                            'wait', TransitionError)
        self.assert_invalid(ActiveRequest, Request.CLOSED,
                            'wait', TransitionError)
        self.assert_invalid(ActiveRequest, Request.GRANTED,
                            'wait', TransitionError)
        rq = self.assert_valid(ActiveRequest, Request.NEW,
                               'wait', Request.WAITING, Request.BUSY)
        self.assertEqual(rq.msg.reason, Request.BUSY)
        self.assert_invalid(ActiveRequest, Request.PREEMPTING,
                            'wait', TransitionError)
        rq = self.assert_valid(ActiveRequest, Request.RESERVED,
                               'wait', Request.WAITING, Request.UNAVAILABLE)
        self.assertEqual(rq.msg.reason, Request.UNAVAILABLE)
        rq = self.assert_valid(ActiveRequest, Request.WAITING,
                               'wait', Request.WAITING, Request.UNAVAILABLE)
        self.assertEqual(rq.msg.reason, Request.UNAVAILABLE)


class TestRequestSets(unittest.TestCase):
    """Unit tests for scheduler request state transitions.

    These tests do not require a running ROS core.
    """

    ####################
    # request set tests
    ####################

    def test_empty_request_set(self):
        self.assertRaises(TypeError, RequestSet)
        self.assertRaises(TypeError, RequestSet, [])
        rset = RequestSet([], RQR_UUID)
        self.assertIsNotNone(rset)
        self.assertEqual(len(rset), 0)
        self.assertNotIn(TEST_UUID, rset)
        sch_msg = SchedulerRequests(requester=unique_id.toMsg(RQR_UUID),
                                    requests=[])
        self.assertEqual(rset.to_msg(stamp=rospy.Time()), sch_msg)
        rset_str = """requester_id: 01234567-89ab-cdef-0123-456789abcdef
requests:"""
        self.assertEqual(str(rset), rset_str)

        # Test equality for empty rsets: the requester_id is
        # significant, but the contents type is ignored.
        self.assertEqual(rset, RequestSet([], RQR_UUID))
        self.assertEqual(rset, RequestSet([], RQR_UUID,
                                          contents=ActiveRequest))
        self.assertNotEqual(rset, RequestSet([], TEST_UUID))

    def test_one_request_set(self):
        msg1 = Request(id=unique_id.toMsg(TEST_UUID),
                       resources=[TEST_WILDCARD],
                       status=Request.NEW)
        rset = RequestSet([msg1], RQR_UUID, contents=ActiveRequest)
        self.assertEqual(len(rset), 1)
        self.assertIn(TEST_UUID, rset)
        self.assertEqual(rset[TEST_UUID].msg, msg1)
        self.assertEqual(rset.get(TEST_UUID), rset[TEST_UUID])
        self.assertNotIn(DIFF_UUID, rset)
        self.assertIsNone(rset.get(DIFF_UUID))
        self.assertEqual(rset.get(DIFF_UUID, 10), 10)
        self.assertEqual(rset, RequestSet([msg1], RQR_UUID))
        self.assertEqual(rset, RequestSet([msg1], RQR_UUID,
                                          contents=ActiveRequest))
        rset_str = """requester_id: 01234567-89ab-cdef-0123-456789abcdef
requests:
  id: 01234567-89ab-cdef-fedc-ba9876543210
    priority: 0
    resources: 
      rocon:/segbot#test_rapp
    status: 0"""
        self.assertEqual(str(rset), rset_str)
        sch_msg = SchedulerRequests(requester=unique_id.toMsg(RQR_UUID),
                                    requests=[msg1])
        self.assertEqual(rset.to_msg(stamp=rospy.Time()), sch_msg)

        # vary message contents:
        rs2 = copy.deepcopy(rset)
        self.assertEqual(rset, rs2)
        rq = rs2.get(TEST_UUID)
        rq.msg.priority = 10
        self.assertNotEqual(rset, rs2)
        rq.msg.priority = 0
        self.assertEqual(rset, rs2)
        rq.msg.availability = rospy.Time(1000.0)
        self.assertNotEqual(rset, rs2)
        rq.msg.availability = rospy.Time()
        self.assertEqual(rset, rs2)
        rq.msg.hold_time = rospy.Duration(1000.0)
        self.assertNotEqual(rset, rs2)
        rq.msg.hold_time = rospy.Duration()
        self.assertEqual(rset, rs2)
        rq.msg.resources = [TEST_RESOURCE]
        self.assertNotEqual(rset, rs2)
        rq.msg.resources = [TEST_WILDCARD]
        self.assertEqual(rset, rs2)

    def test_two_request_set(self):
        msg1 = Request(id=unique_id.toMsg(TEST_UUID),
                       resources=[TEST_WILDCARD])
        msg2 = Request(id=unique_id.toMsg(DIFF_UUID),
                       resources=[TEST_RESOURCE])
        rset = RequestSet([msg1, msg2], RQR_UUID)
        self.assertEqual(len(rset), 2)
        self.assertIn(TEST_UUID, rset)
        self.assertIn(DIFF_UUID, rset)
        self.assertEqual(rset[TEST_UUID].msg, msg1)
        self.assertEqual(rset[DIFF_UUID].msg, msg2)
        self.assertEqual(rset.get(TEST_UUID), rset[TEST_UUID])
        self.assertEqual(rset.get(DIFF_UUID), rset[DIFF_UUID])
        self.assertEqual(rset, RequestSet([msg1, msg2], RQR_UUID))
        self.assertEqual(rset, RequestSet([msg2, msg1], RQR_UUID))
        self.assertNotEqual(rset, RequestSet([msg1], RQR_UUID))
        self.assertNotEqual(rset, RequestSet([msg2], RQR_UUID))
        self.assertNotEqual(rset, RequestSet([], RQR_UUID))
        rs2 = copy.deepcopy(rset)
        self.assertEqual(rset, rs2)
        rs2[TEST_UUID]._transition(EVENT_GRANT)
        self.assertNotEqual(rset, rs2)

    def test_request_set_from_scheduler_requests(self):
        msg1 = Request(id=unique_id.toMsg(TEST_UUID),
                       resources=[TEST_WILDCARD])
        msg2 = Request(id=unique_id.toMsg(DIFF_UUID),
                       resources=[TEST_RESOURCE])
        schreq = SchedulerRequests(requester=unique_id.toMsg(RQR_UUID),
                                   requests=[msg1, msg2])
        rset = RequestSet(schreq)
        self.assertEqual(len(rset), 2)
        self.assertIn(TEST_UUID, rset)
        self.assertIn(DIFF_UUID, rset)
        self.assertEqual(rset[TEST_UUID].msg, msg1)
        self.assertEqual(rset[DIFF_UUID].msg, msg2)
        self.assertEqual(rset.get(TEST_UUID), rset[TEST_UUID])
        self.assertEqual(rset.get(DIFF_UUID), rset[DIFF_UUID])
        self.assertEqual(rset, RequestSet(schreq))
        self.assertEqual(rset, RequestSet(schreq, RQR_UUID))
        self.assertNotEqual(rset, RequestSet([msg1], RQR_UUID))
        self.assertNotEqual(rset, RequestSet([msg2], RQR_UUID))
        self.assertNotEqual(rset, RequestSet([], RQR_UUID))
        rs2 = copy.deepcopy(rset)
        self.assertEqual(rset, rs2)
        rs2[TEST_UUID]._transition(EVENT_GRANT)
        self.assertNotEqual(rset, rs2)

    def test_empty_merge(self):
        msg1 = Request(id=unique_id.toMsg(TEST_UUID),
                       resources=[TEST_WILDCARD],
                       status=Request.NEW)
        rset = RequestSet([msg1], RQR_UUID)
        self.assertEqual(len(rset), 1)
        self.assertIn(TEST_UUID, rset)
        sch_msg = SchedulerRequests(requester=unique_id.toMsg(RQR_UUID),
                                    requests=[msg1])
        self.assertEqual(rset.to_msg(stamp=rospy.Time()), sch_msg)

        # merge an empty request set: rset should remain the same
        rset.merge(RequestSet([], RQR_UUID, contents=ActiveRequest))
        self.assertEqual(len(rset), 1)
        self.assertIn(TEST_UUID, rset)
        self.assertEqual(rset.to_msg(stamp=rospy.Time()), sch_msg)

    def test_closed_merge(self):
        msg1 = Request(id=unique_id.toMsg(TEST_UUID),
                       resources=[TEST_WILDCARD],
                       status=Request.CLOSED)
        rset = RequestSet([msg1], RQR_UUID, contents=ActiveRequest)
        self.assertEqual(len(rset), 1)
        self.assertIn(TEST_UUID, rset)
        sch_msg = SchedulerRequests(requester=unique_id.toMsg(RQR_UUID),
                                    requests=[msg1])
        self.assertEqual(rset.to_msg(stamp=rospy.Time()), sch_msg)

        # merge an empty request set: TEST_UUID should be deleted
        empty_rset = RequestSet([], RQR_UUID)
        rset.merge(empty_rset)
        self.assertEqual(len(rset), 0)
        self.assertNotIn(TEST_UUID, rset)
        self.assertNotEqual(rset.to_msg(stamp=rospy.Time()), sch_msg)
        self.assertEqual(rset, empty_rset)

    def test_canceled_merge(self):
        msg1 = Request(id=unique_id.toMsg(TEST_UUID),
                       resources=[TEST_WILDCARD],
                       status=Request.CANCELING)
        rset = RequestSet([msg1], RQR_UUID)
        self.assertEqual(len(rset), 1)
        self.assertIn(TEST_UUID, rset)
        sch_msg = SchedulerRequests(requester=unique_id.toMsg(RQR_UUID),
                                    requests=[msg1])
        self.assertEqual(rset.to_msg(stamp=rospy.Time()), sch_msg)

        # merge a canceled request: TEST_UUID should be deleted
        msg2 = Request(id=unique_id.toMsg(TEST_UUID),
                       resources=[TEST_WILDCARD],
                       status=Request.CLOSED)
        rel_rset = RequestSet([msg2], RQR_UUID, contents=ActiveRequest)
        rset.merge(rel_rset)
        self.assertEqual(len(rset), 0)
        self.assertNotIn(TEST_UUID, rset)
        self.assertEqual(rset, RequestSet([], RQR_UUID,
                                          contents=ActiveRequest))
        self.assertNotEqual(rset.to_msg(stamp=rospy.Time()), sch_msg)
        self.assertNotEqual(rset, rel_rset)

    def test_canceled_merge_plus_new_request(self):
        msg1 = Request(id=unique_id.toMsg(TEST_UUID),
                       resources=[TEST_RESOURCE],
                       status=Request.CANCELING)
        msg2 = Request(id=unique_id.toMsg(DIFF_UUID),
                       resources=[TEST_WILDCARD],
                       status=Request.NEW)
        rset = RequestSet([msg1, msg2], RQR_UUID)
        self.assertEqual(len(rset), 2)
        self.assertIn(TEST_UUID, rset)
        self.assertIn(DIFF_UUID, rset)
        self.assertEqual(rset[DIFF_UUID].msg.status, Request.NEW)

        # merge a canceled request: TEST_UUID should be deleted, but
        # DIFF_UUID should not
        msg3 = Request(id=unique_id.toMsg(TEST_UUID),
                       resources=[TEST_RESOURCE],
                       status=Request.CLOSED)
        rel_rset = RequestSet([msg3], RQR_UUID)
        rset.merge(rel_rset)
        self.assertEqual(len(rset), 1)
        self.assertNotIn(TEST_UUID, rset)
        self.assertIn(DIFF_UUID, rset)
        self.assertEqual(rset[DIFF_UUID].msg.status, Request.NEW)

        # make a fresh object like the original msg2 for comparison
        msg4 = Request(id=unique_id.toMsg(DIFF_UUID),
                       resources=[TEST_WILDCARD],
                       status=Request.NEW)
        self.assertEqual(rset, RequestSet([msg4], RQR_UUID))
        self.assertNotEqual(rset, rel_rset)

    def test_single_merge(self):
        msg1 = Request(id=unique_id.toMsg(TEST_UUID),
                       resources=[TEST_WILDCARD],
                       status=Request.NEW)
        rset = RequestSet([msg1], RQR_UUID)
        self.assertEqual(rset[TEST_UUID].msg.status, Request.NEW)
        self.assertEqual(rset[TEST_UUID].msg.resources, [TEST_WILDCARD])
        self.assertEqual(rset[TEST_UUID].msg.id, unique_id.toMsg(TEST_UUID))

        # merge an updated request set: resource list should change
        msg2 = Request(id=unique_id.toMsg(TEST_UUID),
                       resources=[TEST_RESOURCE],
                       status=Request.GRANTED)
        rset.merge(RequestSet([msg2], RQR_UUID, contents=ActiveRequest))
        self.assertEqual(len(rset), 1)
        self.assertIn(TEST_UUID, rset)
        self.assertEqual(rset[TEST_UUID].msg.status, Request.GRANTED)
        self.assertEqual(rset[TEST_UUID].msg.resources, [TEST_RESOURCE])
        sch_msg = SchedulerRequests(requester=unique_id.toMsg(RQR_UUID),
                                    requests=[msg2])
        self.assertEqual(rset.to_msg(stamp=rospy.Time()), sch_msg)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('concert_scheduler_requests',
                    'test_transitions',
                    TestTransitions)
    rosunit.unitrun('concert_scheduler_requests',
                    'test_request_sets',
                    TestRequestSets)
