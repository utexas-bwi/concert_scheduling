#!/usr/bin/env python

""" Scheduler for testing example_requester. """

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import unittest
import rospy
from scheduler_msgs.msg import Request, Resource
from concert_scheduler_requests import Scheduler

# Resource to grant
TEST_RESOURCE = Resource(rapp='test_rapp', uri='rocon:/turtlebot/roberto')

# Global variables
# :todo: make these class variables
queued_request = None           # pending request
queued_requester = None         # corresponding requester ID


class TestExampleRequester(unittest.TestCase):

    def test_example_requester_(self):
        """ Initialize ROCON scheduler node for example requester. """
        self.number_of_requests = 3     # number of requests desired
        self.queued_request = None
        rospy.init_node("test_example_requester")
        self.sch = Scheduler(self.callback)
        rospy.spin()                    # spin in the main thread

    def allocate(self, event):
        """ Timer handler: simulated resource availability event.

        This method runs in a different thread from the scheduler
        callback, so acquire the Big Scheduler Lock before doing
        anything.
        """
        with self.sch.lock:
            if self.queued_request:
                self.queued_request.grant([TEST_RESOURCE])
                rospy.loginfo('Request granted: '
                              + str(self.queued_request.uuid))
                if self.number_of_requests == 2:
                    # preempt this request immediately
                    self.queued_request.preempt()
                    rospy.loginfo('Request preempted: '
                                  + str(self.queued_request.uuid))
                self.queued_request = None
                self.sch.notify(self.queued_requester)

    def queue(self, requester_id, request):
        """
        Queue timer request to simulate delayed resource availability.
        """
        # this test case only makes one request at a time:
        self.assertIsNone(self.queued_request)
        self.queued_requester = requester_id
        self.queued_request = request
        self.timer = rospy.Timer(rospy.Duration(1.0),
                                 self.allocate, oneshot=True)
        rospy.loginfo('Request queued: ' + str(request.uuid))

    def callback(self, rset):
        """ Scheduler request callback.

        Makes new requests wait, they will be granted after a second.

        """
        for rq in rset.values():
            if rq.msg.status == Request.NEW:
                rq.wait(reason=Request.BUSY)
                self.queue(rset.requester_id, rq)
            elif rq.msg.status == Request.CANCELING:
                rq.close()
                rospy.loginfo('Request canceled: ' + str(rq.uuid))
                self.number_of_requests -= 1
                if self.number_of_requests <= 0:
                    rospy.signal_shutdown('test completed.')

if __name__ == '__main__':
    import rostest
    rostest.rosrun('concert_scheduler_requests',
                   'test_example_requester',
                   TestExampleRequester) 
