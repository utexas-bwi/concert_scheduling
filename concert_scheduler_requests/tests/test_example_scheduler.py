#!/usr/bin/env python
""" Requester for testing example_scheduler. """

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import unittest
import collections
import rospy
from scheduler_msgs.msg import  Request, Resource
from concert_scheduler_requests import Requester


class TestExampleScheduler(unittest.TestCase):

    def test_example_scheduler_(self):
        """ Initialize ROCON scheduler node for example requester. """
        rospy.init_node("test_example_scheduler")
        self.rqr = Requester(self.feedback)
        self.actions = collections.deque([self.step1, self.step2, self.step3])
        self.timer = rospy.Timer(rospy.Duration(2.0), self.periodic_update)
        rospy.spin()

    def feedback(self, rset):
        """ Scheduler feedback function. """
        print('feedback callback:')
        for rq in rset.values():
            print('  ' + str(rq))
            if rq.msg.status == Request.WAITING:
                print('Request queued: ' + str(rq.uuid))
            elif rq.msg.status == Request.GRANTED:
                print('Request granted: ' + str(rq.uuid))
            elif rq.msg.status == Request.CLOSED:
                print('Request closed: ' + str(rq.uuid))
            elif rq.msg.status == Request.PREEMPTING:
                print('Request preempted (reason=' + str(rq.msg.reason)
                      + '): ' + str(rq.uuid))
                rq.cancel()     # release preempted resource immediately

    def periodic_update(self, event):
        """ Timer event handler for periodic request updates. 

        This method runs in a different thread from the feedback
        callback, so acquire the Big Requester Lock for running each
        action step, even though most of them do not require it.
        """
        # Invoke self.actions in order.
        if len(self.actions) > 0:       # actions remain?
            next_step = self.actions.popleft()
            with self.rqr.lock:
                next_step()
        else:                           # no more actions
            rospy.signal_shutdown('test completed.')

    def request_turtlebot(self):
        """ Request any tutlebot able to run *example_rapp*.
        :returns: UUID of new request sent.
        """
        bot = Resource(rapp='example_rapp', uri='rocon:/turtlebot')
        request_id = self.rqr.new_request([bot])
        self.rqr.send_requests()
        return request_id

    def step1(self):
        """ first step of test sequence. """
        print('Step 1')
        self.rq1 = self.request_turtlebot()
        print('New request: ' + str(self.rq1))

    def step2(self):
        """ second step of test sequence. """
        print('Step 2')
        # verify that there is one active request
        self.assertEqual(len(self.rqr.rset), 1)
        self.assertTrue(self.rq1 in self.rqr.rset)

        # cancel previous request
        self.rqr.cancel_all()
        self.rqr.send_requests()

        # send a new one immediately
        self.rq2 = self.request_turtlebot()
        print('New request: ' + str(self.rq2))

        # verify that there are two active requests
        self.assertEqual(len(self.rqr.rset), 2)
        self.assertTrue(self.rq2 in self.rqr.rset)

    def step3(self):
        """ third step of test sequence. """
        # verify that there is only one active request
        print('Step 3')
        self.assertEqual(len(self.rqr.rset), 1)
        self.assertTrue(self.rq1 not in self.rqr.rset)
        self.assertTrue(self.rq2 in self.rqr.rset)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('concert_scheduler_requests',
                   'test_example_scheduler',
                   TestExampleScheduler) 
