#!/usr/bin/env python
""" Requester for testing scheduler timeout handling. """

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import unittest
import rospy
from scheduler_msgs.msg import Request, Resource
from concert_scheduler_requests import Requester


class TestTimeoutRequester(unittest.TestCase):

    def test_example_requester_(self):
        """ Initialize ROCON scheduler node for example requester. """
        rospy.init_node("test_example_scheduler")
        self.rqr = Requester(self.feedback, frequency=1.0)
        self.next_step = self.step1     # first step of test sequence
        self.timer = rospy.Timer(rospy.Duration(2.0), self.periodic_update)
        rospy.spin()

    def feedback(self, rset):
        """ Scheduler feedback function. """
        rospy.loginfo('feedback callback:')
        for rq in rset.values():
            rospy.logdebug('  ' + str(rq))
            if rq.msg.status == Request.WAITING:
                rospy.loginfo('  request queued: ' + str(rq.uuid))
            elif rq.msg.status == Request.GRANTED:
                rospy.loginfo('  request granted: ' + str(rq.uuid))
            elif rq.msg.status == Request.CLOSED:
                rospy.loginfo('  request closed: ' + str(rq.uuid))
            elif rq.msg.status == Request.PREEMPTING:
                rospy.loginfo('  request preempted (reason='
                              + str(rq.msg.reason) + '): ' + str(rq.uuid))
                rq.cancel()     # release preempted resources immediately

    def periodic_update(self, event):
        """ Timer event handler for periodic request updates.

        Invokes self.next_step(), unless ``None``.
        """
        if self.next_step is not None:  # more to do?
            self.next_step()
        else:                           # no more steps
            rospy.signal_shutdown('test completed.')

    def request_turtlebot(self):
        """ Request any tutlebot able to run *example_rapp*.

        :returns: UUID of new request sent.
        """
        bot = Resource(rapp='tests/example_rapp', uri='rocon:/turtlebot')
        rq_id = self.rqr.new_request([bot])
        rospy.loginfo('  new request: ' + str(rq_id))
        return rq_id

    def verify(self, rq_list):
        self.assertEqual(len(self.rqr.rset), len(rq_list))
        for rq in rq_list:
            self.assertTrue(rq in self.rqr.rset)

    def step1(self):
        rospy.loginfo('Step 1')
        # allocate two requests and send them immediately
        self.rq1 = self.request_turtlebot()
        self.rq2 = self.request_turtlebot()
        self.verify([self.rq1, self.rq2])
        self.rqr.send_requests()
        self.next_step = self.step2

    def step2(self):
        rospy.loginfo('Step 2')
        self.verify([self.rq1, self.rq2])

        # send another request, which should wait
        self.rq3 = self.request_turtlebot()
        self.verify([self.rq1, self.rq2, self.rq3])

        self.rqr.send_requests()
        self.next_step = self.step3

    def step3(self):
        rospy.loginfo('Step 3')
        self.verify([self.rq1, self.rq2, self.rq3])
        self.rqr.rset[self.rq2].cancel()
        self.rq4 = self.request_turtlebot()
        self.verify([self.rq1, self.rq2, self.rq3, self.rq4])
        self.rqr.send_requests()
        self.next_step = self.step4

    def step4(self):
        rospy.loginfo('Step 4')
        self.verify([self.rq1, self.rq3, self.rq4])
        self.next_step = None

if __name__ == '__main__':
    import rostest
    rostest.rosrun('concert_simple_scheduler',
                   'timeout_requester', TestTimeoutRequester)
