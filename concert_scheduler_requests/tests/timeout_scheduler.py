#!/usr/bin/env python
""" Scheduler for requester timeout testing. """

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import unittest
from collections import deque
import rospy
from scheduler_msgs.msg import Request, Resource
from concert_scheduler_requests import Scheduler, TransitionError


class TestTimeoutScheduler(unittest.TestCase):

    def test_timeout_scheduler(self):
        """ Initialize ROCON scheduler node for timeout test. """
        rospy.init_node("timeout_scheduler")
        # simplifying assumptions: all requests want a single robot,
        # and any of these will do:
        self.avail = deque(           # FIFO queue of available robots
            [Resource(
                rapp='example_rapp', uri='rocon:/turtlebot/roberto'),
             Resource(
                rapp='example_rapp', uri='rocon:/turtlebot/marvin')])
        self.ready_queue = deque()      # FIFO queue of waiting requests
        self.seen_requester = False
        self.timer = rospy.Timer(rospy.Duration(2.0), self.check_finished)
        self.sch = Scheduler(self.callback, frequency=1.0)
        rospy.spin()

    def callback(self, rset):
        """ Scheduler request callback. """
        rospy.logdebug('scheduler callback:')
        for rq in rset.values():
            rospy.logdebug('  ' + str(rq))
            if rq.msg.status == Request.NEW:
                self.queue(rset.requester_id, rq)
            elif rq.msg.status == Request.CANCELING:
                self.free(rset.requester_id, rq)

    def check_finished(self, event):
        """ Timer event handler:

        Stops test after at least one requester has come and gone.
        """
        if self.seen_requester:
            if len(self.sch.requesters) == 0:
                rospy.loginfo('requester connection lost')
                rospy.signal_shutdown('test completed.')
        else:
            if len(self.sch.requesters) > 0:
                self.seen_requester = True

    def dispatch(self):
        """ Grant any available resources to waiting requests. """
        while len(self.ready_queue) > 0:
            if len(self.avail) == 0:    # no resources available?
                return
            resource = self.avail.popleft()
            requester_id, rq = self.ready_queue.popleft()
            try:                        # grant request & notify requester
                rq.grant([resource])
                self.sch.notify(requester_id)
                rospy.loginfo('Request granted: ' + str(rq.uuid))
            except (TransitionError, KeyError):
                # request no longer active or requester missing?
                # Put resource back at the front of the queue.
                self.avail.appendleft(resource)

    def free(self, requester_id, rq):
        """ Free all resources allocated for this request. """
        self.avail.extend(rq.allocations)
        rospy.loginfo('Request canceled: ' + str(rq.uuid))
        rq.close()
        self.dispatch()                 # grant waiting requests

    def queue(self, requester_id, rq):
        """ Add request to ready queue, making it wait. """
        try:
            rq.wait(reason=Request.BUSY)
        except TransitionError:         # request no longer active?
            return
        self.ready_queue.append((requester_id, rq))
        rospy.loginfo('Request queued: ' + str(rq.uuid))
        self.dispatch()

if __name__ == '__main__':
    import rostest
    rostest.rosrun('concert_scheduler_requests',
                   'timeout_scheduler', TestTimeoutScheduler)
