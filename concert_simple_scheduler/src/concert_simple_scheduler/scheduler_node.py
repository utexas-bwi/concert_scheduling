""" Simple scheduler node. """
from collections import deque
import rospy
from scheduler_msgs.msg import Request
from scheduler_msgs.msg import Resource
from rocon_scheduler_requests import Scheduler, TransitionError


class SchedulerNode:

    def __init__(self):
        rospy.init_node("simple_scheduler")
        # simplifying assumptions: all requests want a single robot,
        # and any of these will do:
        self.avail = deque([            # FIFO queue of available robots
            Resource(name='example_rapp',
                     platform_info='linux.precise.ros.turtlebot.roberto'),
            Resource(name='example_rapp',
                     platform_info='linux.precise.ros.turtlebot.marvin'),
            ])
        self.ready_queue = deque()      # FIFO queue of waiting requests
        self.sch = Scheduler(self.callback)
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
                rospy.loginfo('Request granted: ' + str(rq.get_uuid()))
            except (TransitionError, KeyError):
                # request no longer active or requester missing?
                # Put resource back at the front of the queue.
                self.avail.appendleft(resource)

    def free(self, requester_id, rq):
        """ Free all resources allocated for this request. """
        self.avail.extend(rq.allocations)
        rospy.loginfo('Request canceled: ' + str(rq.get_uuid()))
        rq.close()
        self.dispatch()                 # grant waiting requests

    def queue(self, requester_id, rq):
        """ Add request to ready queue, making it wait. """
        try:
            rq.wait(reason=Request.BUSY)
        except TransitionError:         # request no longer active?
            return
        self.ready_queue.append((requester_id, rq))
        rospy.loginfo('Request queued: ' + str(rq.get_uuid()))
        self.dispatch()


def main():
    node = SchedulerNode()
