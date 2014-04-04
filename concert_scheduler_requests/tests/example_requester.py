#!/usr/bin/env python
""" Requester usage example. """
import rospy
from scheduler_msgs.msg import Request, Resource
from concert_scheduler_requests import Requester


class ExampleRequester:

    def __init__(self):
        rospy.init_node("example_requester")
        self.rqr = Requester(self.feedback)
        self.request_turtlebot()
        self.timer = rospy.Timer(rospy.Duration(2.0), self.periodic_update)
        rospy.spin()

    def feedback(self, rset):
        """ Scheduler feedback function. """
        for rq in rset.values():
            if rq.msg.status == Request.WAITING:
                rospy.loginfo('Request queued: ' + str(rq.uuid))
            elif rq.msg.status == Request.GRANTED:
                rospy.loginfo('Request granted: ' + str(rq.uuid))
            elif rq.msg.status == Request.CLOSED:
                rospy.loginfo('Request closed: ' + str(rq.uuid))
            elif rq.msg.status == Request.PREEMPTING:
                rospy.loginfo('Request preempted (reason='
                              + str(rq.msg.reason) + '): ' + str(rq.uuid))
                rq.cancel()     # release preempted resource immediately

    def periodic_update(self, event):
        """ Timer event handler for periodic request updates. """
        try:
            # cancel the previous request, holding the Big Requester Lock
            with self.rqr.lock:
                self.rqr.rset[self.request_id].cancel()
        except KeyError:
            # previous request is gone, request another similar robot
            self.request_turtlebot()

    def request_turtlebot(self):
        """ Request any tutlebot able to run *example_rapp*. """
        bot = Resource(rapp='example_rapp', uri='rocon:/turtlebot')
        self.request_id = self.rqr.new_request([bot])
        self.rqr.send_requests()

if __name__ == '__main__':
    node = ExampleRequester()
