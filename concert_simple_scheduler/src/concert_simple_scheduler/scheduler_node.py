# Software License Agreement (BSD License)
#
# Copyright (C) 2013, Jack O'Quin
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the author nor of other contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
.. module:: scheduler_node

Simple scheduler node.

This module implements the scheduler interface for the `Robotics in
Concert`_ (ROCON) project.  It tracks resources and allocates them to
ROCON services.

Being a *simple* scheduler, this node allocates resources to requests
on a `first come, first served`_ basis.

.. _`first come, first served`:
    http://en.wikipedia.org/wiki/First-come,_first-served
.. _ROCON: http://www.robotconcert.org/wiki/Main_Page
.. _`Robotics in Concert`: http://www.robotconcert.org/wiki/Main_Page

"""
from collections import deque

import rospy
from rocon_scheduler_requests import Scheduler, TransitionError

# ROS messages
from scheduler_msgs.msg import Request, Resource

from .resource_pool import ResourcePool


class SchedulerNode:

    def __init__(self):
        rospy.init_node("simple_scheduler")
        self.pool = ResourcePool()
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
        queue_next = 0
        while queue_next < len(self.ready_queue):
            requester_id, rq = self.ready_queue[queue_next]
            queue_next += 1
            resources = self.pool.allocate(rq.msg)
            if resources:
                try:                    # grant request & notify requester
                    rq.grant(resources)
                    self.sch.notify(requester_id)
                    rospy.loginfo('Request granted: ' + str(rq.get_uuid()))
                except (TransitionError, KeyError):
                    # request no longer active or requester missing?
                    # Put resource back at the front of the queue.
                    self.pool.release(resources)

    def free(self, requester_id, rq):
        """ Free all resources allocated for this request. """
        self.pool.release(rq.allocations)
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
