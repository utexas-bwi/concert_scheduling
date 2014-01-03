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


class FifoSchedulerNode(object):
    """ First come, first served (FIFO) scheduler node.

    :param node_name: (str) Default name of scheduler node.

    Derived versions of this class can implement different scheduling
    policies.
    """
    def __init__(self, node_name='fifo_scheduler'):
        """ Constructor. """
        rospy.init_node(node_name)
        self.pool = ResourcePool()
        self.ready_queue = deque()      # FIFO queue of waiting requests
        self.blocked_queue = deque()    # FIFO queue of blocked requests
        self.sch = Scheduler(self.callback)
        rospy.spin()

    def callback(self, rset):
        """ Scheduler request callback.

        See: :class:`.rocon_scheduler_requests.Scheduler` documentation.
        """
        rospy.logdebug('scheduler callback:')
        for rq in rset.values():
            rospy.logdebug('  ' + str(rq))
            if rq.msg.status == Request.NEW:
                self.queue_ready(rq, rset.requester_id)
            elif rq.msg.status == Request.CANCELING:
                self.free(rq, rset.requester_id)

    def dispatch(self):
        """ Grant any available resources to ready requests. """
        while len(self.ready_queue) > 0:
            rq, requester_id = self.ready_queue.popleft()
            resources = None
            resources = self.pool.allocate(rq)
            if not resources:           # oldest request cannot be satisfied?
                # Return it to head of queue.
                self.ready_queue.appendleft((rq, requester_id))
                return
            try:
                rq.grant(resources)
                rospy.loginfo('Request granted: ' + str(rq.get_uuid()))
            except TransitionError:     # request no longer active?
                # Return allocated resources to the pool.
                self.pool.release_resources(resources)
            try:
                self.sch.notify(requester_id)
            except KeyError:            # requester now missing?
                # Release request allocation.
                self.pool.release_request(rq)

    def free(self, request, requester_id):
        """ Free all resources allocated for this *request*.

        :param request: (:class:`.RequestReply`) 
        :param requester_id: (:class:`uuid.UUID`) Unique requester identifier.
        """
        self.pool.release_request(request)
        rospy.loginfo('Request canceled: ' + str(request.get_uuid()))
        request.close()
        self.dispatch()                 # grant waiting requests

    def queue_blocked(self, request, requester_id):
        """ Add request to blocked queue.

        :param request: (:class:`.RequestReply`) 
        :param requester_id: (:class:`uuid.UUID`) Unique requester identifier.
        """
        try:
            request.wait(reason=Request.UNAVAILABLE)
        except TransitionError:         # request no longer active?
            return
        self.blocked_queue.append((requester_id, request))
        rospy.loginfo('Request blocked: ' + str(request.get_uuid()))

    def queue_ready(self, request, requester_id):
        """ Add request to ready queue, making it wait. 

        :param request: (:class:`.RequestReply`) 
        :param requester_id: (:class:`uuid.UUID`) Unique requester identifier.
        """
        try:
            request.wait(reason=Request.BUSY)
        except TransitionError:         # request no longer active?
            return
        self.ready_queue.append((request, requester_id))
        rospy.loginfo('Request queued: ' + str(request.get_uuid()))
        self.dispatch()


def main():
    """ Scheduler node main entry point. """
    node = FifoSchedulerNode()
