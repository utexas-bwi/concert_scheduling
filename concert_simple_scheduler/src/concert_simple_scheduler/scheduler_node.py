# Software License Agreement (BSD License)
#
# Copyright (C) 2013-2014, Jack O'Quin
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

.. include:: weblinks.rst

"""
import rospy
from rocon_scheduler_requests import Scheduler, TransitionError
from scheduler_msgs.msg import Request

from .resource_pool import ResourcePool
from .resource_pool import CurrentStatus
from .priority_queue import PriorityQueue, QueueElement


class SimpleSchedulerNode(object):
    """ Simple scheduler node.

    :param node_name: (str) Default name of scheduler node.
    :param period: (:class:`rospy.Duration`) Rescheduling time quantum.

    Derived versions of this class can implement different scheduling
    policies.
    """
    def __init__(self, node_name='simple_scheduler',
                 period=rospy.Duration(1.0)):
        """ Constructor. """
        rospy.init_node(node_name)
        self.pool = ResourcePool()
        self.ready_queue = PriorityQueue()
        """ Queue of waiting requests. """
        self.blocked_queue = PriorityQueue()
        """ Queue of blocked requests. """
        self.period = period
        """ Duration between periodic rescheduling. """
        self.timer = rospy.Timer(self.period, self.reschedule)
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
                self.queue(rq, rset.requester_id)
            elif rq.msg.status == Request.CANCELING:
                self.free(rq, rset.requester_id)

    def dispatch(self):
        """ Grant any available resources to ready requests. """
        while len(self.ready_queue) > 0:
            # Try to allocate top element in the ready queue.
            elem = self.ready_queue.pop()
            resources = self.pool.allocate(elem.request)
            if not resources:           # top request cannot be satisfied?
                # Return it to head of queue.
                self.ready_queue.add(elem)
                return
            try:
                elem.request.grant(resources)
                rospy.loginfo(
                    'Request granted: ' + str(elem.request.get_uuid()))
            except TransitionError:     # request no longer active?
                # Return allocated resources to the pool.
                self.pool.release_resources(resources)
            try:
                self.sch.notify(elem.requester_id)
            except KeyError:            # requester now missing?
                # Release requested allocation.
                self.pool.release_request(elem.request)

    def free(self, request, requester_id):
        """ Free all resources allocated for this *request*.

        :param request: (:class:`.RequestReply`)
        :param requester_id: (:class:`uuid.UUID`) Unique requester identifier.
        """
        self.pool.release_request(request)
        rospy.loginfo('Request canceled: ' + str(request.get_uuid()))
        request.close()
        self.dispatch()                 # grant waiting requests

    def queue(self, request, requester_id):
        """ Add *request* to ready queue, making it wait.

        :param request: resource request to be queued.
        :type request: :class:`.ActiveRequest`
        :param requester_id: Unique requester identifier.
        :type requester_id: :class:`uuid.UUID`
        """
        try:
            request.wait(reason=Request.BUSY)
        except TransitionError:         # request no longer active?
            return
        self.ready_queue.add(QueueElement(request, requester_id))
        rospy.loginfo('Request queued: ' + str(request.get_uuid()))
        self.dispatch()                 # allocate queued requests

    def reschedule(self, event):
        """ Periodic rescheduling.

        Uses the Big Scheduler Lock to serialize changes with
        operations done within the scheduler callback method.
        """
        with self.sch.lock:
            while len(self.ready_queue) > 0:
                # see if head of ready queue can be scheduled
                elem = self.ready_queue.pop()
                resources = elem.request.msg.resources
                status_set = {CurrentStatus.AVAILABLE,
                              CurrentStatus.ALLOCATED}
                matches = self.pool.match_list(resources, status_set)
                if matches:
                    match_union = set(chain.from_iterable(matches))
                    if len(match_union) >= len(resources):
                        # there is enough in the pool, return elem to queue
                        self.ready_queue.add(elem)
                        return          # done rescheduling

                # move elem to blocked_queue
                self.blocked_queue.add(elem)
                rospy.loginfo('Request blocked: '
                              + str(elem.request.get_uuid()))


def main():
    """ Scheduler node main entry point. """
    node = SimpleSchedulerNode()
