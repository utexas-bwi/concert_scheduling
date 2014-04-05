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
# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import rospy
from concert_scheduler_requests import Scheduler, TransitionError
from concert_scheduler_requests.priority_queue import (PriorityQueue,
                                                       QueueElement)
from scheduler_msgs.msg import Request

from .resource_pool import CurrentStatus
from .resource_pool import InvalidRequestError
from .resource_pool import ResourcePool
from .scheduler_clients import SchedulerClients


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
        self.ready_queue = PriorityQueue()
        """ Queue of waiting requests. """
        self.blocked_queue = PriorityQueue()
        """ Queue of blocked requests. """
        self.period = period
        """ Time duration between periodic rescheduling. """
        self.notification_set = set()
        """ Set of requester identifiers to notify. """
        self.timer = rospy.Timer(self.period, self.reschedule)

        try:
            topic_name = rospy.get_param('~topic_name')
            self.sch = Scheduler(self.callback, topic=topic_name)
        except KeyError:
            self.sch = Scheduler(self.callback)  # use the default
        self.clients = SchedulerClients(self.sch.lock, ResourcePool)
        """ Known ROCON clients. """

        # Handle messages until canceled.
        rospy.spin()

    def callback(self, rset):
        """ Scheduler request callback.

        Called in the scheduler callback thread holding the Big
        Scheduler Lock.

        See: :class:`.concert_scheduler_requests.Scheduler` documentation.
        """
        rospy.logdebug('scheduler callback:')
        for rq in rset.values():
            rospy.logdebug('  ' + str(rq))
            if rq.msg.status == Request.NEW:
                self.queue(rq, rset.requester_id)
            elif rq.msg.status == Request.CANCELING:
                self.free(rq, rset.requester_id)
        self.dispatch()                 # try to allocate ready requests

    def dispatch(self):
        """ Grant any available resources to ready requests.

        Notifies all affected requesters.
        """
        while len(self.ready_queue) > 0:
            # Try to allocate top element in the ready queue.
            elem = self.ready_queue.pop()
            resources = []
            try:
                resources = self.clients.allocate(elem.request)
            except InvalidRequestError as ex:
                self.reject_request(elem, ex)
                continue                # skip to next queue element

            if not resources:           # top request cannot be satisfied?
                # Return it to head of queue.
                self.ready_queue.add(elem)
                break                   # stop looking

            try:
                elem.request.grant(resources)
                rospy.loginfo(
                    'Request granted: ' + str(elem.request.uuid))
            except TransitionError:     # request no longer active?
                # Return allocated resources to the pool.
                self.clients.release_resources(resources)
            self.notification_set.add(elem.requester_id)

        # notify all affected requesters
        self.notify_requesters()

        # notify of resource pool changes, if necessary
        self.clients.notify_resources()

    def free(self, request, requester_id):
        """ Free all resources allocated for this *request*.

        :param request: (:class:`.ActiveRequest`)
        :param requester_id: (:class:`uuid.UUID`) Unique requester identifier.
        """
        self.clients.release_request(request)
        rospy.loginfo('Request canceled: ' + str(request.uuid))
        request.close()
        # remove request from any queues
        request_id = request.uuid
        for queue in [self.ready_queue, self.blocked_queue]:
            if request_id in queue:
                queue.remove(request_id)
                break                   # should not be in any other queue
        self.notification_set.add(requester_id)

    def notify_requesters(self):
        """ Notify affected requesters.

        :pre: self.notification_set contains requesters to notify.
        :post: self.notification_set is empty.
        """
        for requester_id in self.notification_set:
            try:
                self.sch.notify(requester_id)
            except KeyError:            # requester now missing?
                # shut down this requester
                self.shutdown_requester(requester_id)
        self.notification_set.clear()

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
        rospy.loginfo('Request queued: ' + str(request.uuid))
        self.notification_set.add(requester_id)

    def reject_request(self, element, exception):
        """ Reject an invalid queue *element*.

        :param element: Queue element to reject.
        :type element: :class:`.QueueElement`
        :param exception: Associated exception object.
        """
        rospy.logwarn(str(exception))
        if hasattr(Request, "INVALID"):  # new reason code defined?
            element.request.cancel(Request.INVALID)
        else:
            element.request.cancel(Request.UNAVAILABLE)
        self.notification_set.add(element.requester_id)

    def reschedule(self, event):
        """ Periodic rescheduling thread.

        Moves requests that cannot be satisfied with
        currently-available resources to the blocked queue.

        TODO: Also check the blocked queue for requests that can
        be satisfied due to newly arrived resources (#16).

        Uses the Big Scheduler Lock to serialize changes with
        operations done within the scheduler callback thread.
        """
        with self.sch.lock:
            while len(self.ready_queue) > 0:
                # see if head of ready queue can be scheduled
                elem = self.ready_queue.pop()

                # see if all available or allocated resources would suffice
                criteria = {CurrentStatus.AVAILABLE,
                            CurrentStatus.ALLOCATED}
                if self.clients.match_list(elem.request.msg.resources,
                                           criteria):
                    # request not blocked
                    self.ready_queue.add(elem)
                    break               # done rescheduling

                # move elem to blocked_queue
                rospy.loginfo('Request blocked: '
                              + str(elem.request.uuid))
                elem.request.wait(reason=Request.UNAVAILABLE)
                self.blocked_queue.add(elem)
                self.notification_set.add(elem.requester_id)

            # try to allocate any remaining ready requests
            self.dispatch()

    def shutdown_requester(self, requester_id):
        """ Shut down this requester, recovering all resources assigned. """
        for queue in [self.ready_queue, self.blocked_queue]:
            for rq in queue:
                if rq.requester_id == requester_id:
                    self.free(rq, requester_id)


def main():
    """ Scheduler node main entry point. """
    node = SimpleSchedulerNode()
