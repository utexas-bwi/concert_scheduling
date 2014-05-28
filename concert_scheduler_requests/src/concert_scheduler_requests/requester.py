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
.. module:: requester

Python interface for ROCON services making scheduler requests.

.. include:: weblinks.rst

"""

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import copy

# ROS dependencies
import rospy
import threading
import unique_id

# ROS messages
from scheduler_msgs.msg import Request, SchedulerRequests

# internal modules
from . import common
from . import TransitionError, WrongRequestError
from .transitions import RequestSet


class Requester:
    """
    This class is used by a ROCON service to handle its resource
    requests.  When an instance of :class:`.Requester` is created, it
    creates its own scheduler feedback topic and connects to the ROCON
    scheduler topic.

    :param feedback: Callback function invoked with the current
                     :class:`.RequestSet` when feedback arrives.

    :param uuid: UUID_ of this requester. If ``None`` provided, a random
                 UUID will be assigned.
    :type uuid: :class:`uuid.UUID`

    :param priority: default priority for requests from this requester.

    :param topic: Base name of of resource allocation request topic.
        If ``None``, search for a ROS ``topic_name`` parameter, or
        else assume ``rocon_scheduler``.
    :type topic: str or ``None``

    :param frequency: requester heartbeat frequency in Hz.  Use the
                      default, except in exceptional situations or for
                      testing.
    :type frequency: float

    :param lock: The big requester serialization lock, allocated
        internally, if ``None``.
    :type lock: :class:`.threading.RLock()`

    As long as the :class:`.Requester` object remains, it will
    periodically send request messages to the scheduler, even when no
    requests are outstanding.  The scheduler will provide feedback for
    them if anything has changed.  The caller-provided *feedback*
    function will be invoked each time a feedback message arrives,
    like this:

    .. describe:: feedback(rset)

       :param rset: The current set of requests including any updates
                    from the scheduler.
       :type rset: :class:`.RequestSet`


    The *feedback* function is called when new feedback is received
    from the scheduler, already holding the :ref:`Big Requester Lock
    <Big_Requester_Lock>`.  It is expected to iterate over its
    :class:`.RequestSet`, checking the status of every
    :class:`.ResourceRequest` it contains, modifying them
    appropriately, then return without waiting. If any changes occur,
    the scheduler will be notified after this callback returns.

    Usage example:

    .. literalinclude:: ../tests/example_requester.py

    """

    def __init__(self, feedback, uuid=None, priority=0, topic=None,
                 frequency=common.HEARTBEAT_HZ, lock=None):
        """ Constructor. """
        if lock is None:
            lock = threading.RLock()
        self.lock = lock
        """
        .. _Big_Requester_Lock:

        Big Requester Lock.

        This recursive Python threading lock serializes access to
        requester data.  The requester *feedback* function is always
        invoked holding it.  All :class:`.Requester` methods acquire
        it automatically, whenever needed.

        In any other thread, acquire it when updating shared request
        set objects.  Never hold it when sleeping or waiting for I/O.
        """
        if uuid is None:
            uuid = unique_id.fromRandom()
        self.requester_id = uuid
        """ :class:`uuid.UUID` of this requester. """
        self.rset = RequestSet([], self.requester_id)
        """
        :class:`.RequestSet` containing the current status of every
        :class:`.ResourceRequest` made by this requester.  All
        requester operations are done using this object and its
        contents.
        """
        self.priority = priority
        """ Default for new requests' priorities if none specified. """

        self.feedback = feedback        # requester feedback
        if topic is None:
            topic_param = rospy.search_param('topic_name')
            if topic_param is None:
                topic = common.SCHEDULER_TOPIC
            else:
                topic = rospy.get_param(topic_param)
        self.pub_topic = topic
        self.sub_topic = common.feedback_topic(uuid, topic)
        rospy.loginfo('ROCON requester feedback topic: ' + self.sub_topic)
        self.sub = rospy.Subscriber(self.sub_topic, SchedulerRequests,
                                    self._feedback,
                                    queue_size=1, tcp_nodelay=True)
        self.pub = rospy.Publisher(self.pub_topic, SchedulerRequests,
                                   latch=True, queue_size=1)
        self.time_delay = rospy.Duration(1.0 / frequency)
        self._set_timer()

    def cancel_all(self):
        """ Cancel all current requests to the scheduler.

        After calling this method to cancel all your requests, invoke
        :py:meth:`.send_requests` to notify the scheduler immediately.

        """
        with self.lock:
            self.rset.cancel_all()

    def _feedback(self, msg):
        """ Scheduler feedback message handler. """
        with self.lock:
            prev_rset = copy.deepcopy(self.rset)
            self.rset.merge(RequestSet(msg))
            if self.rset != prev_rset:  # anything changed?
                # invoke user-defined callback function
                self.feedback(self.rset)
                if self.rset != prev_rset:
                    # msg or callback changed something, so send
                    # updated requests immediately
                    self.send_requests()

    def _heartbeat(self, event):
        """ Scheduler request heartbeat timer handler.

        Triggered after nothing has been sent to the scheduler within
        the previous time_delay duration.  Sends another copy of the
        current request set to the scheduler.

        """
        if self.timer:
            self.send_requests()

    def new_request(self, resources, priority=None, uuid=None,
                    reservation=rospy.Time(),
                    hold_time=rospy.Duration()):
        """ Add a new scheduler request.

        Call this method for each desired new request, then invoke
        :py:meth:`.send_requests` to notify the scheduler.

        :param resources: ROCON resources requested
        :type resources: list of scheduler_msgs/Resource

        :param priority: Scheduling priority of this request.  If
            ``None`` provided, use this requester's default priority.
        :type priority: int

        :param uuid: UUID_ of this request. If ``None`` provided, a
            random UUID will be assigned.
        :type uuid: :class:`uuid.UUID` or ``None``

        :param reservation: time when request desired, default:
            immediately.
        :type reservation: rospy.Time

        :param hold_time: estimated duration the resource will be
            held, default: unknown.
        :type hold_time: rospy.Duration

        :returns: UUID (:class:`uuid.UUID`) assigned.
        :raises: :exc:`.WrongRequestError` if request already exists.
        """
        if priority is None:
            priority = self.priority
        status = Request.NEW
        if reservation != rospy.Time():
            status = Request.RESERVED
        if uuid is None:
            uuid = unique_id.fromRandom()
        if uuid in self.rset:
            raise WrongRequestError('UUID already in use.')
        msg = Request(id=unique_id.toMsg(uuid),
                      priority=priority,
                      resources=resources,
                      status=status,
                      availability=reservation,
                      hold_time=hold_time)
        with self.lock:
            self.rset[uuid] = msg
        return uuid

    def send_requests(self):
        """ Send all current requests to the scheduler.

        Use this method after updating :py:attr:`.rset` or calling
        :py:meth:`.new_request` one or more times.  It will send them
        to the scheduler immediately.  Otherwise, they would not go
        out until the next heartbeat timer event.

        .. note::

           A recent heartbeat may already have sent some recent
           requests.  This method just ensures they are all sent
           without further delay.

        """
        with self.lock:
            self.pub.publish(self.rset.to_msg())

    def _set_timer(self):
        """ Schedule heartbeat timer callback. """
        if not rospy.is_shutdown():
            self.timer = rospy.Timer(self.time_delay, self._heartbeat)

    def _unregister(self):
        """ Stop sending heartbeat messages to scheduler.

        Mainly for testing, *not for normal use.*
        """
        self.timer.shutdown()
        self.timer = None
