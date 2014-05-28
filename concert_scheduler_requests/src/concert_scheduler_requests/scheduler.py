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
.. module:: scheduler

Python interface for ROCON schedulers handling resource requests.

.. include:: weblinks.rst

"""

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import rospy
import threading
import unique_id

# ROS messages
from scheduler_msgs.msg import Request, SchedulerRequests

# internal modules
from . import common
from .transitions import ActiveRequest, RequestSet


class _RequesterStatus:
    """
    This class tracks the status of all resource requests made by a
    single requester.  It subscribes to the requester feedback topic,
    and provides updated information when appropriate.

    Being internal to the :class:`.Scheduler`, methods of this class
    are invoked already holding the :ref:`Big Scheduler Lock
    <Big_Scheduler_Lock>`.

    :param sched: (:class:`.Scheduler`) Scheduler object with which
        this requester is connected.

    :param msg: (scheduler_msgs/SchedulerRequests) Initial resource
        allocation requests.

    """

    def __init__(self, sched, msg):
        """ Constructor. """
        self.last_msg_time = rospy.Time.now()
        """ Local time of last message.  Avoid using message header
        time stamps because other Concert components may not be using
        synchronized clocks."""
        self.sched = sched
        """ Scheduler serving this requester. """
        self.requester_id = unique_id.fromMsg(msg.requester)
        """ :class:`uuid.UUID` of this requester. """
        self.rset = RequestSet(msg, contents=ActiveRequest)
        """ All active requests for this requester. """

        feedback_topic = common.feedback_topic(self.requester_id,
                                               self.sched.topic)
        rospy.loginfo('requester feedback topic: ' + feedback_topic)
        self.pub = rospy.Publisher(feedback_topic, SchedulerRequests,
                                   latch=True, queue_size=1)

    def contact(self):
        """ Contact newly-connected requester. """
        # Cancel any out-of-date requests the requester had lying around.
        self.rset.cancel_out_of_date(reason=Request.TIMEOUT)
        self.sched.callback(self.rset)  # handle initial message
        self.send_feedback()

    def send_feedback(self):
        """ Send feedback message to requester. """
        self.pub.publish(self.rset.to_msg())

    def update(self, msg):
        """ Update requester status.

        :param msg: Latest resource allocation request.
        :type msg: scheduler_msgs/SchedulerRequests

        """
        self.last_msg_time = rospy.Time.now()
        # Make a new RequestSet from this message
        new_rset = RequestSet(msg, contents=ActiveRequest)
        if self.rset != new_rset:       # something new?
            self.rset.merge(new_rset)
            self.sched.callback(self.rset)
            if self.rset != new_rset:   # still different?
                self.send_feedback()

    def timeout(self, limit, event):
        """ Check for requester timeout.

        :param limit: Time *limit* since last message received.
        :type limit: :class:`rospy.Duration`
        :param event: Current :class:`rospy.TimerEvent` object.
        :returns: True if *limit* exceeded, False if still active.

        """
        lost = (event.current_real - self.last_msg_time) > limit
        if lost:                # lost contact with this requester?
            # Cancel every active request, so the callback will
            # recover everything it had allocated.
            self.rset.cancel_all(reason=Request.TIMEOUT)
            self.sched.callback(self.rset)
            # No one left to notify.
        return lost


class Scheduler:
    """
    This class is used by a ROCON scheduler to manage all the resource
    requests sent by various ROCON services.  It subscribes to the
    ROCON scheduler topic, handling resource requests as they are
    received.

    :param callback: Callback function invoked with the updated
        :class:`.RequestSet` when requests arrive.

    :param frequency: requester heartbeat frequency in Hz.
    :type frequency: float
    :param topic: Topic name for resource allocation requests.  If
        ``None``, search for a ROS ``topic_name`` parameter, or else
        assume ``rocon_scheduler``.
    :type topic: str or ``None``
    :param lock: The big scheduler serialization lock, allocated
        internally, if ``None``.
    :type lock: :class:`.threading.RLock()`

    .. describe:: callback(rset)

       :param rset: (:class:`.RequestSet`) The current status of all
           requests for some active requester.

    The *callback* function is called when new or updated requests are
    received, already holding the :ref:`Big Scheduler Lock
    <Big_Scheduler_Lock>`.  It is expected to iterate over its
    :class:`.RequestSet`, checking the status of every
    :class:`.ActiveRequest` it contains, modifying them appropriately,
    then return without waiting.  The results will be sent to the
    requester after this callback returns.

    Usage example:

    .. literalinclude:: ../tests/example_scheduler.py

    """

    def __init__(self, callback, frequency=common.HEARTBEAT_HZ,
                 topic=None, lock=None):
        """ Constructor. """
        self.callback = callback
        """ Callback function for request updates. """
        if lock is None:
            lock = threading.RLock()
        self.lock = lock
        """
        .. _Big_Scheduler_Lock:

        Big Scheduler Lock.

        This recursive Python threading lock serializes access to
        scheduler data.  The scheduler *callback* is always invoked
        holding it.  All :class:`.Scheduler` methods acquire it
        automatically, whenever needed.

        In any other thread, acquire it when updating shared request
        set objects.  Never hold it when sleeping or waiting for I/O.
        """
        self.requesters = {}
        """ Dictionary of active requesters and their requests. """
        if topic is None:
            topic_param = rospy.search_param('topic_name')
            if topic_param is None:
                topic = common.SCHEDULER_TOPIC
            else:
                topic = rospy.get_param(topic_param)
        self.topic = topic
        """ Scheduler request topic name. """
        rospy.loginfo('scheduler request topic: ' + self.topic)
        self.sub = rospy.Subscriber(self.topic, SchedulerRequests,
                                    self._allocate_resources,
                                    queue_size=1, tcp_nodelay=True)
        self.duration = rospy.Duration(1.0 / frequency)
        self.time_limit = self.duration * 4.0
        self.timer = rospy.Timer(self.duration, self._watchdog)

    def _allocate_resources(self, msg):
        """ Scheduler resource allocation message handler. """
        with self.lock:
            rqr_id = unique_id.fromMsg(msg.requester)
            rqr = self.requesters.get(rqr_id)
            if rqr:                     # known requester?
                rqr.update(msg)
            else:                       # new requester
                rqr = _RequesterStatus(self, msg)
                self.requesters[rqr_id] = rqr
                rqr.contact()

    def _watchdog(self, event):
        """ Scheduler request watchdog timer handler. """
        # Must iterate over a copy of the dictionary items, because
        # some may be deleted inside the loop.
        with self.lock:
            for rqr_id, rqr in self.requesters.items():
                if rqr.timeout(self.time_limit, event):
                    del self.requesters[rqr_id]

    def notify(self, requester_id):
        """ Notify requester of status updates.

        :param requester_id: Requester to notify.
        :type requester_id: uuid.UUID

        :raises: :exc:`KeyError` if unknown requester identifier.

        """
        with self.lock:
            self.requesters[requester_id].send_feedback()
