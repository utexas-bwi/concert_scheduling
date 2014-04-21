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
.. module:: transitions

This module tracks resource request state transitions, which occur as
`scheduler_msgs/Request`_ messages flow between schedulers and
requesters.

As individual ``Request`` messages are passed back and forth between
the original requester and the scheduler, their ``status`` passes
through several state transitions.  Gray states are created by the
scheduler via solid transitions.  The dashed **cancel** transitions
may be initiated by either the requester or the scheduler.

.. graphviz:: state_transitions.dot

.. include:: weblinks.rst

"""

# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

# Ros dependencies
import rospy
import unique_id

from scheduler_msgs.msg import Request, SchedulerRequests
from . import TransitionError

# Starting and terminal request states:
STARTING_STATES = frozenset([Request.NEW, Request.RESERVED])
TERMINAL_STATES = frozenset([Request.CLOSED])

# Printable name for each state, indexed by number.
STATE_NAME = ['NEW', 'RESERVED', 'WAITING', 'GRANTED',
              'PREEMPTING', 'CANCELING', 'CLOSED']

## State transition merge table.
#
#  An immutable set of (old, new) status pairs.  All pairs in the
#  table are considered valid state transitions.  Any others are not.
#
#  This table is only used for merging scheduler and requester request
#  sets when a new message arrives.  Data from the new message are
#  ignored unless the corresponding transition appears here.
#
TRANS_TABLE = frozenset([
    (Request.CANCELING, Request.CANCELING),
    (Request.CANCELING, Request.CLOSED),

    (Request.GRANTED, Request.CANCELING),
    (Request.GRANTED, Request.GRANTED),
    (Request.GRANTED, Request.PREEMPTING),

    (Request.NEW, Request.CANCELING),
    (Request.NEW, Request.GRANTED),
    (Request.NEW, Request.PREEMPTING),
    (Request.NEW, Request.WAITING),

    (Request.PREEMPTING, Request.CANCELING),
    (Request.PREEMPTING, Request.CLOSED),       # really??
    (Request.PREEMPTING, Request.PREEMPTING),

    (Request.RESERVED, Request.CANCELING),
    (Request.RESERVED, Request.GRANTED),
    (Request.RESERVED, Request.PREEMPTING),
    (Request.RESERVED, Request.RESERVED),
    (Request.RESERVED, Request.WAITING),

    (Request.WAITING, Request.CANCELING),
    (Request.WAITING, Request.GRANTED),
    (Request.WAITING, Request.PREEMPTING),
    (Request.WAITING, Request.WAITING)])


class _EventTranitions:
    """
    Define valid status transitions for a given event type.

    :param name: Human-readable name for this event type.
    :type name: str
    :param trans: Dictionary of valid status transitions.
    :type trans: dict

    """
    def __init__(self, name, trans):
        self.name = name
        """ Name of this event type. """
        self.trans = trans
        """ Dictionary of valid status transitions. """

##  Requester or scheduler transitions:
#
EVENT_CANCEL = _EventTranitions('cancel', {
    Request.CANCELING: Request.CANCELING,
    Request.CLOSED: Request.CLOSED,
    Request.GRANTED: Request.CANCELING,
    Request.NEW: Request.CANCELING,
    Request.PREEMPTING: Request.CANCELING,
    Request.RESERVED: Request.CANCELING,
    Request.WAITING: Request.CANCELING,
    })

##  Scheduler transitions:
#
EVENT_CLOSE = _EventTranitions('close', {
    Request.CANCELING: Request.CLOSED,
    Request.PREEMPTING: Request.CLOSED,
    })

EVENT_GRANT = _EventTranitions('grant', {
    Request.NEW: Request.GRANTED,
    Request.RESERVED: Request.GRANTED,
    Request.WAITING: Request.GRANTED,
    })

EVENT_PREEMPT = _EventTranitions('preempt', {
    Request.CANCELING: Request.CANCELING,
    Request.CLOSED: Request.CLOSED,
    Request.GRANTED: Request.PREEMPTING,
    Request.NEW: Request.NEW,
    Request.PREEMPTING: Request.PREEMPTING,
    Request.RESERVED: Request.RESERVED,
    Request.WAITING: Request.WAITING,
    })

EVENT_WAIT = _EventTranitions('wait', {
    Request.NEW: Request.WAITING,
    Request.RESERVED: Request.WAITING,
    Request.WAITING: Request.WAITING,
    })


class RequestBase(object):
    """
    Base class for tracking the status of a single resource request.

    *Not for general use.*

    :param msg: ROCON scheduler request message.
    :type msg: scheduler_msgs/Request

    Requesters and schedulers will use one of the following derived
    classes, with higher-level interfaces creating it automatically:

    * Requester: :class:`.ResourceRequest`
    * Scheduler: :class:`.ActiveRequest`

    .. describe:: str(rq)

       :returns: String representation of this resource request.

    """
    def __init__(self, msg):
        """ Constructor. """
        self.msg = msg
        """ Corresponding *scheduler_msgs/Request* message. """
        self.uuid = unique_id.fromMsg(msg.id)
        """ The :class:`uuid.UUID` of this request. """

    def cancel(self, reason=None):
        """ Cancel a previously-requested resource.

        :param reason: Reason code for cancellation, or ``None``.

        *Always valid for both requesters and schedulers.*
        """
        self._transition(EVENT_CANCEL, reason)

    def __str__(self):
        """ Generate string representation. """
        return 'id: ' + str(self.uuid) \
            + '\n    priority: ' + str(self.msg.priority) \
            + '\n    resources: ' + self._str_resources() \
            + '\n    status: ' + str(self.msg.status)

    def _str_resources(self):
        """ Format requested resource into a human-readable string. """
        retval = ''
        for res in self.msg.resources:
            retval += '\n      ' + res.uri + '#' + res.rapp
        return retval

    def _transition(self, event, reason=None):
        """
        Update status for this resource request.

        :param event: Transition table for this type of *event*.
        :type event: :class:`._EventTranitions`
        :param reason: Reason code for transition, or ``None``.
        :raises: :exc:`.TransitionError` if not a valid transition.
        """
        new_status = event.trans.get(self.msg.status)
        if new_status is None:
            raise TransitionError('invalid event ' + event.name
                                  + ' in state ' + STATE_NAME[self.msg.status])
        self.msg.status = new_status
        if reason is not None:
            self.msg.reason = reason

    def _validate(self, new_status):
        """
        Validate status update for this resource request.

        :param new_status: Proposed new status for this request.
        :returns: ``True`` if this is a valid state transition.
        """
        return (self.msg.status, new_status) in TRANS_TABLE


class ResourceRequest(RequestBase):
    """
    This represents a single resource request created by and for its
    original requester.

    :param msg: ROCON scheduler request message.
    :type msg: scheduler_msgs/Request

    Provides all attributes defined for :class:`.RequestBase`.
    """
    def reconcile(self, update):
        """
        Reconcile scheduler updates with requester status for a merge
        operation.

        :param update: Latest information for this request, or
            ``None`` if no longer present.
        :type update: :class:`.ResourceRequest` or ``None``

        Only the requester creates new requests.  If something is
        missing from the scheduler feedback, that just means the
        scheduler has not gotten to it yet.
        """
        if update is None:      # this request not yet known to scheduler?
            return              # leave it alone
        if self._validate(update.msg.status):
            self.msg.status = update.msg.status
            self.msg.priority = update.msg.priority
            self.msg.resources = update.msg.resources
            if update.msg.availability != rospy.Time():
                self.msg.availability = update.msg.availability  # test gap


class ActiveRequest(RequestBase):
    """
    This represents a single active resource known to the scheduler.

    :param msg: ROCON scheduler request message.
    :type msg: scheduler_msgs/Request

    Provides all attributes defined for :class:`.RequestBase`.
    """
    def __init__(self, msg):
        """ Constructor """
        super(ActiveRequest, self).__init__(msg)
        self.allocations = []
        """ List of resources actually allocated for this request (not
        just those requested). """

    def close(self):
        """ Close resource request.

        :raises: :exc:`.TransitionError`
        """
        self._transition(EVENT_CLOSE)

    def grant(self, resources):
        """ Grant some specific requested resources.

        :param resources: Exact resources granted.
        :type resources: list of ``scheduler_msgs/Resource``
        :raises: :exc:`.TransitionError`

        The caller is responsible for ensuring that the granted
        resources really do fully satisfy this request.

        """
        self._transition(EVENT_GRANT, reason=Request.NONE)
        self.msg.resources = resources
        self.allocations = resources

    def reconcile(self, update):
        """
        Reconcile updated request with current scheduler status for a
        merge operation.

        :param update: Latest information for this request, or
            ``None`` if no longer present.
        :type update: :class:`.ActiveRequest` or ``None``
        """
        # test gap:
        if update is None:
            # Only the requester creates new requests.  Since no
            # update is present, the requester has either deleted this
            # request, or a new requester instance no longer knows
            # about it.  So, consider it closed.
            update = ResourceRequest(self.msg)
            update.msg.status = Request.CLOSED
        if self._validate(update.msg.status):
            self.msg.status = update.msg.status
            self.msg.hold_time = update.msg.hold_time
            if (update.msg.status == Request.RESERVED
                    and update.msg.availability != rospy.Time()):
                self.msg.availability = update.msg.availability

    def preempt(self, reason=Request.NONE):
        """ Preempt a previously granted request.

        :param reason: Reason for preemption.
        :type reason: int

        Always valid for the scheduler, but has no effect unless the
        request was previously granted.

        """
        if self.msg.status != Request.GRANTED:
            reason = self.msg.reason    # leave reason unchanged
        self._transition(EVENT_PREEMPT, reason)

    def wait(self, reason=Request.NONE):
        """
        Put request in wait status until a suitable resource is available.

        :param reason: Reason for waiting.
        :type reason: int
        :raises: :exc:`.TransitionError`
        """
        self._transition(EVENT_WAIT, reason)


class RequestSet:
    """
    This class is a container for all the resource requests or
    responses for a single requester.  It acts like a dictionary.

    :param reqs: Either a ``SchedulerRequests`` message or a list of
        ``Request`` messages, like the ``requests`` component of a
        ``SchedulerRequests`` message.
    :param requester_id: (:class:`uuid.UUID`) Unique ID this requester
        or ``None``.
    :param contents: Class from which to instantiate set members,
        either :class:`.ResourceRequest` (the default) for a requester
        or :class:`.ActiveRequest` for a scheduler.
    :raises: :exc:`TypeError` if the *requester_id* is not specified
        explicitly or as part of a ``SchedulerRequests`` message.

    :class:`.RequestSet` supports these standard container operations:

    .. describe:: len(rset)

       :returns: The number of requests in the set.

    .. describe:: rset[uuid]

       :returns: The item corresponding to *uuid*.
       :raises: :exc:`KeyError` if no such request.

    .. describe:: rset[uuid] = msg

       Assign a Request message for this *uuid*.

       :param uuid: (:class:`uuid.UUID`) UUID of the request.
       :param msg: (*scheduler_msgs/Request*) message to add.

    .. describe:: rset == other

       :returns: ``True`` if *rset* and *other* have the same contents.

        Ignores the difference between request and reply messages.

    .. describe:: rset != other

       :returns: ``True`` if *rset* and *other* have different contents.

        Ignores the difference between request and reply messages.

    .. describe:: str(rset)

       :returns: String representation of :class:`.RequestSet`.

    .. describe:: uuid in rset

       :returns: ``True`` if *rset* has a key *uuid*, else ``False``.

    .. describe:: uuid not in rset

       Equivalent to ``not uuid in rset``.

    These attributes are also provided:

    """

    def __init__(self, reqs, requester_id=None, contents=ResourceRequest):
        """ Constructor. """
        self.requester_id = requester_id
        """ :class:`uuid.UUID` of this requester. """
        self.contents = contents
        """ Type of objects this request set contains. """
        self.stamp = rospy.Time()       # zero time stamp
        """ ROS time (:class:`rospy.Time`) of last update, or time zero. """

        # reqs is either a SchedulerRequests message, or a list of
        # Request messages.
        if isinstance(reqs, SchedulerRequests):
            self.stamp = reqs.header.stamp
            self.requester_id = unique_id.fromMsg(reqs.requester)
            # Reset *reqs* to list of requests from the message.
            reqs = reqs.requests
        if self.requester_id is None:
            raise TypeError('Requester ID missing.')

        self.requests = {}
        """ Dictionary of active requests. """
        for msg in reqs:
            rq = self.contents(msg)
            self.requests[rq.uuid] = rq

    def __contains__(self, uuid):
        """ Request set membership. """
        return uuid in self.requests

    def __eq__(self, other):
        """ RequestSet equality operator. """
        if self.requester_id != other.requester_id:
            return False        # different requester
        if set(self.requests.keys()) != set(other.requests.keys()):
            return False        # different request IDs
        for rqid, rq in self.requests.items():
            other_msg = other[rqid].msg
            if rq.msg.status != other_msg.status:
                return False
            if rq.msg.priority != other_msg.priority:
                return False
            if rq.msg.availability != other_msg.availability:
                return False
            if rq.msg.hold_time != other_msg.hold_time:
                return False
            if rq.msg.resources != other_msg.resources:
                return False
        return True

    def __getitem__(self, uuid):
        """
        :param uuid: UUID of desired request.
        :type uuid: :class:`uuid.UUID`

        :returns: named item.
        :raises: :exc:`KeyError` if no such request
        """
        return self.requests[uuid]

    def __len__(self):
        """ Number of requests. """
        return len(self.requests)

    def __ne__(self, other):
        """ RequestSet != operator. """
        return not self == other

    def __setitem__(self, uuid, msg):
        """ Assign a Request message for this *uuid*. """
        self.requests[uuid] = self.contents(msg)  # test gap

    def __str__(self):
        rval = 'requester_id: ' + str(self.requester_id) + '\nrequests:'
        for rq in self.requests.values():
            rval += '\n  ' + str(rq)
        return rval

    def cancel_all(self, reason=None):
        """ Cancel every active request in this set.

        :param reason: Reason code for mass cancellation, or ``None``.
        """
        # test gap:
        for rq in self.requests.values():
            rq.cancel(reason=reason)

    def cancel_out_of_date(self, reason=None):
        """ Cancel every out-of-date request in this set.

        Only requests in a starting state are preserved.  This is done
        whenever a requester first connects to the scheduler.  If it
        presents requests that had previously been granted or
        preempted, they will be canceled and then closed.

        :param reason: Reason code for mass cancellation, or ``None``.
        """
        # test gap:
        for rq in self.requests.values():
            if rq.msg.status not in STARTING_STATES:
                rq.cancel(reason=reason)

    def get(self, uuid, default=None):
        """ Get request, if known.

        :param uuid: UUID of desired request.
        :type uuid: :class:`uuid.UUID`
        :param default: value to return if no such request.

        :returns: named item, if successful; else *default*.

        """
        return self.requests.get(uuid, default)

    def items(self):
        """
        :returns: all (key, value) pairs for this :class:`.RequestSet`.
        :rtype: list (Python2) or dictionary view (Python3)

        """
        return self.requests.items()

    def keys(self):
        """
        :returns: all UUIDs for this :class:`.RequestSet`.
        :rtype: list (Python2) or dictionary view (Python3)

        """
        return self.requests.keys()     # test gap

    def _list_requests(self):
        """
        Return a list of resource requests suitable for inclusion in
        a ``SchedulerRequests`` message.

        :returns: list of *scheduler_msgs/Request* messages.

        """
        return [rq.msg for rq in self.requests.values()]

    def merge(self, updates):
        """
        Merge new request information into this RequestSet.

        :param updates: Request set containing updated information.
        :type updates: :class:`.RequestSet`

        This is *not* a :py:meth:`set.update` or :py:meth:`set.union`
        operation:

        * New elements from the *updates* will be added, but only if
          they are in an initial state (NEW or RESERVED).

        * Existing elements will be reconciled with the corresponding
          *updates* status.

        * Any element reaching a terminal status known by both sides
          of the protocol will be deleted.

        """
        # Add any new requests not previously known.
        for rid, new_rq in updates.items():
            if (rid not in self.requests and
                    new_rq.msg.status in STARTING_STATES):
                self.requests[rid] = self.contents(new_rq.msg)  # test gap

        # Reconcile each existing request with the updates.  Make a
        # copy of the dictionary items, so it can be altered in the loop.
        for rid, rq in self.requests.items():
            new_rq = updates.get(rid)
            if ((rq.msg.status == Request.CANCELING and
                    new_rq.msg.status == Request.CLOSED)
                    or (rq.msg.status == Request.CLOSED and
                        new_rq is None)):
                del self.requests[rid]  # no longer needed
            else:
                rq.reconcile(new_rq)

    def to_msg(self, stamp=None):
        """ Convert to ROS ``scheduler_msgs/SchedulerRequest`` message.

        :param stamp: Time stamp for message header. If ``None``, use
            current time.
        :type stamp: rospy.Time

        :returns: corresponding ``scheduler_msgs/SchedulerRequests``

        """
        msg = SchedulerRequests(requester=unique_id.toMsg(self.requester_id),
                                requests=self._list_requests())
        if stamp is None:
            stamp = rospy.Time.now()    # test gap
        msg.header.stamp = stamp
        return msg

    def values(self):
        """
        :returns: all requests for this :class:`.RequestSet`.
        :rtype: list (Python2) or dictionary view (Python3)

        """
        return self.requests.values()   # test gap
