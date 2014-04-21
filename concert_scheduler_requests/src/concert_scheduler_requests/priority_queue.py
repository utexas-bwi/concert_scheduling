# Software License Agreement (BSD License)
#
# Copyright (C) 2014, Jack O'Quin
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
.. module:: priority_queue

This module provides queue containers for scheduler requests for the
`Robotics in Concert`_ (ROCON) project.

.. include:: weblinks.rst

"""
# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import copy
import heapq
import itertools


class PriorityQueue(object):
    """ This is a container class for ROCON_ scheduler request queue elements.

    :param iterable: Iterable yielding initial contents, either
        :class:`.QueueElement` objects, or something that behaves
        similarly.

    This implementation is based on the :py:mod:`heapq` module and
    uses some of the ideas explained in its `priority queue
    implementation notes`_.

    .. describe:: len(queue)

       :returns: The number of elements in the *queue*.

    .. describe:: request in queue

       :param request: (:class:`uuid.UUID` or :class:`.QueueElement`)
           *request* to query.
       :returns: ``True`` if *request* is in the *queue*.

    """
    def __init__(self, iterable=[]):
        self._queue = []
        """ Priority queue of :class:`.QueueElement`. """
        self._requests = {}
        """ Dictionary of queued requests. """
        for element in iterable:
            self.add(element)

    def __contains__(self, request):
        return hash(request) in self._requests

    def __len__(self):
        return len(self._requests)

    def __str__(self):
        contents = sorted(self.values())
        rval = 'queue: '
        for elem in contents:
            rval += '\n ' + str(elem)
        return rval

    def add(self, element, priority=None):
        """ Add a new *element* to the queue.

        :param element: Queue *element* to add.
        :type element: :class:`.QueueElement`
        :param priority: (Optional) new priority for this *element*.
        :type priority: int

        If a request with the same identifier was already in the
        queue, its old element is removed and replaced by the new
        *element*.

        If a new *priority* is specified, the priority of the original
        request is updated.  That is the only safe way to change the
        priority of an *element* that is already queued.

        .. warning:: Changing priority via some other name for that
           request would break the queue implementation.
        """
        if hash(element) in self._requests:  # already in the queue?
            self.remove(element)        # mark that copy inactive
        element = copy.copy(element)
        element.active = True
        if priority is not None:
            element.request.msg.priority = priority
        self._requests[hash(element)] = element
        heapq.heappush(self._queue, element)

    def peek(self):
        """ Return the top-priority element from the queue head
        without removing it.

        :raises: :exc:`IndexError` if queue was empty.
        """
        # Return the top element that was not previously removed.
        for element in self._queue:
            if element.active:          # not previously removed?
                return element
        raise IndexError('peek at an empty priority queue')

    def pop(self):
        """ Remove the top-priority element from the queue head.

        :raises: :exc:`IndexError` if queue was empty.
        """
        # Return the top element that was not previously removed.
        while self._queue:
            element = heapq.heappop(self._queue)
            if element.active:          # not previously removed?
                del self._requests[hash(element)]
                return element
        raise IndexError('pop from an empty priority queue')

    def remove(self, request_id):
        """ Remove element corresponding to *request_id*.

        :param request_id: Identifier of the request to remove.
        :type request_id: :class:`uuid.UUID` or :class:`.QueueElement`
        :raises: :exc:`KeyError` if *request_id* not in the queue.
        """
        # Remove it from the dictionary and mark it inactive, but
        # leave it in the queue to avoid re-sorting.
        element = self._requests.pop(hash(request_id))
        element.active = False

    def values(self):
        """ Current queue contents.
        :returns: iterable with active queue elements in random order.
        """
        return self._requests.values()


class QueueElement(object):
    """ Request queue element class.

    :param request: Corresponding scheduler request object.
    :type request: :class:`.ActiveRequest`
    :param requester_id: Unique identifier of requester.
    :type requester_id: :class:`uuid.UUID`

    Queue elements need fit into normal Python dictionaries, so they
    provide the required ``hash()`` operator, based on the unique ID
    of that *request*.

    .. describe:: hash(element)

       :returns: (int) Hash signature for *element*.

    Python 3 requires that hashable objects must also provide an
    equals operator.  The hash signatures of equal requests must be
    equal.

    .. describe:: element == other

       :returns: ``True`` if *element* and *other* have the same
           *request* ID (*not* their *requester_id* values).

    .. describe:: element != other

       :returns: ``True`` if *element* and *other* do not have the
           same *request* ID.

    Queue elements need to sort in the normal Python way, so they
    provide the required ``<`` operator.  The ``__cmp__`` method is
    not used, because Python 3 does not allow it.  **But**, we want
    requests with higher-numbered priorities to sort *ahead* of lower
    priority ones, so :py:mod:`heapq` and other Python modules work
    properly.

    .. describe:: element < other

       :returns: ``True`` if *element* has higher priority than *other*, or
           their priorities are the same and *element* has a lower sequence
           number.

    This class does *not* provide a total ordering.  The ``==`` and
    ``<`` operators test completely different fields.  However, the
    *request* identifiers are unique, so no two valid queue elements
    should ever compare both equal and less, although that situation
    could be constructed artificially.

    .. describe:: str(element)

       :returns: String representation of this queue element, empty if
           it is not active.

    """
    _sequence = itertools.count()
    """ Class variable: next available sequence number. """

    def __init__(self, request, requester_id):
        self.request = request
        """ Corresponding scheduler :class:`ActiveRequest` object. """
        self.requester_id = requester_id
        """ :class:`uuid.UUID` of requester. """
        self.sequence = next(self.__class__._sequence)
        """
        Unique sequence number of this queue element.  All elements
        created earlier have lower numbers, those created afterward
        will be higher.
        """
        self.active = True
        """ ``True`` unless this element has been removed from its queue. """

    def __eq__(self, other):
        return self.request.msg.id == other.request.msg.id

    def __hash__(self):
        return hash(self.request.uuid)

    def __lt__(self, other):
        return (self.request.msg.priority > other.request.msg.priority
                or (self.request.msg.priority == other.request.msg.priority
                    and self.sequence < other.sequence))

    def __ne__(self, other):
        return self.request.msg.id != other.request.msg.id

    def __str__(self):
        """ Generate string representation. """
        if not self.active:
            return ''
        return 'id: ' + str(self.requester_id) \
            + '\n  seq: ' + str(self.sequence) \
            + '\n  request: ' + str(self.request)
