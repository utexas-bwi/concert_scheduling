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
.. module:: request_queue

This module provides queue containers for scheduler requests for the
`Robotics in Concert`_ (ROCON) project.

.. _`Robotics in Concert`: http://www.robotconcert.org/wiki/Main_Page
.. _ROCON: http://www.robotconcert.org/wiki/Main_Page

"""
from collections import deque


class QueueElement(object):
    """ Request queue element class.

    :param request: Corresponding scheduler request object.
    :type request: :class:`.ResourceReply`
    :param requester_id: Unique identifier of requester.
    :type requester_id: :class:`uuid.UUID`

    Queue elements need fit into normal Python dictionaries, so they
    provide the required ``hash()`` operator, based on the unique ID
    of that *request*.

    .. describe:: hash(qe)

       :returns: (int) Hash signature for *qe*.

    Python 3 requires that hashable objects must also provide an
    equals operator.  The hash signatures of equal requests must be
    equal.

    .. describe:: qe == other

       :returns: ``True`` if the *qe* and *other* have the same
           *request* ID (*not* their *requester_id* values).

    Queue elements need to sort in the normal Python way, so they
    provide the required ``<`` operator.  The ``__cmp__`` method is
    not used, because Python 3 does not allow it.  **But**, we want
    requests with higher-numbered priorities to sort *ahead* of lower
    priority ones, so :py:mod:`heapq` and other Python modules work
    properly.

    .. describe:: qe < other

       :returns: ``True`` if *qe* has higher priority than *other*, or
           their priorities are the same and *qe* has a lower sequence
           number.
    
    This class does *not* provide a total ordering.  The ``==`` and
    ``<`` operators test completely different fields.  However, the
    *request* identifiers are unique, so no two queue elements should
    ever compare both equal and less.

    """
    _sequence = 0
    """ Class variable: sequence number of next instance. """

    def __init__(self, request, requester_id):
        self.request = request
        """ Corresponding scheduler request object. """
        self.requester_id = requester_id
        """ :class:`uuid.UUID` of requester. """
        self.sequence = self.__class__._sequence
        """ Unique sequence number of this element. """
        self.__class__._sequence += 1

    def __eq__(self, other):
        return self.request.msg.id == other.request.msg.id

    def __hash__(self):
        return hash(self.request.get_uuid())

    def __lt__(self, other):
        return (self.request.msg.priority > other.request.msg.priority
                or (self.request.msg.priority == other.request.msg.priority
                    and self.sequence < other.sequence))


class RequestQueue(object):
    """ This is a container class for a queue of tuples describing
    ROCON_ scheduler requests.

    Different scheduling policies may provide derived subclasses.

    :param iterable: Iterable yielding initial :class:`.QueueElement` objects.

    .. describe:: len(queue)

       :returns: The number of elements in the *queue*.

    """
    def __init__(self, iterable=[]):
        self._queue = deque(iterable)
        """ FIFO queue of :class:`.QueueElement`. """

    def __len__(self):
        return len(self._queue)

    def append(self, element):
        """ Add *element* to tail of queue. """
        self._queue.append(element)

    def appendleft(self, element):
        """ Add *element* to head of queue. """
        self._queue.appendleft(element)

    def pop(self):
        """ Remove element at queue tail.

        :raises: :exc:`IndexError` if queue was empty.
        """
        return self._queue.pop()

    def popleft(self):
        """ Remove element at queue head.

        :raises: :exc:`IndexError` if queue was empty.
        """
        return self._queue.popleft()
