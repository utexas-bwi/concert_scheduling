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

## ROS messages
from rocon_scheduler_requests.transitions import ResourceReply


class RequestQueueElement(object):
    """ Request queue element class.

    :param request: Corresponding scheduler request object.
    :type request: :class:`.ResourceReply`

    Different scheduling policies may want to provide a derived
    subclass.
    """
    def __init__(self, request):
        self.request = request
        """ Corresponding scheduler request object. """


class RequestQueue(object):
    """ This is a container class for a queue of requests to a ROCON_
    scheduler.  Different scheduling policies may provide derived
    subclasses.

    :param iterable: Initial queued requests.
    :type iterable: :class:`.ResourceReply` iterable

    .. describe:: len(queue)

       :returns: The number of requests in the *queue*.

    """
    def __init__(self, iterable=[]):
        self.queue = deque()
        """ FIFO queue of :class:`.ResourceReply` objects. """
        for request in iterable:
            self.append(request)

    def __len__(self):
        return len(self.queue)

    def append(self, request):
        """ Add newest element. """
        self.queue.append(RequestQueueElement(request))

    def appendleft(self, request):
        """ Add oldest element. """
        self.queue.appendleft(RequestQueueElement(request))

    def pop(self):
        """ Remove newest element.

        :raises: :exc:`IndexError` if queue was empty.
        """
        return self.queue.pop().request

    def popleft(self):
        """ Remove oldest element.

        :raises: :exc:`IndexError` if queue was empty.
        """
        return self.queue.popleft().request
