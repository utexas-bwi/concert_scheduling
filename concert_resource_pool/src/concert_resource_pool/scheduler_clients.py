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
.. module:: scheduler_clients

This module handles the ROS interfaces for client resources managed by
a scheduler for the `Robotics in Concert`_ (ROCON) project.

It wraps the :mod:`resource_pool` classes, adding ROS topic and
service interfaces to its basic resource pool allocation and release.
Derived classes may override those interfaces with different resource
allocation policies.

Subscribes:

 * ``concert_client_changes`` (`concert_msgs/ConcertClients`_)
   containing updated client information known to the Conductor.

Publishes:

 * ``resource_pool`` (`scheduler_msgs/KnownResources`_) containing the
   current status of all resources known to the Scheduler.

.. include:: weblinks.rst

"""
from __future__ import absolute_import, print_function, unicode_literals

import rospy
import threading

from concert_msgs.msg import ConcertClients
from scheduler_msgs.msg import KnownResources

from .rapp_handler import (
    FailedToStartRappError, FailedToStopRappError, RappHandler)
from .resource_pool import (
    CurrentStatus, InvalidRequestError, PoolResource, ResourcePool)


class SchedulerResource(PoolResource):
    """ Scheduler clients interface.

    :param msg: ROCON resource description message.
    :type msg: ``concert_msgs/ConcertClient``

    Provides all attributes defined for the base *pool_resource*
    class, plus these:
    """
    def __init__(self, msg):
        self.rapp_handler = RappHandler(msg)
        """ Handler for starting and stopping rapps on this resource. """
        super(SchedulerResource, self).__init__(msg)

    def release(self, request_id=None):
        """ Release this resource and stop any running rapps.

        :param request_id: Optional owning request.
        :type request_id: :class:`uuid.UUID` or ``None``

        :raises: :exc:`.ResourceNotOwnedError` if *request_id* is
            specified and is not the owner.
        :raises: :exc:`.FailedToStopRappError` if the associated
            client rapp does stop when requested.
        """
        super(SchedulerResource, self).release(request_id)
        try:
            self.rapp_handler.stop()    # stop any running rapps
        except FailedToStopRappError as e:
            rospy.logerr(str(e))


class SchedulerClients(ResourcePool):
    """ Scheduler clients interface.

    :param lock: The big scheduler serialization lock, allocated
        internally, if ``None``.
    :type lock: :class:`.threading.RLock()`
    :param resource_pool: resource pool class to use, must provide a
        compatible :class:`.ResourcePool` interface.

    Provides all attributes defined for the base *resource_pool*
    class, plus these:
    """
    def __init__(self, lock=None,
                 resource_pool=ResourcePool,
                 pool_resource=SchedulerResource):
        """ Constructor. """
        super(SchedulerClients, self).__init__(pool_resource=pool_resource)
        if lock is None:
            lock = threading.RLock()
        self.lock = lock
        """ Big scheduler lock for serializing updates. """
        self._pub = rospy.Publisher('resource_pool', KnownResources,
                                    queue_size=1, latch=True)
        self._pub.publish(self.known_resources())
        self._sub = rospy.Subscriber('concert_client_changes',
                                     ConcertClients, self.track_clients,
                                     queue_size=1, tcp_nodelay=True)

    def notify_resources(self):
        """ Update ``resource_pool`` topic, if anything changed. """
        if self.changed:
            self._pub.publish(self.known_resources())

    def start_resources(self, resources):
        """ Start rapps specified by the resources list.

        :param resources: List of ``scheduler_msgs/Resource`` messages.

        :raises: :exc:`.FailedToStartRappError`
        """
        for res in resources:
            pool_res = self.pool[res.uri]
            pool_res.rapp_handler.start(res.rapp, res.remappings)

    def track_clients(self, msg):
        """ Concert clients message callback.

        Updates the resource pool based on client changes published by
        the concert conductor.

        Uses the Big Scheduler Lock to serialize changes with
        operations done within the scheduler callback thread.
        """
        with self.lock:
            self.update(msg.clients + msg.missing_clients)
