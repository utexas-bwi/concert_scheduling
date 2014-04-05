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
from concert_msgs.msg import ConcertClients
from scheduler_msgs.msg import KnownResources

from .rapp_handler import (
    FailedToStartRappError, FailedToStopRappError, RappHandler)
from .resource_pool import (
    CurrentStatus, InvalidRequestError, ResourcePool)


class SchedulerClients(ResourcePool):
    """ Scheduler clients interface.

    :param lock: The big scheduler serialization lock.
    :param resource_pool: resource pool class to use, must provide a
        compatible :class:`.ResourcePool` interface.

    Provides all attributes defined for the base *resource_pool*
    class, plus these:
    """
    def __init__(self, lock, resource_pool=ResourcePool):
        """ Constructor. """
        super(SchedulerClients, self).__init__()
        self.lock = lock
        """ Big scheduler lock for serializing updates. """
        self.resource_pool = resource_pool
        """ Associated *resource_pool* instance. """
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

    def track_clients(self, msg):
        """ Concert clients message callback.

        Updates the resource pool based on client changes published by
        the concert conductor.

        Uses the Big Scheduler Lock to serialize changes with
        operations done within the scheduler callback thread.
        """
        with self.lock:
            self.update(msg.clients)
