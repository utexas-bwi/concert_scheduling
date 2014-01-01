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
.. module:: resource_pool

This module tracks all known resources managed by this scheduler.  The ROS
`scheduler_msgs/Resource`_ message describes resources used by the
`Robotics in Concert`_ (ROCON) project.

.. _ROCON: http://www.robotconcert.org/wiki/Main_Page
.. _`Robotics in Concert`: http://www.robotconcert.org/wiki/Main_Page
.. _`scheduler_msgs/KnownResources`:
    http://docs.ros.org/api/scheduler_msgs/html/msg/KnownResources.html

"""
import copy
from itertools import chain, islice, permutations

## ROS messages
from scheduler_msgs.msg import Resource
try:
    from scheduler_msgs.msg import CurrentStatus, KnownResources
except ImportError:
    from rocon_scheduler_requests.resources import CurrentStatus, KnownResources

from rocon_scheduler_requests.resources import ResourceSet


class ResourcePool:
    """ This class tracks a pool of resources managed by this scheduler.

    :param resources: Initial resources for the pool.
    :type resources: :class:`.ResourceSet` or ``None``

    """
    def __init__(self, resources=None):
        self.pool = resources
        """ :class:`.ResourceSet` of current resource pool contents. """
        if resources is None:
            self.pool = ResourceSet()   # pool initially empty

    def allocate(self, request):
        """ Try to allocate all resources for a *request*.

        :param request: Scheduler request message, some resources may
            include regular expression syntax.
        :type request: ``scheduler_msgs/Request``

        :returns: List of ``scheduler_msgs/Resource`` messages
            allocated, in requested order with platform info fully
            resolved; or ``[]`` if not everything is available.

        If successful, matching ROCON resources are allocated to this
        *request*.  Otherwise, the *request* remains unchanged.

        """
        n_wanted = len(request.resources)  # number of resources wanted

        # Make a list containing sets of the available resources
        # matching each requested item.
        matches = self.match_list(request.resources)
        if not matches:                 # unsuccessful?
            return []                   # give up

        # See if there are as least as many different resources in the
        # matches set as the number requested.
        match_union = set(chain.from_iterable(matches))
        if len(match_union) < n_wanted:
            return []                   # not enough stuff

        # At least one resource is available that satisfies each item
        # requested.  Try to allocate them all in the order requested.
        alloc = self._allocate_permutation(range(n_wanted), request.resources)
        if alloc:                       # successful?
            # allocate to this request
            return alloc
        if n_wanted > 3:                # lots of permutations?
            return []                   # give up

        # Look for some other permutation that satisfies them all.
        for perm in islice(permutations(range(n_wanted)), 1, None):
            alloc = self._allocate_permutation(perm, request.resources)
            if alloc:                   # successful?
                # allocate to this request
                return alloc
        return []                       # failure

    def _allocate_permutation(self, perm, resources):
        #print(str(perm) + '\nresources:\n' + str(resources))
        return copy.deepcopy(resources)  # copy fake results

    def match_list(self, resources):
        """
        Make a list containing sets of the available resources
        matching each element of *resources*.

        *What if list is empty?*

        :returns: List of :class:`set` containing names of matching
            resources, empty if any item cannot be satisfied.
        """
        matches = []
        for res_req in resources:
            match_set = self.match_subset(res_req)
            if len(match_set) == 0:     # no matches for this resource?
                return []               # give up
            matches.append(match_set)
        return matches

    def match_subset(self, resource_request):
        """ Find all resources matching *resource_request*.

        :param resource_request: Resource message from scheduler Request.
        :type resource_request: ``scheduler_msgs/Resource``

        :returns: :class:`set` containing matching resource names.
        """
        avail = set()
        for res in self.pool.resources.values():
            if (res.status == CurrentStatus.AVAILABLE
                    and res.match(resource_request)):
                avail.add(res.platform_info)
        return avail

    def release(self, resources):
        """ Release all the *resources* in this list. """
        pass                            # stub, nothing released
