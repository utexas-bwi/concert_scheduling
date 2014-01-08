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

.. include:: weblinks.rst

"""
from __future__ import absolute_import, print_function, unicode_literals

import copy
from itertools import chain, islice, permutations
import re

## ROS messages
from scheduler_msgs.msg import Resource
try:
    from scheduler_msgs.msg import CurrentStatus
except ImportError:
    class CurrentStatus:
        # provide a stub for the CurrentStatus message:
        AVAILABLE, ALLOCATED, MISSING = range(3)

        def __init__(self, platform_info='', rapps=[]):
            self.platform_info = platform_info
            self.rapps = rapps
            self.status = CurrentStatus.AVAILABLE
            self.owner = None
try:
    from scheduler_msgs.msg import KnownResources
except ImportError:
    class KnownResources:
        # provide a stub for the KnownResources message:
        def __init__(self, resources=[]):
            self.resources = resources


## Exceptions
class ResourceNotAvailableError(Exception):
    """ Error exception: resource not available. """
    pass


class ResourceNotOwnedError(Exception):
    """ Error exception: resource not owned. """
    pass


def rocon_name(platform_info):
    """ Generate canonical ROCON resource name.

    :param platform_info: Platform info string from a
        :class:`.PoolResource`, ``scheduler_msgs/Resource`` message,
        or other resource representation.

    :returns: (str) Canonical ROCON name for this resource.

    A fully-resolved canonical name uniquely describes each resource
    within a ROCON_ Concert.  Some requests may match multiple
    resources by embedding regular expression syntax in the name.

    If the *platform_info* is not already a ROCON name starting with
    'rocon://', assume it may use the dotted syntax and convert shell
    wildcard asterisks to the equivalent Python regular expression.

    """
    if platform_info[0:8] == 'rocon://':
        return platform_info            # already canonical

    # Assume dotted representation, convert that to canonical format.
    retval = 'rocon://'
    for part in platform_info.split('.'):
        if part == '*':                 # shell wildcard syntax?
            part = '\\.*'               # convert to Python RE
        retval += '/' + part
    return retval


class ResourcePool(object):
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

        :param request: Scheduler request object, some resources may
            include regular expression syntax.
        :type request: :class:`.ResourceReply`

        :returns: List of ``scheduler_msgs/Resource`` messages
            allocated, in requested order with platform info fully
            resolved; or ``[]`` if not everything is available.

        If successful, matching ROCON resources are allocated to this
        *request*.  Otherwise, the *request* remains unchanged.

        """
        n_wanted = len(request.msg.resources)  # number of resources wanted

        # Make a list containing sets of the available resources
        # matching each requested item.
        matches = self._match_list(request.msg.resources)
        if not matches:                 # unsuccessful?
            return []                   # give up

        # See if there are as least as many different resources in the
        # matches set as the number requested.
        match_union = set(chain.from_iterable(matches))
        if len(match_union) < n_wanted:
            return []                   # not enough stuff

        # At least one resource is available that satisfies each item
        # requested.  Try to allocate them all in the order requested.
        alloc = self._allocate_permutation(range(n_wanted), request, matches)
        if alloc:                       # successful?
            return alloc
        if n_wanted > 3:                # lots of permutations?
            return []                   # give up

        # Look for some other permutation that satisfies them all.
        for perm in islice(permutations(range(n_wanted)), 1, None):
            alloc = self._allocate_permutation(perm, request, matches)
            if alloc:                   # successful?
                return alloc
        return []                       # failure

    def _allocate_permutation(self, perm, request, matches):
        """ Try to allocate some permutation of resources for a *request*.

        :param perm: List of permuted resource indices for this
            *request*, like [0, 1, 2] or [1, 2, 0].
        :param request: Scheduler request object, some resources may
            include regular expression syntax.
        :type request: :class:`.ResourceReply`
        :param matches: List containing sets of the available
            resources matching each element of *request.msg.resources*.
        :returns: List of ``scheduler_msgs/Resource`` messages
            allocated, in requested order with platform info fully
            resolved; or ``[]`` if not everything is available.

        If successful, matching ROCON resources are allocated to this
        *request*.  Otherwise, the *request* remains unchanged.

        """
        # Copy the list of Resource messages and all their contents.
        alloc = copy.deepcopy(request.msg.resources)

        # Search in permutation order for some valid allocation.
        names_allocated = set([])
        for i in perm:
            # try each matching name in order
            for name in matches[i]:
                if name not in names_allocated:  # still available?
                    names_allocated.add(name)
                    alloc[i].platform_info = name
                    break               # go on to next resource
            else:
                return []               # failure: no matches work

        # successful: allocate to this request
        req_id = request.get_uuid()
        for resource in alloc:
            self.pool[resource.platform_info].allocate(req_id)
        return alloc                    # success

    def _match_list(self, resources):
        """
        Make a list containing sets of the available resources
        matching each element of *resources*.

        *What if list is empty?*

        :returns: List of :class:`set` containing names of matching
            resources, empty if any item cannot be satisfied.
        """
        matches = []
        for res_req in resources:
            match_set = self._match_subset(res_req)
            if len(match_set) == 0:     # no matches for this resource?
                return []               # give up
            matches.append(match_set)
        return matches

    def _match_subset(self, resource_msg):
        """
        Make a set of names of all available resources matching *resource_msg*.

        :param resource_msg: Resource message from a scheduler Request.
        :type resource_msg: ``scheduler_msgs/Resource``

        :returns: :class:`set` containing matching resource names.
        """
        avail = set()
        for res in self.pool.resources.values():
            if (res.status == CurrentStatus.AVAILABLE
                    and res.match(resource_msg)):
                avail.add(res.platform_info)
        return avail

    def release_request(self, request):
        """ Release all the resources owned by this *request*.

        :param request: Current owner of resources to release.
        :type request: :class:`.ResourceReply`

        Only appropriate when this *request* is being closed.
        """
        rq_id = request.get_uuid()
        for res in request.allocations:
            self.pool[res.platform_info].release(rq_id)

    def release_resources(self, resources):
        """ Release a list of *resources*.

        :param resources: List of ``scheduler_msgs/Resource`` messages.

        This makes newly allocated *resources* available again when
        they cannot be assigned to a request for some reason.
        """
        for res in resources:
            pool_res = self.pool[res.platform_info]
            pool_res.release()


class PoolResource:
    """
    Class for tracking the status of a single ROCON_ resource.

    :param msg: ROCON scheduler resource message.
    :type msg: scheduler_msgs/CurrentStatus or scheduler_msgs/Resource

    .. describe:: hash(res)

       :returns: Hash key for this resource.

    .. describe:: res == other

       :returns: ``True`` if this :class:`.PoolResource` equals the *other*.

    .. describe:: res != other

       :returns: ``True`` if this :class:`.PoolResource` differs from
           the *other*.

    .. describe:: str(res)

       :returns: Human-readable string representation of this
           :class:`.PoolResource`.

    These attributes are also provided:

    """
    def __init__(self, msg):
        """ Constructor. """
        self.platform_info = rocon_name(msg.platform_info)
        """ Fully-resolved canonical ROCON resource name. """
        try:
            self.rapps = set(msg.rapps)
            """ The :class:`set` of ROCON application name strings
            this platform advertises. """
        except AttributeError:
            self.rapps = set([msg.name])
        self.owner = None
        """ :class:`uuid.UUID` of request to which this resource is
        currently assigned, or ``None``.
        """
        self.status = CurrentStatus.AVAILABLE
        """ Current status of this resource. """

    def __eq__(self, other):
        if self.platform_info != other.platform_info:
            return False
        if self.rapps != other.rapps:
            return False                # different rapps advertised
        if self.owner != other.owner:
            return False
        if self.status != other.status:
            return False
        return True

    def __hash__(self):
        return hash(self.platform_info)

    def __ne__(self, other):
        return not self == other

    def __str__(self):
        rappstr = ''
        for rapp_name in self.rapps:
            rappstr += '\n    ' + str(rapp_name)
        return (self.platform_info + ', status: ' + str(self.status)
                + '\n  owner: ' + str(self.owner)
                + '\n  rapps:' + rappstr)

    def allocate(self, request_id):
        """ Allocate this resource.

        :param request_id: New owner of this resource.
        :type request_id: :class:`uuid.UUID`

        :raises: :exc:`.ResourceNotAvailableError` if not available
        """
        if (self.status != CurrentStatus.AVAILABLE):
            raise ResourceNotAvailableError('resource not available: '
                                            + self.platform_info)
        assert self.owner is None
        self.owner = request_id
        self.status = CurrentStatus.ALLOCATED

    def match(self, res):
        """ Match this resource to a requested one.

        :param res: Resource request to match.
        :type res: ``scheduler_msgs/Resource`` or :class:`.PoolResource`
        :returns: ``True`` if this specific resource matches.

        To match, the *res.name* must be one of the rapps advertised
        by this ROCON resource.  The *res.platform_info* may include
        Python regular expression syntax for matching multiple
        resource names.

        If the *res.platform_info* is not a canonical ROCON name
        starting with 'rocon://', it will be converted from shell
        wildcard syntax into an equivalent Python regular expression.

        """
        return self.match_pattern(rocon_name(res.platform_info), res.name)

    def match_pattern(self, pattern, rapp):
        """ Match this resource to a ROCON name and application.

        :param pattern: Canonical ROCON name to match, maybe a regular
            expression.
        :type pattern: str
        :param rapp: ROCON application name.
        :type rapp: str
        :returns: ``True`` if this specific resource matches.

        The *rapp* must be one of those advertised by this ROCON
        resource.  The *pattern* may include Python regular expression
        syntax for matching multiple resource names.

        """
        if rapp not in self.rapps:      # rapp not advertised here?
            return False
        return re.match(pattern, self.platform_info)

    def release(self, request_id=None):
        """ Release this resource.

        :param request_id: Optional owning request.
        :type request_id: :class:`uuid.UUID` or ``None``

        :raises: :exc:`.ResourceNotOwnedError` if *request_id* is
            specified and is not the owner.
        """
        if (request_id is not None and self.owner != request_id):
            raise ResourceNotOwnedError('resource not owned by '
                                        + str(request_id) + ': '
                                        + self.platform_info)
        self.owner = None
        if self.status == CurrentStatus.ALLOCATED:  # not gone missing?
            self.status = CurrentStatus.AVAILABLE


class ResourceSet:
    """
    This class is a container for :class:`.PoolResource` objects
    known to the scheduler.  It acts like a dictionary.

    *Deprecated*: use :class:`.ResourcePool`, instead.  Useful
    attributes of this class will be merged, as needed.

    :param msg: An optional ``scheduler_msgs/KnownResources`` or
        ``scheduler_msgs/Request`` message or a list of
        ``CurrentStatus`` or ``Resource`` messages, like the
        ``resources`` component of one of those messages.

    :class:`.ResourceSet` supports these standard container operations:

    .. describe:: key in resources

       :returns: ``True`` if *resources* contains *key*, else ``False``.

    .. describe:: key not in resources

       Equivalent to ``not key in resources``.

    .. describe:: len(resources)

       :returns: The number of resources in the set.

    .. describe:: resources[key]

       :param key: (str) A ROCON resource name.
       :returns: The :class:`PoolResource` corresponding to *key*.
       :raises: :exc:`KeyError` if no such *key*.

    .. describe:: resources[key] = res

       Assign a :class:`.PoolResource` to this *key*.

       :param key: (str) A ROCON resource name.
       :param res: Resource to add.
       :type res: :class:`.PoolResource` or ``scheduler_msgs/Resource``

    .. describe:: resources == another

       :returns: ``True`` if this :class:`.ResourceSet` is equal to *another*.

    .. describe:: resources != another

       :returns: ``True`` if this :class:`.ResourceSet` and *another* have
           different contents.

    .. describe:: str(resources)

       :returns: Human-readable string representation of this
           :class:`.ResourceSet`.

    These attributes are also provided:

    """
    def __init__(self, msg=None):
        """ Constructor. """
        self.resources = {}
        """ Dictionary of known :class:`.PoolResource` objects. """
        if msg is not None:
            if hasattr(msg, 'resources'):
                msg = msg.resources
            for res in msg:
                rocon_res = PoolResource(res)
                self.resources[hash(rocon_res)] = rocon_res

    def __contains__(self, res):
        return hash(res) in self.resources

    def __eq__(self, other):
        if set(self.resources.keys()) != set(other.resources.keys()):
            return False        # different resources hash IDs
        for res_id, res in self.resources.items():
            if res != other[res_id]:
                return False
        return True

    def __getitem__(self, key):
        """
        :param key: Key of desired resource.

        :returns: named item.
        :raises: :exc:`KeyError` if no such request
        """
        return self.resources[hash(key)]

    def __len__(self):
        """ Number of resources. """
        return len(self.resources)

    def __ne__(self, other):
        return not self == other

    def __setitem__(self, key, res):
        """ Assign a :class:`.PoolResource` to this *key*. """
        if not isinstance(res, PoolResource):
            res = PoolResource(res)     # make a PoolResource instance
        self.resources[hash(key)] = res

    def __str__(self):
        """ Format resource set into a human-readable string. """
        res_str = ''
        for res in self.resources.values():
            res_str += '\n  ' + str(res)
        return ('ROCON resource set:' + res_str)

    def get(self, key, default=None):
        """ Get resource, if known.

        :param key: ROCON name of desired resource.
        :type key: str
        :param default: value to return if no such resource.

        :returns: named :class:`.PoolResource` if successful, else *default*.

        """
        return self.resources.get(hash(key), default)
