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
.. module:: resource_pool

This module tracks all known resources managed by the scheduler.  The
`scheduler_msgs/Resource`_ ROS message describes resources used by the
`Robotics in Concert`_ (ROCON) project.

.. include:: weblinks.rst

"""
from __future__ import absolute_import, print_function, unicode_literals

import copy
from itertools import chain, islice, permutations
import rocon_uri
import unique_id

## ROS messages
from scheduler_msgs.msg import CurrentStatus, KnownResources, Resource
from concert_msgs.msg import ConcertClientState


## Exceptions
class InvalidRequestError(Exception):
    """ Request cannot be satisfied as specified. """
    pass


class ResourceNotAvailableError(Exception):
    """ Error exception: resource not available. """
    pass


class ResourceNotOwnedError(Exception):
    """ Error exception: resource not owned. """
    pass


class PoolResource(object):
    """
    Base class for tracking the status of a single ROCON_ resource.

    :param msg: ROCON scheduler resource description message.
    :type msg: ``ConcertClient``, ``CurrentStatus`` or ``Resource``

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
        try:
            self.uri = msg.platform_info.uri
            """ Fully-resolved canonical ROCON resource name. """
        except AttributeError:          # not a ConcertClient message?
            self.uri = msg.uri
        try:
            self.rapps = set()
            for rapp in msg.rapps:
                self.rapps.add(rapp.name)
            """ The :class:`set` of ROCON application name strings
            this platform advertises. """
        except AttributeError:          # not a ConcertClient message?
            if hasattr(msg, 'rapps'):   # CurrentStatus message?
                self.rapps = set(msg.rapps)
            else:                       # Resource message
                self.rapps = set([msg.rapp])
        self.status = CurrentStatus.AVAILABLE
        """ Current status of this resource. """
        self.owner = None
        """ :class:`uuid.UUID` of request to which this resource is
        currently assigned, or ``None``.
        """
        self.priority = 0
        """ Priority of request to which this resource is currently
        assigned. """

    def __eq__(self, other):
        if self.uri != other.uri:
            return False
        if self.rapps != other.rapps:
            return False                # different rapps advertised
        if self.owner != other.owner:
            return False
        if self.status != other.status:
            return False
        return True

    def __hash__(self):
        return hash(self.uri)

    def __ne__(self, other):
        return not self == other

    def __str__(self):
        rappstr = ''
        for rapp_name in self.rapps:
            rappstr += '\n    ' + str(rapp_name)
        return (self.uri + ', status: ' + str(self.status)
                + '\n  owner: ' + str(self.owner)
                + '\n  priority: ' + str(self.priority)
                + '\n  rapps:' + rappstr)

    def allocate(self, request):
        """ Allocate this resource.

        :param request: New owner of this resource.
        :type request: :class:`.ActiveRequest`

        :raises: :exc:`.ResourceNotAvailableError` if not available
        """
        if (self.status != CurrentStatus.AVAILABLE):
            raise ResourceNotAvailableError('resource not available: '
                                            + self.uri)
        assert self.owner is None
        self.owner = request.uuid
        self.priority = request.msg.priority
        self.status = CurrentStatus.ALLOCATED

    def current_status(self):
        """ :returns: ``scheduler_msgs/CurrentStatus`` for this resource. """
        msg = CurrentStatus(uri=self.uri, status=self.status,
                            rapps=list(self.rapps))
        if self.status == CurrentStatus.ALLOCATED:
            msg.owner = unique_id.toMsg(self.owner)
            msg.priority = self.priority
        return msg

    def match(self, res):
        """ Match this resource to a requested one.

        :param res: Resource request to match.
        :type res: ``scheduler_msgs/Resource`` or :class:`.PoolResource`
        :returns: ``True`` if this specific resource matches.

        To match, the *res.rapp* must be one of the rapps advertised
        by this ROCON resource.  The *res.uri* may include Python
        regular expression syntax for matching multiple resource
        names.

        If the *res.uri* is not a canonical ROCON name starting with
        'rocon:', it will be converted from shell wildcard syntax into
        an equivalent Python regular expression.

        """
        return self.match_pattern(res.uri, res.rapp)

    def match_pattern(self, pattern, rapp):
        """ Match this resource to a ROCON name and application.

        :param pattern: Canonical ROCON name to match, maybe a regular
            expression.
        :type pattern: str
        :param rapp: ROCON application name.
        :type rapp: str
        :returns: ``True`` if this specific resource matches.
        :raises: :class:`.RoconURIValueError` if *pattern* invalid.

        The *rapp* must be one of those advertised by this ROCON
        resource.  The *pattern* may be a ROCON pattern matching
        multiple resource names.
        """
        if rapp not in self.rapps:      # rapp not advertised here?
            return False
        return rocon_uri.is_compatible(self.uri, pattern)

    def release(self, request_id=None):
        """ Release this resource.

        :param request_id: Optional owning request.
        :type request_id: :class:`uuid.UUID` or ``None``

        :raises: :exc:`.ResourceNotOwnedError` if *request_id* is
            specified and is not the owner.
        """
        if (request_id is not None and self.owner != request_id):
            raise ResourceNotOwnedError('resource not owned by '
                                        + str(request_id) + ': ' + self.uri)
        self.owner = None
        self.priority = 0               # no longer applicable
        if self.status == CurrentStatus.ALLOCATED:  # not MISSING or GONE?
            self.status = CurrentStatus.AVAILABLE


class ResourcePool(object):
    """
    This class manages a pool of :class:`.PoolResource` objects known
    to the scheduler.

    :param msg: An optional ``scheduler_msgs/KnownResources`` or
        ``scheduler_msgs/Request`` message or a list of
        ``CurrentStatus`` or ``Resource`` messages, like the
        ``resources`` component of one of those messages.
    :param pool_resource: a possibly-derived class providing a
        compatible :class:`.ResourcePool` interface.

    :class:`.ResourcePool` supports these standard container operations:

    .. describe:: len(pool)

       :returns: The number of resources in the pool.

    .. describe:: pool[uri]

       :param uri: (str) The Uniform Resource Identifier of a resource
           in the *pool*.
       :returns: The actual pool resource corresponding to *uri*.
       :raises: :exc:`KeyError` if *uri* not in the *pool*.

    .. describe:: uri in pool

       :returns: ``True`` if *pool* contains *uri*, else ``False``.

    .. describe:: uri not in pool

       Equivalent to ``not uri in pool``.

    .. describe:: str(pool)

       :returns: Human-readable string representation of this
           :class:`.ResourcePool`.

    """
    def __init__(self, msg=None, pool_resource=PoolResource):
        self.pool = {}
        """ Dictionary of known *pool_resource* objects, indexed by
        the fully-resolved ROCON resource name.
        """
        self.changed = True
        """ True, if resource pool has changed since the previous
        known_resources() call. """
        self.pool_resource = pool_resource
        """ The :class:`.ResourcePool` class this pool contains. """
        if msg is not None:
            if hasattr(msg, 'resources'):
                msg = msg.resources
            for res in msg:
                pool_res = self.pool_resource(res)
                self.pool[pool_res.uri] = pool_res

    def __contains__(self, uri):
        return uri in self.pool

    def __getitem__(self, uri):
        return self.pool[uri]

    def __len__(self):
        return len(self.pool)

    def __str__(self):
        s = 'pool contents:'
        for resource in self.pool.values():
            s += '\n  ' + str(resource)
        return s

    def allocate(self, request):
        """ Try to allocate all resources for a *request*.

        :param request: Scheduler request object, some resources may
            include regular expression syntax.
        :type request: :class:`.ActiveRequest`

        :returns: List of ``scheduler_msgs/Resource`` messages
            allocated, in requested order with platform info fully
            resolved; or ``[]`` if not everything is available.

        :raises: :exc:`.InvalidRequestError` if the request is not valid.

        If successful, matching ROCON resources are allocated to this
        *request*.  Otherwise, the *request* remains unchanged.

        """
        n_wanted = len(request.msg.resources)  # number of resources wanted
        if n_wanted == 0:
            raise InvalidRequestError('No resources requested.')

        # Make a list containing sets of the available resources
        # matching each requested item.
        matches = self.match_list(request.msg.resources,
                                  {CurrentStatus.AVAILABLE})
        if not matches:                 # unsuccessful?
            return []                   # give up

        # At least one resource is available that satisfies each item
        # requested.  Try to allocate them all in the order requested.
        alloc = self._allocate_permutation(range(n_wanted), request, matches)
        if alloc:                       # successful?
            return alloc

        if n_wanted < 4:                # not too many permutations?
            # Look for some other permutation that satisfies them all.
            for perm in islice(permutations(range(n_wanted)), 1, None):
                alloc = self._allocate_permutation(perm, request, matches)
                if alloc:               # successful?
                    return alloc

        raise InvalidRequestError(
            'Resources are available, but this request cannot be satisfied.')

    def _allocate_permutation(self, perm, request, matches):
        """ Try to allocate some permutation of resources for a *request*.

        :param perm: List of permuted resource indices for this
            *request*, like [0, 1, 2] or [1, 2, 0].
        :param request: Scheduler request object, some resources may
            include regular expression syntax.
        :type request: :class:`.ActiveRequest`
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
                    alloc[i].uri = name
                    break               # go on to next resource
            else:
                return []               # failure: no matches work

        # successful: allocate to this request
        for resource in alloc:
            self.pool[resource.uri].allocate(request)
            self.changed = True
        return alloc                    # success

    def get(self, resource_name, default=None):
        """ Get named pool resource, if known.

        :param resource_name: Name of desired resource.
        :type resource_name: str
        :param default: value to return if no such resource.
        :returns: named pool resource if successful, else *default*.
        """
        return self.pool.get(resource_name, default)

    def known_resources(self):
        """ Convert resource pool to ``scheduler_msgs/KnownResources``. """
        msg = KnownResources()
        for resource in self.pool.values():
            msg.resources.append(resource.current_status())
        self.changed = False
        return msg

    def match_list(self, resources, criteria):
        """
        Make a list containing sets of the available resources
        matching each item in *resources*.

        :param resources: List of Resource messages to match.
        :param criteria: :class:`set` of resource status values allowed.

        :returns: List of :class:`set` containing names of matching
            resources, empty if any item cannot be satisfied, or there
            are not enough resources, or the original *resources* list
            was empty.
        """
        matches = []
        for res_req in resources:
            match_set = self._match_subset(res_req, criteria)
            if len(match_set) == 0:     # no matches for this resource?
                return []               # give up
            matches.append(match_set)
        if not matches:
            return []                   # give up

        # Each individual request can be satisfied, but there might
        # not be enough to satisfy them all at once.  Verify that the
        # union of the match sets contains as many resources as requested.
        match_union = set(chain.from_iterable(matches))
        if len(match_union) < len(resources):
            return []                   # not enough stuff
        return matches

    def _match_subset(self, resource_msg, criteria):
        """
        Make a set of names of all available resources matching *resource_msg*.

        :param resource_msg: Resource message from a scheduler Request.
        :type resource_msg: ``scheduler_msgs/Resource``
        :param criteria: :class:`set` of resource status values allowed.
        :returns: :class:`set` containing matching resource names.
        """
        avail = set()
        for res in self.pool.values():
            if (res.status in criteria and res.match(resource_msg)):
                avail.add(res.uri)
        return avail

    def release_request(self, request):
        """ Release all the resources owned by this *request*.

        :param request: Current owner of resources to release.
        :type request: :class:`.ActiveRequest`

        Only appropriate when this *request* is being closed.
        """
        self.release_resources(request.allocations, request.uuid)

    def release_resources(self, resources, request_id=None):
        """ Release a list of *resources*.

        :param resources: List of ``scheduler_msgs/Resource`` messages.
        :param request_id: Optional owning request.
        :type request_id: :class:`uuid.UUID` or ``None``

        This makes newly allocated *resources* available again when
        they cannot be assigned to a request for some reason.
        """
        for res in resources:
            pool_res = self.pool[res.uri]
            pool_res.release(request_id)
            if pool_res.status == CurrentStatus.GONE:
                del self.pool[res.uri]  # forget about it
            self.changed = True

    def update(self, client_list):
        """ Update resource pool from a new concert clients list.

        :param client_list: current list of ``ConcertClient`` messages.
        """
        clients_found = set()
        for client in client_list:
            uri = client.platform_info.uri
            clients_found.add(uri)
            if uri not in self.pool:    # not previously-known?
                self.pool[uri] = self.pool_resource(client)
                self.changed = True
            pool_res = self.pool[uri]
            if client.state != ConcertClientState.AVAILABLE:
                pool_res.status = CurrentStatus.MISSING
                self.changed = True
            elif pool_res.status == CurrentStatus.MISSING:
                if pool_res.owned:
                    pool_res.status = CurrentStatus.ALLOCATED
                else:
                    pool_res.status = CurrentStatus.AVAILABLE
                self.changed = True

        # Previously-known resources not in clients_found are GONE,
        # having voluntarily left the Concert.
        missing_clients = set(self.pool.keys()) - clients_found
        for uri in missing_clients:
            res = self.pool[uri]
            res.status = CurrentStatus.GONE
            if res.owner is None:       # not currently owned?
                del self.pool[uri]      # forget about it
            self.changed = True
