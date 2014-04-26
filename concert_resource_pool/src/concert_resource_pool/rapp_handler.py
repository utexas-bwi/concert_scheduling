#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Module
##############################################################################

"""
.. module:: rapp_handler

This module provides a class to other nodes (not the conductor itself)
that can be used to start/stop rapps on each concert client. This class
is initialised by data coming from the conductor's concert client
publishers.

"""

##############################################################################
# Imports
##############################################################################

import rospy
from rocon_app_manager_msgs.srv import (
    StartRapp, StartRappRequest, StopRapp, StopRappRequest)


##############################################################################
# Exceptions
##############################################################################


class FailedToStartRappError(Exception):
    """ Failed to start rapp. """
    pass


class FailedToStopRappError(Exception):
    """ Failed to stop rapp. """
    pass

##############################################################################
# Classes
##############################################################################


class RappHandler(object):
    """
    Initialises from a conductor message detailing information about a
    concert client. Once established, this instance can be used as
    a convenience to start and stop rapps on the concert client.
    """
    __slots__ = [
        'name',           # conductor's alias for this concert client
        'gateway_name',   # unique gateway name
        'uri',            # rocon uri for this concert client
        'rapps',          # list of rapp names that can be started
        'start_rapp',     # service proxy to this client's start_rapp service
        'stop_rapp'       # service proxy to this client's stop_rapp service
    ]

    def __init__(self, msg):
        """
        Initialise the class with the relevant data required to start and stop
        rapps on this concert client.

        :param msg concert_msgs/ConcertClient: detailed information
            about a concert client.
        """
        self.name = msg.name
        self.gateway_name = msg.gateway_name
        self.uri = msg.platform_info.uri
        self.rapps = [rapp.name for rapp in msg.rapps]
        self.start_rapp = rospy.ServiceProxy(
            '/' + self.gateway_name + '/start_rapp', StartRapp)
        self.stop_rapp = rospy.ServiceProxy(
            '/' + self.gateway_name + '/stop_rapp', StopRapp)

    def start(self, rapp, remappings):
        """
        Start the rapp with the specified remappings.

        :param rapp str: name of the rapp to start (e.g. rocon_apps/teleop)
        :param remappings rocon_std_msgs/Remapping[]: remappings to apply to
            the rapp when starting.

        :raises: :exc:`.FailedToStartRappError`
        """
        try:
            resp = self.start_rapp(StartRappRequest(name=rapp,
                                                    remappings=remappings))
        except (rospy.service.ServiceException,
                rospy.exceptions.ROSInterruptException) as e:
            # Service not found or ros is shutting down
            raise FailedToStartRappError(str(e))
        else:
            if not resp.started:
                # start_rapp request failed
                raise FailedToStartRappError(resp.message)

    def stop(self):
        """
        Stop a rapp on this concert client (if one should be running). This
        doesn't need a rapp specification since only one rapp can ever be
        running - it will just stop the currently running rapp.
        """
        try:
            resp = self.stop_rapp(StopRappRequest())
        except (rospy.service.ServiceException,
                rospy.exceptions.ROSInterruptException) as e:
            # Service not found or ros is shutting down
            raise FailedToStopRappError(str(e))
        else:
            if not resp.stopped:
                raise FailedToStopRappError(resp.message)
