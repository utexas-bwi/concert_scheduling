#!/usr/bin/env python
""" Mock conductor for providing simple scheduler test resources. """

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import rospy
import rospkg
import yaml
from rocon_app_manager_msgs.msg import Rapp
from rocon_app_manager_msgs.srv import (
    StartRapp, StartRappResponse, StopRapp, StopRappResponse)
from rocon_std_msgs.msg import PlatformInfo
from concert_msgs.msg import ConcertClient, ConcertClients
try:
    from urllib.parse import urlparse   # python 3
except ImportError:
    from urlparse import urlparse       # python 2

def url_filename(url):
    """ :returns: file name corresponding to the *url*. """
    parse = urlparse(url)
    if parse.scheme == 'package':
        r = rospkg.RosPack()
        return r.get_path(parse.netloc) + parse.path
    elif parse.scheme == 'file':
        return parse.path
    else:
        raise ValueError('invalid resources URL: ' + url)


class MockGateway(object):
    """ Representation of a Concert Gateway. """
    def __init__(self, concert_client):
        self.started = False
        self.client = concert_client
        self.rapps = set()
        """ Set of rapps supported at this gateway. """
        for rapp in self.client.rapps:
            self.rapps.add(rapp.name)
        namespace = '/' + self.client.gateway_name
        self.start_service = rospy.Service(namespace + '/start_rapp',
                                           StartRapp, self.start)
        self.stop_service = rospy.Service(namespace + '/stop_rapp',
                                          StopRapp, self.stop)

    def start(self, req):
        """ Mock StartRapp service handler. """
        resp = StartRappResponse(
            started=True, application_namespace=self.client.gateway_name)
        if self.started == True:
            resp.message = 'Rapp already started: ' + self.client.gateway_name
            rospy.logerr(resp.message)
            resp.started = False
        elif req.name not in self.rapps:
            resp.message = 'Unknown rapp : ' + req.name
            rospy.logerr(resp.message)
            resp.started = False
        else:
            self.started = True
        return resp

    def stop(self, req):
        """ Mock StopRapp service handler. """
        self.started = False
        return StopRappResponse(stopped=True)


class MockConductor(object):

    def __init__(self):
        """ Constructor. """
        rospy.init_node('mock_conductor')
        self.pub = rospy.Publisher('concert_client_changes',
                                   ConcertClients, queue_size=1, latch=True)
        self.gateways = {}              # dictionary of gateways
        delay = rospy.get_param('~delay', 1.0)
        rospy.sleep(delay)              # wait for test to commence
        try:
            self.send_resources()
        except (ValueError, IOError, rospkg.common.ResourceNotFound) as e:
            rospy.logfatal(str(e))
        else:
            rospy.spin()                # wait for shutdown

    def send_resources(self):
        """ Send resources on latched concert client topic. """
        url = rospy.get_param(
            '~resources_url',
            'package://concert_simple_scheduler/tests/params/clients2.yaml')
        rospy.loginfo('resources URL: ' + url)
        yaml_name = url_filename(url)
        rospy.logdebug(yaml_name)
        resources = []
        with open(yaml_name, 'rt') as f:
            resources = yaml.load(f)
            if resources is None:
                resources = []
        msg = ConcertClients()
        for res in resources:
             ccl = ConcertClient(
                 name=res['name'],
                 platform_info = PlatformInfo(uri=res['uri']))
             for rapp in res['rapps']:
                 ccl.rapps.append(Rapp(name=rapp))

             gw = res.get('gateway_name')
             if gw is None:
                 # ignore missing gateway name
                 rospy.logwarn(ccl.name + ': gateway name missing, ' +
                               'no start or stop services')
             elif gw not in self.gateways:
                 ccl.gateway_name = gw
                 self.gateways[gw] = MockGateway(ccl)

             msg.clients.append(ccl)

        self.pub.publish(msg)


if __name__ == '__main__':
    node = MockConductor()
