#!/usr/bin/env python
""" Mock conductor for providing simple scheduler test resources. """

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import rospy
import rospkg
import yaml
from rocon_app_manager_msgs.msg import App
from rocon_std_msgs.msg import PlatformInfo
from concert_msgs.msg import ConcertClient, ConcertClients


class MockConductor():

    def __init__(self):
        """ Constructor. """
        rospy.init_node('mock_conductor')
        self.pub = rospy.Publisher('concert_client_changes',
                                   ConcertClients, queue_size=1, latch=True)
        rospy.sleep(1.0)                # let publisher initialize
        self.send_resources()
        rospy.spin()                    # wait for shutdown
        
    def send_resources(self):
        """ Send resources on latched concert client topic. """
        url = rospy.get_param(
            '~resources_url',
            'package://concert_simple_scheduler/tests/params/clients1.yaml')
        # TODO: parse package name, get file name and path
        package = 'concert_simple_scheduler'
        path = 'tests/params/clients1.yaml'
        r = rospkg.RosPack()
        yaml_name = r.get_path(package) + '/' + path
        with open(yaml_name, 'rt') as f:
            resources = yaml.load(f)
        msg = ConcertClients()
        for res in resources:
             ccl = ConcertClient(
                 name=res['name'],
                 platform_info = PlatformInfo(uri=res['uri']))
             for rapp in res['rapps']:
                 ccl.apps.append(App(name=rapp))
             msg.clients.append(ccl)
        self.pub.publish(msg)


if __name__ == '__main__':
    node = MockConductor()
