#!/usr/bin/env python

# enable some python3 compatibility options:
# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

import uuid
import unittest

# module being tested:
import concert_scheduler_requests.common as common

TEST_UUID_HEX = '0123456789abcdef0123456789abcdef'
TEST_UUID = uuid.UUID(hex=TEST_UUID_HEX)

class TestCommonModule(unittest.TestCase):
    """Unit tests for scheduler request manager common module.

    These tests do not require a running ROS core.
    """

    def test_feedback_default_topic(self):
        self.assertEqual(common.feedback_topic(TEST_UUID),
                         common.SCHEDULER_TOPIC + '_' + TEST_UUID_HEX)

    def test_feedback_topic(self):
        topic = common.feedback_topic(TEST_UUID, scheduler_topic='xxx')
        self.assertEqual(topic, 'xxx_' + TEST_UUID_HEX)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('concert_scheduler_requests_common',
                    'test_common_module',
                    TestCommonModule)
