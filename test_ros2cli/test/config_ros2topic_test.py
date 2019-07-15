# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

sys.path.append(os.path.dirname(__file__))

from test_config import TestConfig  # noqa


def get_talker_node(topic_name=None):
    remappings = None
    if topic_name is not None:
        remappings = [('chatter', topic_name)]
    return launch_ros.actions.Node(
        package='demo_nodes_py', node_executable='talker', node_name='my_talker',
        node_namespace='my_ns', output='screen', remappings=remappings,
        sigterm_timeout=LaunchConfiguration('sigterm_timeout', default=30)
    )

test_configs = [
    TestConfig(
        command='topic',
        arguments=['list'],
        actions=[get_talker_node(), get_talker_node(topic_name='_chatter')],
        expected_output=['/chatter', '/parameter_events', '/rosout'],
        bad_output=['/_chatter'],
    ),
    TestConfig(
        command='topic',
        arguments=['list --include-hidden-topics'],
        actions=[get_talker_node(topic_name='_chatter')],
        expected_output=['/_chatter', '/chatter', '/parameter_events', '/rosout'],
    ),
    TestConfig(
        command='topic',
        arguments=['list -t'],
        actions=[get_talker_node()],
        expected_output=[
            '/chatter [std_msgs/msg/String]',
            '/parameter_events [rcl_interfaces/msg/ParameterEvent]',
            '/rosout [rcl_interfaces/msg/Log]'
        ],
    ),
    TestConfig(
        command='topic',
        arguments=['list -c'],
        actions=[get_talker_node()],
        expected_output=['3'],
    ),
    TestConfig(
        command='topic',
        arguments=['info /chatter'],
        actions=[get_talker_node()],
        expected_output=['Publisher count: 1', 'Subscriber count: 0'],
    ),
    TestConfig(
        command='topic',
        arguments=['info /some_chatter'],
        actions=[get_talker_node()],
        expected_output=['Publisher count: 0', 'Subscriber count: 0'],
    ),
    TestConfig(
        command='topic',
        arguments=['type /_chatter'],
        actions=[get_talker_node(topic_name='_chatter')],
        expected_output=None,
    ),
    TestConfig(
        command='topic',
        arguments=['find rcl_interfaces/msg/Log'],
        actions=[get_talker_node()],
        expected_output=['/rosout'],
    ),
    TestConfig(
        command='topic',
        arguments=['find rcl_interfaces/msg/NotAMessage'],
        actions=[get_talker_node()],
        expected_output=None,
    ),
    TestConfig(
        command='topic',
        arguments=['find rcl_interfaces/msg/NotAMessage'],
        actions=[get_talker_node()],
        expected_output=None,
    ),
]
