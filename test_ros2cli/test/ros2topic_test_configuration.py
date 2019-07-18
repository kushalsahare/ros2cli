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

import itertools

import os
import re
import sys
import time

from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.append(os.path.dirname(__file__))

from cli_test_configuration import CLITestConfiguration  # noqa


def get_dummy_base_controller_node_action(*, topic_name='/my_ns/cmd_vel'):
    return ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '-r', '1', topic_name,
            'geometry_msgs/msg/TwistStamped',
            '{header: {stamp: {sec: ' + str(int(time.time())) + '}}}'
        ], name='dummy_base_controller', output='screen',
        sigterm_timeout=LaunchConfiguration(
            'sigterm_timeout', default=60
        )
    )


def get_talker_node_action(*, node_name='my_talker',
                           node_namespace='my_ns',
                           topic_name='chatter'):
    return Node(
        package='demo_nodes_py', node_executable='talker', node_name=node_name, name=node_name,
        node_namespace=node_namespace, remappings=[('chatter', topic_name)], output='screen',
        sigterm_timeout=LaunchConfiguration('sigterm_timeout', default=30)
    )


def get_listener_node_action(*, node_name='my_listener',
                             node_namespace='my_ns',
                             topic_name='chatter'):
    return Node(
        package='demo_nodes_py', node_executable='listener', node_name=node_name, name=node_name,
        node_namespace=node_namespace, remappings=[('chatter', topic_name)], output='screen',
        sigterm_timeout=LaunchConfiguration('sigterm_timeout', default=30)
    )


test_configurations = [
    CLITestConfiguration(
        command='topic',
        arguments=['list'],
        fixture_actions=[
            get_talker_node_action(),
            get_talker_node_action(topic_name='_chatter')
        ],
        expected_output=[
            '/my_ns/chatter',
            '/my_ns/parameter_events',
            '/my_ns/rosout',
            '/parameter_events',
            '/rosout'
        ]
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['list', '--include-hidden-topics'],
        fixture_actions=[
            get_talker_node_action(),
            get_talker_node_action(topic_name='_chatter')
        ],
        expected_output=[
            '/my_ns/_chatter',
            '/my_ns/chatter',
            '/my_ns/parameter_events',
            '/my_ns/rosout',
            '/parameter_events',
            '/rosout'
        ]
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['list', '-t'],
        fixture_actions=[get_talker_node_action()],
        expected_output=[
            '/my_ns/chatter [std_msgs/msg/String]',
            '/my_ns/parameter_events [rcl_interfaces/msg/ParameterEvent]',
            '/my_ns/rosout [rcl_interfaces/msg/Log]',
            '/parameter_events [rcl_interfaces/msg/ParameterEvent]',
            '/rosout [rcl_interfaces/msg/Log]'
        ],
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['list', '-c'],
        fixture_actions=[get_talker_node_action()],
        expected_output=[lambda line: int(line) == 5],
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['info', '/my_ns/chatter'],
        fixture_actions=[get_talker_node_action()],
        expected_output=[
            'Topic: /my_ns/chatter',
            'Publisher count: 1',
            'Subscriber count: 0'
        ],
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['info', '/some_chatter'],
        fixture_actions=[get_talker_node_action()],
        expected_output=[
            'Topic: /some_chatter',
            'Publisher count: 0',
            'Subscriber count: 0'
        ],
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['type', '/my_ns/chatter'],
        fixture_actions=[get_talker_node_action()],
        expected_output=['std_msgs/msg/String'],
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['type', '/my_ns/_chatter'],
        fixture_actions=[get_talker_node_action(topic_name='_chatter')],
        expected_output=None,
        exit_codes=[1]
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['find', 'rcl_interfaces/msg/Log'],
        fixture_actions=[get_talker_node_action()],
        expected_output=[
            '/my_ns/rosout',
            '/rosout'
        ],
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['find', 'rcl_interfaces/msg/NotAMessageType'],
        fixture_actions=[get_talker_node_action()],
        expected_output=None,
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['find', 'rcl_interfaces/msg/NotAMessageType'],
        fixture_actions=[get_talker_node_action()],
        expected_output=None,
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['echo', '/my_ns/chatter'],
        fixture_actions=[get_talker_node_action()],
        expected_output=itertools.cycle([
            re.compile(r"data: 'Hello World: \d+'"),
            '---'
        ]),
        self_terminates=False,
        exit_codes=[2],
        timeout=2.0
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['echo', '--no-str', '/my_ns/chatter'],
        fixture_actions=[get_talker_node_action()],
        expected_output=itertools.cycle([
            re.compile(r"data: '<string length: <\d+>>'"),
            '---'
        ]),
        self_terminates=False,
        exit_codes=[2],
        timeout=2.0
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['echo', '--truncate-length 5', '/my_ns/chatter'],
        fixture_actions=[get_talker_node_action()],
        expected_output=itertools.cycle([
            re.compile(r'data: Hello...'),
            '---'
        ]),
        self_terminates=False,
        exit_codes=[1],
        timeout=2.0,
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['pub', '/my_ns/chatter', 'std_msgs/msg/String', '{data: test}'],
        fixture_actions=[get_listener_node_action(node_name='my_listener')],
        expected_output={
            'cli': itertools.chain(
                ['publisher: beginning loop'],
                itertools.cycle([
                    re.compile(r"publishing #\d+: std_msgs\.msg\.String\(data='test'\)"),
                    ''
                ])
            ),
            'my_listener': itertools.repeat('[INFO] [my_ns.my_listener]: I heard: [test]')
        },
        self_terminates=False,
        exit_codes=[2],
        timeout=5.0
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['pub', '--once', '/my_ns/chatter', 'std_msgs/msg/String', '{data: test}'],
        expected_output=[
            'publisher: beginning loop',
            re.compile(r"publishing #1: std_msgs\.msg\.String\(data='test'\)"),
            ''
        ],
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['pub', '-r', '1', '/chatter', 'std_msgs/msg/String', '{data: test}'],
        expected_output=[
            'publisher: beginning loop',
            *([re.compile(r"publishing #\d+: std_msgs\.msg\.String\(data='test'\)"), '']*3),
        ],
        self_terminates=False,
        exit_codes=[2],
        timeout=2.0
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['pub', '-r', '2', '/my_ns/chatter', 'std_msgs/msg/String', '{data: test}'],
        expected_output=[
            'publisher: beginning loop',
            *([re.compile(r"publishing #\d+: std_msgs\.msg\.String\(data='test'\)"), ''] * 5),
        ],
        self_terminates=False,
        exit_codes=[2],
        timeout=2.0
    ),
    CLITestConfiguration(
        command='topic',
        arguments=[
            'pub',
            '-p', '2',
            '/my_ns/chatter',
            'std_msgs/msg/String',
            '{data: test}'
        ],
        expected_output=[
            'publisher: beginning loop',
            "publishing #2: std_msgs.msg.String(data='test')",
            ''
        ],
        self_terminates=False,
        exit_codes=[2],
        timeout=2.0
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['delay', '/my_ns/cmd_vel'],
        fixture_actions=[get_dummy_base_controller_node_action()],
        expected_output=itertools.cycle([
            re.compile(r'average delay: \d{,2}.\d{3}'),
            re.compile(r'\s*min: \d{,2}.\d{3}s max: \d{,2}.\d{3}s std dev: \d+.\d{5}s window: \d+')
        ]),
        self_terminates=False,
        exit_codes=[2],
        timeout=5.0
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['hz', '/my_ns/chatter'],
        fixture_actions=[get_talker_node_action()],
        expected_output=itertools.cycle([
            lambda line: (
                re.match(r'average rate: \d\.\d{3}', line) is not None
                and abs(float(line[-5:]) - 1.) < 0.01
            ),
            re.compile(r'\s*min: \d\.\d{3}s max: \d\.\d{3}s std dev: \d\.\d{5}s window: \d+')
        ]),
        self_terminates=False,
        exit_codes=[2],
        timeout=5.0
    ),
    CLITestConfiguration(
        command='topic',
        arguments=[
            'hz',
            '--filter',
            'int(m.data.rpartition(\":\")[-1]) % 2 == 0',
            '/my_ns/chatter'
        ],
        fixture_actions=[get_talker_node_action()],
        expected_output=itertools.cycle([
            lambda line: (
                re.match(r'average rate: \d\.\d{3}', line) is not None
                and abs(float(line[-5:]) - 0.5) < 0.01
            ),
            re.compile(r'\s*min: \d\.\d{3}s max: \d\.\d{3}s std dev: \d\.\d{5}s window: \d+')
        ]),
        self_terminates=False,
        exit_codes=[2],
        timeout=10.0
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['bw', '/my_ns/chatter'],
        fixture_actions=[get_talker_node_action()],
        expected_output=itertools.chain(['Subscribed to [/my_ns/chatter]'], itertools.cycle([
            re.compile(r'average: 2\d\.\d{2}B/s'),
            re.compile(r'\s*mean: 2\d\.\d{2}B/s min: 2\d\.\d{2}B/s max: 2\d\.\d{2}B/s window: \d+')
        ])),
        self_terminates=False,
        exit_codes=[2],
        timeout=5.0
    ),
]
