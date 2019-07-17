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

from cli_test_configuration import CLITestConfiguration  # noqa

def get_complex_pub_cli_action(*, ):
    cmd = ['ros2', 'pub', '']
    cmd.extend(test_configuration.arguments)
    command_under_test = ExecuteProcess(
        cmd=cmd,
        name='cli',
        output='screen',
        sigterm_timeout=LaunchConfiguration(
            'sigterm_timeout', default=60
        )
    )


def get_talker_node_action(*, node_name='my_talker', node_namespace='my_ns', topic_name='chatter'):
    return launch_ros.actions.Node(
        package='demo_nodes_py', node_executable='talker', node_name=node_name,
        node_namespace=node_namespace, remappings=[('chatter', topic_name)], output='screen',
        sigterm_timeout=LaunchConfiguration('sigterm_timeout', default=30)
    )

def get_listener_node_action(*, node_name='my_listener', node_namespace='my_ns', topic_name='chatter'):
    return launch_ros.actions.Node(
        package='demo_nodes_py', node_executable='listener', node_name=node_name,
        node_namespace=node_namespace, remappings=[('chatter', topic_name)], output='screen',
        sigterm_timeout=LaunchConfiguration('sigterm_timeout', default=30)
    )


test_configurations = [
    CLITestConfiguration(
        command='topic',
        arguments=['list'],
        actions=[get_talker_node_action(), get_talker_node_action(topic_name='_chatter')],
        expected_output=['/chatter', '/parameter_events', '/rosout'],
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['list', '--include-hidden-topics'],
        actions=[get_talker_node_action(), get_talker_node_action(topic_name='_chatter')],
        expected_output=['/_chatter', '/chatter', '/parameter_events', '/rosout'],
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['list', '-t'],
        actions=[get_talker_node_action()],
        expected_output=[
            '/chatter [std_msgs/msg/String]',
            '/parameter_events [rcl_interfaces/msg/ParameterEvent]',
            '/rosout [rcl_interfaces/msg/Log]'
        ],
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['list', '-c'],
        actions=[get_talker_node_action()],
        expected_output=[lambda line: int(line) == 3],
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['info', '/chatter'],
        actions=[get_talker_node_action()],
        expected_output=[
            'Topic: /chatter'
            'Publisher count: 1',
            'Subscriber count: 0'
        ],
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['info', '/some_chatter'],
        actions=[get_talker_node_action()],
        expected_output=[
            'Topic: /some_chatter'
            'Publisher count: 0',
            'Subscriber count: 0'
        ],
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['type', '/chatter'],
        actions=[get_talker_node_action()],
        expected_output=['std_msgs/msg/String'],
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['type', '/_chatter'],
        actions=[get_talker_node_action(topic_name='_chatter')],
        expected_output=None,
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['find', 'rcl_interfaces/msg/Log'],
        actions=[get_talker_node_action()],
        expected_output=['/rosout'],
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['find', 'rcl_interfaces/msg/NotAMessageType'],
        actions=[get_talker_node_action()],
        expected_output=None,
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['find', 'rcl_interfaces/msg/NotAMessageType'],
        actions=[get_talker_node_action()],
        expected_output=None,
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['echo', '/chatter'],
        actions=[get_talker_node_action()],
        expected_output=itertools.cycle([
            re.compile(r"data: 'Hello World: \d+'"),
            '---'
        ]),
        timeout=2,
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['echo', '--no-str', '/chatter'],
        actions=[get_talker_node_action()],
        expected_output=itertools.cycle([
            re.compile(r"data: '<string length: <\d+>>'"),
            '---'
        ]),
        timeout=2,
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['echo', '--truncate-length 5', '/chatter'],
        actions=[get_talker_node_action()],
        expected_output=itertools.cycle([
            re.compile(r"data: Hello..."),
            '---'
        ]),
        timeout=2,
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['pub', '/chatter', 'std_msgs/msg/String', "'{data: test}'"],
        fixture_actions=[get_listener_node_action()],
        expected_output={
            'cli': itertools.chain(
                ['publisher: beginning loop'],
                itertools.cycle([
                    re.compile(r"publishing #\d+: std_msgs.msg.String(data='test')"),
                    ''
                ])
            ),
            'listener': itertools.repeat("[INFO] [my_listener]: I heard: [test]")
        },
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['pub', '--once', '/chatter', 'std_msgs/msg/String', "'{data: test}'"],
        expected_output=[
            'publisher: beginning loop',
            re.compile(r"publishing #1: std_msgs.msg.String(data='test')"),
            ''
        ],
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['pub', '-r 2', '/chatter', 'std_msgs/msg/String', "'{data: test}'"],
        expected_output=[
            'publisher: beginning loop',
            *([re.compile(r"publishing #\d+: std_msgs.msg.String(data='test')"), '']*3),
        ],
        timeout=2
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['pub', '-r 2', '/chatter', 'std_msgs/msg/String', "'{data: test}'"],
        expected_output=[
            'publisher: beginning loop',
            *([re.compile(r"publishing #\d+: std_msgs.msg.String(data='test')"), '']*3),
        ],
        timeout=2
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['pub', '-p 2', '/chatter', 'std_msgs/msg/String', "'{data: test}'"],
        expected_output=[
            'publisher: beginning loop',
            "publishing #1: std_msgs.msg.String(data='test')"
            '',
        ],
        timeout=4
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['pub', '-p 2', '/chatter', 'std_msgs/msg/String', "'{data: test}'"],
        expected_output=[
            'publisher: beginning loop',
            "publishing #1: std_msgs.msg.String(data='test')"
            '',
        ],
        timeout=4
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['delay', '/chatter'],
        fixture_actions=[get_complex_talker_node_action()],
        expected_output=itertools.cycle([
            re.compile(r'average delay: \d{,2}.\d{3}'),
            re.compile(r'\s*min: \d{,2}.\d{3}s max: \d{,2}.\d{3}s std dev: \d+.\d{5}s window: \d+')
        ]),
        timeout=4
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['hz', '/chatter'],
        fixture_actions=[get_talker_node_action()],
        expected_output=itertools.cycle([
            re.compile(r'average rate: 1.00\d'),
            re.compile(r'\s*min: 1.00\ds max: 1.00\ds std dev: 0.000\d{2}s window: \d+')
        ]),
        timeout=4
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['hz', '--filter', "'int(m.data.rpartition(\":\")[-1]) % 2 == 0'",  '/chatter'],
        fixture_actions=[get_talker_node_action()],
        expected_output=itertools.cycle([
            re.compile(r'average rate: 0.50\d'),
            re.compile(r'\s*min: 0.50\ds max: 0.50\ds std dev: 0.000\d{2}s window: \d+')
        ]),
        timeout=4
    ),
    CLITestConfiguration(
        command='topic',
        arguments=['bw', '/chatter'],
        fixture_actions=[get_talker_node_action()],
        expected_output=itertools.chain(['Subscribed to [/chatter]'], itertools.cycle([
            re.compile(r'average: 24.\d{2}B/s'),
            re.compile(r'\s*mean: 24.\d{2}B/s min: 24.\d{2}B/s max: 24.\d{2}B/s window: \d+')
        ]),
        timeout=4
    ),
]
