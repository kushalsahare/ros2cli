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

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.append(os.path.dirname(__file__))

from cli_test_configuration import CLITestConfiguration  # noqa


def get_add_two_ints_server_action(*, node_name='my_add_two_ints_server',
                                   node_namespace='my_ns',
                                   service_name='add_two_ints'):
    return Node(
        package='demo_nodes_py', node_executable='add_two_ints_server',
        node_name=node_name, node_namespace=node_namespace,
        output='screen', remappings=[('add_two_ints', service_name)],
        sigterm_timeout=LaunchConfiguration('sigterm_timeout', default=30)
    )


def get_add_two_ints_call_output(*, a=0, b=0):
    return [
        'requester: making request: ' +
        'example_interfaces.srv.AddTwoInts_Request(a={}, b={})'.format(a, b),
        '',
        'response:',
        'example_interfaces.srv.AddTwoInts_Response(sum={})'.format(a + b),
        ''
    ]


test_configurations = [
    CLITestConfiguration(
        command='service',
        arguments=['list'],
        fixture_actions=[get_add_two_ints_server_action(),
                         get_add_two_ints_server_action(service_name='_add_two_ints')],
        expected_output=itertools.chain(
            # Cope with launch internal ROS 2 node.
            itertools.repeat(re.compile(r'/launch_ros/.*parameter.*'), 6),
            ['/my_ns/add_two_ints'],
            itertools.repeat(re.compile(
                r'/my_ns/my_add_two_ints_server/.*parameter.*'
            ), 6)
        )
    ),
    CLITestConfiguration(
        command='service',
        arguments=['--include-hidden-services', 'list'],
        fixture_actions=[get_add_two_ints_server_action(),
                         get_add_two_ints_server_action(service_name='_add_two_ints')],
        expected_output=itertools.chain(
            # Cope with launch internal ROS 2 node.
            itertools.repeat(re.compile(r'/launch_ros/.*parameter.*'), 6),
            ['/my_ns/_add_two_ints', '/my_ns/add_two_ints'],
            itertools.repeat(re.compile(
                r'/my_ns/my_add_two_ints_server/.*parameter.*'
            ), 6)
        )
    ),
    CLITestConfiguration(
        command='service',
        arguments=['list', '-t'],
        fixture_actions=[get_add_two_ints_server_action()],
        expected_output=itertools.chain(
            # Cope with launch internal ROS 2 node.
            itertools.repeat(re.compile(
                r'/launch_ros/.*parameter.* \[rcl_interfaces/srv/.*Parameter.*\]'
            ), 6),
            ['/my_ns/add_two_ints [example_interfaces/srv/AddTwoInts]'],
            itertools.repeat(re.compile(
                r'/my_ns/my_add_two_ints_server/.*parameter.* \[rcl_interfaces/srv/.*Parameter.*\]'
            ), 6)
        )
    ),
    CLITestConfiguration(
        command='service',
        arguments=['list', '-c'],
        fixture_actions=[get_add_two_ints_server_action()],
        expected_output=[lambda line: int(line) > 0]
    ),
    CLITestConfiguration(
        command='service',
        arguments=['find', 'example_interfaces/srv/AddTwoInts'],
        fixture_actions=[get_add_two_ints_server_action()],
        expected_output=['/my_ns/add_two_ints']
    ),
    CLITestConfiguration(
        command='service',
        arguments=['find', '-c', 'example_interfaces/srv/AddTwoInts'],
        fixture_actions=[get_add_two_ints_server_action()],
        expected_output=[lambda line: int(line) == 1]
    ),
    CLITestConfiguration(
        command='service',
        arguments=['find', 'not_a_service_type'],
        expected_output=None
    ),
    CLITestConfiguration(
        command='service',
        arguments=['find', '--include-hidden-services',
                   'example_interfaces/srv/AddTwoInts'],
        fixture_actions=[get_add_two_ints_server_action(),
                         get_add_two_ints_server_action(service_name='_add_two_ints')],
        expected_output=['/my_ns/_add_two_ints', '/my_ns/add_two_ints'],
    ),
    CLITestConfiguration(
        command='service',
        arguments=['type', '/not_a_service'],
        expected_output=None,
        exit_codes=[1]
    ),
    CLITestConfiguration(
        command='service',
        arguments=['type', '/my_ns/add_two_ints'],
        fixture_actions=[get_add_two_ints_server_action()],
        expected_output=['example_interfaces/srv/AddTwoInts'],
    ),
    CLITestConfiguration(
        command='service',
        arguments=['call', '/not_a_service', 'not_a_service_type'],
        expected_output=['The passed service type is invalid'],
        exit_codes=[1]
    ),
    CLITestConfiguration(
        command='service',
        arguments=['call', '/my_ns/add_two_ints', 'example_interfaces/srv/AddTwoInts'],
        fixture_actions=[get_add_two_ints_server_action()],
        expected_output=[
            'waiting for service to become available...',
            *get_add_two_ints_call_output()
        ],
    ),
    CLITestConfiguration(
        command='service',
        arguments=[
            'call',
            '/my_ns/add_two_ints',
            'example_interfaces/srv/AddTwoInts',
            '{a: 1, b: -1}'
        ],
        fixture_actions=[get_add_two_ints_server_action()],
        expected_output=[
            'waiting for service to become available...',
            *get_add_two_ints_call_output(a=1, b=-1)
        ],
    ),
    CLITestConfiguration(
        command='service',
        arguments=[
            'call',
            '-r', '0.5',
            '/my_ns/add_two_ints',
            'example_interfaces/srv/AddTwoInts',
            '{a: 1, b: 1}'
        ],
        fixture_actions=[get_add_two_ints_server_action()],
        expected_output=[
            'waiting for service to become available...',
            *get_add_two_ints_call_output(a=1, b=1)
        ],
        self_terminates=False,
        exit_codes=[2],
        timeout=1.0
    ),
    CLITestConfiguration(
        command='service',
        arguments=[
            'call',
            '-r', '1',
            '/my_ns/add_two_ints',
            'example_interfaces/srv/AddTwoInts',
            '{a: 1, b: 1}'
        ],
        fixture_actions=[get_add_two_ints_server_action()],
        expected_output=[
            'waiting for service to become available...',
            *(get_add_two_ints_call_output(a=1, b=1) * 3)
        ],
        self_terminates=False,
        exit_codes=[2],
        timeout=2.0
    )
]
