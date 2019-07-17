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
import re
import sys

import itertools

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

sys.path.append(os.path.dirname(__file__))

from cli_test_configuration import CLITestConfiguration  # noqa


def get_add_two_ints_server(service_name=None):
    remappings = None
    if service_name is not None:
        remappings = [('add_two_ints', service_name)]
    return Node(
        package='demo_nodes_py', node_executable='add_two_ints_server',
        node_namespace='my_ns', output='screen', remappings=remappings,
        sigterm_timeout=LaunchConfiguration('sigterm_timeout', default=30)
    )

def get_add_two_ints_call_output(*, a=0, b=0):
    return [
        'requester: making request: ' +
        'example_interfaces.srv.AddTwoInts_Request(a={}, b={})'.format(a, b),
        '',
        'response:',
        'example_interfaces.srv.AddTwoInts_Response(sum={})'.format(a + b)
    ]

test_configurations = [
    CLITestConfiguration(
        command='service',
        arguments=['list'],
        actions=[get_add_two_ints_server(),
                 get_add_two_ints_server(service_name='_hidden_add_two_ints')],
        expected_output=['/add_two_ints', re.compile(r'/add_two_ints_server/.*parameter.*')],
    ),
    CLITestConfiguration(
        command='service',
        arguments=[' --include-hidden-services list'],
        actions=[get_add_two_ints_server(),
                 get_add_two_ints_server(service_name='_add_two_ints')],
        expected_output=['/_add_two_ints', '/add_two_ints'],
    ),
    CLITestConfiguration(
        command='service',
        arguments=['list -t'],
        actions=[get_add_two_ints_server()],
        expected_output=itertools.chain([
            '/add_two_ints [example_interfaces/srv/AddTwoInts]',
        ], itertools.repeat(
            re.compile(r'/add_two_ints_server/.*parameter.* \[rcl_interfaces/srv/.*Parameter.*\]')
        )),
    ),
    CLITestConfiguration(
        command='service',
        arguments=['list -c'],
        actions=[get_add_two_ints_server()],
        expected_output=[lambda line: int(line) > 0],
    ),
    CLITestConfiguration(
        command='service',
        arguments=['find', 'example_interfaces/srv/AddTwoInts'],
        actions=[get_add_two_ints_server()],
        expected_output=['/add_two_ints'],
    ),
    CLITestConfiguration(
        command='service',
        arguments=['find', '-c', 'example_interfaces/srv/AddTwoInts'],
        actions=[get_add_two_ints_server()],
        expected_output=[lambda line: int(line) == 1],
    ),
    CLITestConfiguration(
        command='service',
        arguments=['find', '--include-hidden-services',
                   'example_interfaces/srv/AddTwoInts'],
        actions=[get_add_two_ints_server(),
                 get_add_two_ints_server(service_name='_add_two_ints')],
        expected_output=['/_add_two_ints', '/add_two_ints'],
    ),
    CLITestConfiguration(
        command='service',
        arguments=['type', '/add_two_ints'],
        actions=[get_add_two_ints_server()],
        expected_output=['example_interfaces/srv/AddTwoInts'],
    ),
    CLITestConfiguration(
        command='service',
        arguments=['call', '/add_two_ints'],
        actions=[get_add_two_ints_server()],
        expected_output=['waiting for service to become available...',
                         *get_add_two_ints_call_output()],
    ),
    CLITestConfiguration(
        command='service',
        arguments=['call', '/add_two_ints', '{a: 1, b: -1}'],
        actions=[get_add_two_ints_server()],
        expected_output=['waiting for service to become available...',
                         *get_add_two_ints_call_output(a=1, b=-1)],
    ),
    CLITestConfiguration(
        command='service',
        arguments=['call', '/add_two_ints', '{a: 1, b: 1}', '-r 0.5'],
        actions=[get_add_two_ints_server()],
        expected_output=['waiting for service to become available...',
                         *get_add_two_ints_call_output(a=1, b=1)],
        exit_codes=[2],
        timeout=1
    ),
    CLITestConfiguration(
        command='service',
        arguments=['call', '/add_two_ints', '{a: 1, b: 1}', '-r 1'],
        actions=[get_add_two_ints_server()],
        expected_output=['waiting for service to become available...',
                         *(get_add_two_ints_call_output(a=1, b=1) * 2)],
        exit_codes=[2],
        timeout=2
    ),
]
