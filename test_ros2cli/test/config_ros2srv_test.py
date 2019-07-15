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

sys.path.append(os.path.dirname(__file__))

from test_config import TestConfig  # noqa

some_services_from_std_srvs = [
    'std_srvs/srv/Empty',
    'std_srvs/srv/SetBool',
    'std_srvs/srv/Trigger',
]

configs = [
    TestConfig(
        command='srv',
        arguments=['list'],
        expected_output=some_services_from_std_srvs,
    ),
    TestConfig(
        command='srv',
        arguments=['package', 'std_srvs'],
        expected_output=some_services_from_std_srvs,
    ),
    TestConfig(
        command='srv',
        arguments=['package', 'not_a_package_with_services'],
        expected_output=['Unknown package name'],
    ),
    TestConfig(
        command='srv',
        arguments=['packages'],
        expected_output=['std_srvs'],
    ),
    TestConfig(
        command='srv',
        arguments=['show', 'std_srvs/srv/SetBool'],
        expected_output=['bool data', '---', 'bool success', 'string message'],
    ),
    TestConfig(
        command='srv',
        arguments=['show', 'std_srvs/srv/Trigger'],
        expected_output=['---', 'bool success', 'string message'],
    ),
    TestConfig(
        command='srv',
        arguments=['show', 'std_srvs/srv/NotAService'],
        expected_output=['Unknown service name'],
    ),
]
