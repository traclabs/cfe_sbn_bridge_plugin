# Copyright 2019 Apex.AI, Inc.
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
import unittest

import launch
import launch.actions

import launch_testing
import launch_testing.actions
from launch_testing.asserts import assertSequentialStdout

import pytest


@pytest.mark.launch_test
def generate_test_description():
    TEST_PROC_PATH = os.path.join(
        os.environ.get("CFE_ROOT"),
        'build/exe/cpu1'
    )

    TEST_PROC_EXE = os.path.join(
        TEST_PROC_PATH,
        'core-cpu1'
    )

    # This is necessary to get unbuffered output from the process under test
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'

    dut_process = launch.actions.ExecuteProcess(
        cmd=[TEST_PROC_EXE],
        cwd=TEST_PROC_PATH,
        env=proc_env, output='screen'
    )

    return launch.LaunchDescription([
        dut_process,

        # Start tests right away - no need to wait for anything
        launch_testing.actions.ReadyToTest(),
    ]), {'dut_process': dut_process}


class TestGoodCfeStartup(unittest.TestCase):
    def test_wait_for_msg(self, proc_output):
        proc_output.assertWaitFor('EVS Port1 66/1/CFE_TIME 21: Stop FLYWHEEL', timeout=30, stream='stdout')


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_info):
        # Check that all processes in the launch (in this case, there's just one) exit
        # with code 0
        launch_testing.asserts.assertExitCodes(proc_info)

    def test_cfe_shutdown(self, proc_output, dut_process):
        with assertSequentialStdout(proc_output, dut_process) as cm:
            cm.assertInStdout('CFE_PSP: Shutdown initiated - Exiting cFE')
