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
import time
import unittest

import launch
import launch_testing.actions
import launch.actions

import launch_testing
import launch_testing.actions

import pytest

import rclpy

from cfe_msgs.msg import TOLABEnableOutputCmdt
from cfe_msgs.msg import CFEESHousekeepingTlm


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

    cfe_process = launch.actions.ExecuteProcess(
        cmd=[TEST_PROC_EXE],
        cwd=TEST_PROC_PATH,
        env=proc_env, output='screen'
    )

    gsw_bridge_process = launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'cfe_plugin', 'cfe_bridge.launch.py'],
        env=proc_env, output='screen'
    )

    return launch.LaunchDescription([
        cfe_process,
        gsw_bridge_process,

        # Start tests right away - no need to wait for anything
        launch_testing.actions.ReadyToTest(),
    ]), {'cfe_process': cfe_process, 'gsw_bridge_process': gsw_bridge_process}


class TestEnableTOOutput(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        # Create a ROS node for tests
        self.node = rclpy.create_node('test_enable_to_output_node')

    def tearDown(self):
        self.node.destroy_node()

    def test_enable_to_output(self, proc_output):
        # Wait for both CFE and the Bridge to start.
        proc_output.assertWaitFor('EVS Port1 66/1/CFE_TIME 21: Stop FLYWHEEL', timeout=90, stream='stdout')
        proc_output.assertWaitFor('found message dictionary of size: 160', timeout=90, stream='stdout')

        # They've started, now create a publisher and publish the enable TO output command.
        pub = self.node.create_publisher(
            TOLABEnableOutputCmdt,
            '/groundsystem/to_lab_enable_output_cmd',
            10)

        msg = TOLABEnableOutputCmdt()
        msg.payload.dest_ip = '127.0.0.1'
        pub.publish(msg)

        # OK, now let's subscribe to the ES housekeeping.
        msgs_rx = []
        sub = self.node.create_subscription(
            CFEESHousekeepingTlm,
            '/groundsystem/cfe_es_hk_tlm',
            lambda msg: msgs_rx.append(msg),
            10
        )

        # Now lets check to make sure we received some TLM.
        try:
            # Wait until the bridge plugin sends 5 messages.
            end_time = time.time() + 10
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if len(msgs_rx) > 5:
                    break

            self.assertGreater(len(msgs_rx), 5)

            # Make sure the sequence numbers are sequentially increasing.
            previous_sequence_number = msgs_rx[0].telemetry_header.msg.ccsds.pri.sequence
            for msg in msgs_rx[1:]:
                this_sequence_number = msg.telemetry_header.msg.ccsds.pri.sequence
                # Assert that this sequence number is one more than the current sequence num
                self.assertEqual(this_sequence_number, previous_sequence_number + 1)
                previous_sequence_number = this_sequence_number
        finally:
            self.node.destroy_publisher(pub)
            self.node.destroy_subscription(sub)




#@launch_testing.post_shutdown_test()
#class TestProcessOutput(unittest.TestCase):
#    def test_exit_code(self, proc_info):
#        # Check that all processes in the launch (in this case, there's just one) exit
#        # with code 0
#        launch_testing.asserts.assertExitCodes(proc_info)
#
#    def test_cfe_shutdown(self, proc_output, cfe_process):
#        with assertSequentialStdout(proc_output, cfe_process) as cm:
#            cm.assertInStdout('CFE_PSP: Shutdown initiated - Exiting cFE')
