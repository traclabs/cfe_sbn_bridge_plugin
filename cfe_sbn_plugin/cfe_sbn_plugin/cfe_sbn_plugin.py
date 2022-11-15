#!/usr/bin/env python3

import os
import struct

from fsw_ros2_bridge.fsw_plugin_interface import FSWPluginInterface

from cfe_sbn_plugin.sbn_receiver import SBNReceiver
from cfe_sbn_plugin.telem_handler import TelemHandler
from cfe_sbn_plugin.command_handler import CommandHandler

# from fsw_ros2_bridge.telem_info import TelemInfo
# from fsw_ros2_bridge.command_info import CommandInfo

from ament_index_python.packages import get_package_share_directory

from cfe_sbn_bridge_msgs.srv import Subscribe
from cfe_sbn_bridge_msgs.srv import Unsubscribe
from cfe_sbn_bridge_msgs.srv import TriggerROSHousekeeping

from juicer_util.juicer_interface import JuicerInterface
from juicer_util.parse_cfe_config import ParseCFEConfig


class FSWPlugin(FSWPluginInterface):

    def __init__(self, node):

        self._node = node
        self._node.get_logger().info("Setting up cFE-SBN bridge plugin")
        # self._routing_service = None

        self._node.declare_parameter('plugin_params.cfs_root', '~/code/cFS')
        self._cfs_root = self._node.get_parameter('plugin_params.cfs_root').get_parameter_value(). \
            string_value
        if '~' in self._cfs_root:
            self._cfs_root = os.path.expanduser(self._cfs_root)
        self._node.get_logger().info("  using cfs_root: " + self._cfs_root)

        self._node.declare_parameter('plugin_params.msg_pkg', 'cfe_msgs')
        self._msg_pkg = self._node.get_parameter('plugin_params.msg_pkg').get_parameter_value(). \
            string_value
        self._node.get_logger().info("  using msg_pkg: " + self._msg_pkg)
        self._cfs_msgs_dir = get_package_share_directory(self._msg_pkg)

        self._node.declare_parameter('plugin_params.udp_receive_port', 1234)
        self._udp_receive_port = self._node.get_parameter('plugin_params.udp_receive_port'). \
            get_parameter_value().integer_value
        self._node.get_logger().info("  using udp_receive_port: " + str(self._udp_receive_port))

        self._node.declare_parameter('plugin_params.udp_send_port', 1235)
        self._udp_send_port = self._node.get_parameter('plugin_params.udp_send_port'). \
            get_parameter_value().integer_value
        self._node.get_logger().info("  using udp_send_port: " + str(self._udp_send_port))

        self._node.declare_parameter('plugin_params.udp_ip', '127.0.0.1')
        self._udp_ip = self._node.get_parameter('plugin_params.udp_ip').get_parameter_value(). \
            string_value
        self._node.get_logger().info("  using udp_ip: " + self._udp_ip)

        self._node.declare_parameter("plugin_params.processor_id", 0x2)
        self._processor_id = self._node.get_parameter('plugin_params.processor_id').get_parameter_value().integer_value
        self._node.declare_parameter("plugin_params.spacecraft_id", 0x42)
        self._spacecraft_id = self._node.get_parameter('plugin_params.spacecraft_id').get_parameter_value().integer_value
        
        # Create the subscribe/unsubscribe services.
        self._subscribe_srv = self._node.create_service(Subscribe,
                                                        '/cfe_sbn_bridge/subscribe',
                                                        self.subscribe_callback)
        self._unsubscribe_srv = self._node.create_service(Unsubscribe,
                                                          '/cfe_sbn_bridge/unsubscribe',
                                                          self.unsubscribe_callback)
        self._trigger_ros_hk_srv = self._node.create_service(TriggerROSHousekeeping,
                                                             '/cfe_sbn_bridge/trigger_ros_hk',
                                                             self.trigger_ros_hk_callback)

        resource_path = get_package_share_directory("cfe_plugin") + "/resource/"
        self._juicer_interface = JuicerInterface(self._node, resource_path)

        # these lists will hold information about the message structures and MIDs once we are
        # parsing the info from the param file and the juicer sql databases
        self._telem_info = self._juicer_interface.get_telemetry_message_info()
        self._command_info = self._juicer_interface.get_command_message_info()

        command_params = ["structure", "cfe_mid", "cmd_code", "topic_name"]
        telemetry_params = ["cfe_mid", "topic_name"]
        cfe_config = ParseCFEConfig(self._node, command_params, telemetry_params)
        cfe_config.print_commands()
        cfe_config.print_telemetry()
        self._command_dict = cfe_config.get_command_dict()
        self._telemetry_dict = cfe_config.get_telemetry_dict()
        self._telem_handler = TelemHandler(self._node, self._msg_pkg, self._telemetry_dict, self._juicer_interface)

        self._recv_map = {}

        # set up callbacks for commands from ROS
        self._command_info = self._juicer_interface.reconcile_command_info(self._command_info, self._command_dict)
        for ci in self._command_info:
            key = ci.get_key()
            cmd_ids = self._command_dict[key]
            ch = CommandHandler(self._node, ci, self.command_callback, int(cmd_ids['cfe_mid'], 16), cmd_ids['cmd_code'])
            ci.set_callback_func(ch.process_callback)

        ###########################################################
        ###########################################################
        # initialize/configure connection to SBN application here
        ###########################################################
        ###########################################################
        self._node.get_logger().info("Setting up connection to SBN application!!")

        self._sbn_receiver = SBNReceiver(self._node, self._udp_ip, self._udp_receive_port, self.telem_callback)

        # TODO: self._sbn_sender should be deprecated in favor of SBNReceiver.peers
        # TODO: Update cfg yaml to define a list of peers
        self._sbn_sender = self._sbn_receiver.add_peer(self._udp_ip, self._udp_send_port, 0x42, 1)


        # TESTING subscribe to housekeeping tlm message for testing
        self._sbn_sender.send_subscription_msg(0x800)

    def get_telemetry_message_info(self):
        return self._telem_info

    def get_command_message_info(self):
        return self._command_info

    def get_latest_data(self, key):
        retval = None
        if key in self._recv_map:
            retval = self._recv_map[key]
        return retval

    def create_ros_msgs(self, msg_dir):
        # place holder for now
        msg_list = []
        return msg_list

    def get_msg_package(self):
        return self._msg_pkg

    def subscribe_callback(self, request, response):
        self._node.get_logger().info('Subscribe()')
        self._sbn_sender.send_subscription_msg(request.message_id)
        return response

    def unsubscribe_callback(self, request, response):
        self._node.get_logger().info('Unsubscribe()')
        self._sbn_sender.send_unsubscription_msg(request.message_id)
        return response

    def trigger_ros_hk_callback(self, request, response):
        self._node.get_logger().info('TriggerROSHk()')
        cfe_message = struct.pack("BBBBBBBB", 0x18, 0x97, 0xC0, 0x00, 0x00, 0x01, 0x00, 0x00)
        self._sbn_sender.send_cfe_message_msg(cfe_message)
        return response

    def command_callback(self, command_info, message):
        key_name = command_info.get_key()
        self._node.get_logger().info('Handling cmd ' + key_name)
        cmd_ids = self._command_dict[key_name]
        packet = self._juicer_interface.parse_command(command_info, message, cmd_ids['cfe_mid'], cmd_ids['cmd_code'])
        self._sbn_sender.send_cfe_message_msg(packet)
        self._node.get_logger().info('Command ' + ros_name + ' sent.')

    def telem_callback(self, msg):
        # handle telemetry from cFE
        self._node.get_logger().info('Handling telemetry message')
        (ros_name, msg) = self._telem_handler.handle_packet(msg)
        self._recv_map[ros_name] = msg
