#!/usr/bin/env python3

import os

from fsw_ros2_bridge.fsw_plugin_interface import FSWPluginInterface

from cfe_sbn_plugin.sbn_receiver import SBNReceiver
from cfe_sbn_plugin.sbn_sender import SBNSender
# from fsw_ros2_bridge.telem_info import TelemInfo
# from fsw_ros2_bridge.command_info import CommandInfo

from ament_index_python.packages import get_package_share_directory


class FSWPlugin(FSWPluginInterface):

    def __init__(self, node):

        self._node = node
        self._node.get_logger().info("Setting up cFE-SBN bridge plugin")
        # self._routing_service = None

        self._node.declare_parameter('plugin_params.cfs_root', '~/code/cFS')
        self._cfs_root = self._node.get_parameter('plugin_params.cfs_root').get_parameter_value().\
            string_value
        if '~' in self._cfs_root:
            self._cfs_root = os.path.expanduser(self._cfs_root)
        self._node.get_logger().info("  using cfs_root: " + self._cfs_root)

        self._node.declare_parameter('plugin_params.msg_pkg', 'cfe_msgs')
        self._msg_pkg = self._node.get_parameter('plugin_params.msg_pkg').get_parameter_value().\
            string_value
        self._node.get_logger().info("  using msg_pkg: " + self._msg_pkg)
        self._cfs_msgs_dir = get_package_share_directory(self._msg_pkg)

        self._node.declare_parameter('plugin_params.udp_receive_port', 1234)
        self._udp_receive_port = self._node.get_parameter('plugin_params.udp_receive_port').\
            get_parameter_value().integer_value
        self._node.get_logger().info("  using udp_receive_port: " + str(self._udp_receive_port))

        self._node.declare_parameter('plugin_params.udp_send_port', 1235)
        self._udp_send_port = self._node.get_parameter('plugin_params.udp_send_port').\
            get_parameter_value().integer_value
        self._node.get_logger().info("  using udp_send_port: " + str(self._udp_send_port))

        self._node.declare_parameter('plugin_params.udp_ip', '127.0.0.1')
        self._udp_ip = self._node.get_parameter('plugin_params.udp_ip').get_parameter_value().\
            string_value
        self._node.get_logger().info("  using udp_ip: " + self._udp_ip)

        self._node.declare_parameter("plugin_params.processor_id", 0x2)
        self._processor_id = self._node.get_parameter('plugin_params.processor_id').get_parameter_value().integer_value
        self._node.declare_parameter("plugin_params.spacecraft_id", 0x42)
        self._spacecraft_id = self._node.get_parameter('plugin_params.spacecraft_id').get_parameter_value().integer_value
        

        # these lists will hold information about the message structures and MIDs once we are
        # parsing the info from the param file and the juicer sql databases
        self._telem_info = []
        self._command_info = []

        self._recv_map = {}

        ###########################################################
        ###########################################################
        # initialize/configure connection to SBN application here
        ###########################################################
        ###########################################################
        self._node.get_logger().info("Setting up connection to SBN application!!")
        self._sbn_sender = SBNSender(self._node, self._udp_ip, self._udp_send_port, self._spacecraft_id, self._processor_id)
        self._sbn_receiver = SBNReceiver(self._node, self._udp_ip, self._udp_receive_port,
                                         self._sbn_sender)

    def get_telemetry_message_info(self):
        return self._telem_info

    def get_command_message_info(self):
        return self._command_info

    def get_latest_data(self, key):
        return self._recv_map[key].get_latest_data()

    def create_ros_msgs(self, msg_dir):
        # place holder for now
        msg_list = []
        return msg_list

    def get_msg_package(self):
        return self._msg_pkg
