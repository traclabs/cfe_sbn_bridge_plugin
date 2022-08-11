#!/usr/bin/env python3

from fsw_ros2_bridge.fsw_plugin_interface import FSWPluginInterface

# from fsw_ros2_bridge.telem_info import TelemInfo
# from fsw_ros2_bridge.command_info import CommandInfo

# from cfe_sbn_plugin.telem_pages_info import TelemPagesInfo
# from cfe_sbn_plugin.cmd_pages_info import CmdPagesInfo

# from cfe_sbn_plugin.cfs_telem_reciever import CFSTelemReciever
# from cfe_sbn_plugin.cfs_command_broadcaster import CFSCommandBroadcaster

# from cfe_sbn_plugin.routing_service import RoutingService

# from pathlib import Path

from ament_index_python.packages import get_package_share_directory


class FSWPlugin(FSWPluginInterface):

    def __init__(self, node):

        self._node = node
        self._node.get_logger().info("Setting up cFE-SBN bridge plugin")

        # self._routing_service = None

        self._node.declare_parameter('plugin_params.cfs_root', '/home/swhart/code/cFS')
        self._cfs_root = self._node.get_parameter('plugin_params.cfs_root').get_parameter_value().\
            string_value

        self._node.get_logger().info("  using cfs_root: " + self._cfs_root)

        self._root_dir = self._cfs_root + "/tools/cFS-GroundSystem/Subsystems"
        # self._tlm_def_file = f"{self._root_dir}/tlmGUI/telemetry-pages.txt"
        # self._cmd_def_file = f"{self._root_dir}/cmdGui/command-pages.txt"

        self._msg_pkg = "cfs_groundsystem_msgs"
        self._cfs_msgs_dir = get_package_share_directory(self._msg_pkg)

        # self._telem_pages_info = TelemPagesInfo(self._tlm_def_file, self._cfs_msgs_dir)
        # self._cmd_pages_info = CmdPagesInfo(self._cmd_def_file, self._cfs_msgs_dir)

        self._telem_info = []
        self._command_info = []

        self._recv_map = {}
        # for i in range(self._telem_pages_info.get_telem_map_size()):
        #     key = self._telem_pages_info.get_telem_desc(i)
        #     # if key != "ES HK Tlm": continue
        #     def_file = self._telem_pages_info.get_telem_def_file(i)
        #     msg_type = self._telem_pages_info.get_ros_msg_name(i)
        #     if def_file == "null":
        #         continue
        #     self._recv_map[key] = CFSTelemReciever(self._telem_pages_info, key,
        #                                            self._telem_pages_info.get_telem_app_id(i),
        #                                            f"{self._root_dir}/tlmGUI/", def_file)
        #     topic_name = Path(def_file).stem.replace("-", "_")
        #     t = TelemInfo(key, msg_type, topic_name)
        #     self._telem_info.append(t)

        # self._broad_map = {}
        # for i in range(self._cmd_pages_info.get_cmd_map_size()):
        #     key = self._cmd_pages_info.get_cmd_desc(i)
        #     def_file = self._cmd_pages_info.get_cmd_def_file(i)
        #     msg_type = self._cmd_pages_info.get_ros_msg_name(i)
        #     if def_file == "":
        #         continue
        #     topic_name = Path(def_file).stem.replace("-", "_").lower()
        #     topic_name = topic_name.replace("__", "_")
        #     self._broad_map[key] = CFSCommandBroadcaster(self._node, self._cmd_pages_info, key,
        #                                                  self._cmd_pages_info.get_cmd_app_id(i),
        #                                                  f"{self._root_dir}/cmdGui/",
        #                                                  def_file, topic_name)
        #     c = CommandInfo(key, msg_type, topic_name, self._broad_map[key].process_callback)
        #     self._command_info.append(c)

        # self.init_routing_service()

    # def init_routing_service(self):
    #     self._routing_service = RoutingService(self._node)
    #     self._routing_service.start()

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
