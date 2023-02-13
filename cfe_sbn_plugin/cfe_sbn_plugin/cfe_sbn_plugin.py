#!/usr/bin/env python3

import os
import struct

from fsw_ros2_bridge.fsw_plugin_interface import FSWPluginInterface

from cfe_sbn_plugin.sbn_receiver import SBNReceiver
from cfe_sbn_plugin.telem_handler import TelemHandler
from cfe_sbn_plugin.command_handler import CommandHandler
from cfe_sbn_plugin.sbn_peer import SBNPeer

# from fsw_ros2_bridge.telem_info import TelemInfo
# from fsw_ros2_bridge.command_info import CommandInfo

from ament_index_python.packages import get_package_share_directory

from rcl_interfaces.msg import Log

from cfe_sbn_bridge_msgs.srv import Subscribe
from cfe_sbn_bridge_msgs.srv import Unsubscribe
from cfe_sbn_bridge_msgs.srv import TriggerROSHousekeeping

from juicer_util.juicer_interface import JuicerInterface
from juicer_util.parse_cfe_config import ParseCFEConfig


class FSWPlugin(FSWPluginInterface):

    def __init__(self, node):
        ## ROS Node pointer
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
        telemetry_params = ["structure", "cfe_mid", "topic_name"]
        cfe_config = ParseCFEConfig(self._node, command_params, telemetry_params)
        cfe_config.print_commands()
        cfe_config.print_telemetry()
        self._command_dict = cfe_config.get_command_dict()
        self._telemetry_dict = cfe_config.get_telemetry_dict()

        ## TelemHandler
        self._telem_handler = TelemHandler(self._node, self._msg_pkg, self._telemetry_dict, self._juicer_interface)
        self._telem_info = self._juicer_interface.reconcile_telem_info(self._telem_info, self._telemetry_dict)

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

        ## Array of dictionaries defining subscriptions that ROS is requesting from all Peers
        # Default value list is for testing housekeeping tlm message
        # TESTING subscribe to housekeeping tlm message for testing
        self._ros_subscriptions = [0x800,0x898,0x899,0x89A,0x89B,0x89C]
        
        # TODO: Update cfg yaml to define a list of peers.
        self._sbn_receiver.add_peer(self._udp_ip, self._udp_send_port, 0x42, 1, self._ros_subscriptions)

        # Testing.  Subscribe to the /rosout topic to get the rosout messages.  This is on the flight side
        self._subscribe_srv = self._node.create_subscription(Log, '/rosout', self.rosout_callback, 10)


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

    ## Called on receipt of ROS /cfe_sbn_bridge/subscribe
    # @param request.message_id CFE Message ID to subscribe to
    def subscribe_callback(self, request, response):
        self._node.get_logger().info('Subscribe()')

        if request.message_id in self._ros_subscriptions:
            self._node.get_logger().warn("Duplicate subscription request for " + request.message_id + " ignored.")
        else:
            self._ros_subscriptions.append(request.message_id); # Add to list
            SBNPeer.send_all_subscription_msg(request.message_id); # And send to all Peers immediately
        
        return response

    ## Called on receipt of ROS /cfe_sbn_bridge/unsubscribe
    # @param request.message_id CFE Message ID to unsubscribe to
    def unsubscribe_callback(self, request, response):
        self._node.get_logger().info('Unsubscribe()')
        SBNPeer.send_all_unsubscription_msg(request.message_id);
        
        self._ros_subscriptions.remove(request.message_id);

        return response

    def trigger_ros_hk_callback(self, request, response):
        self._node.get_logger().info('TriggerROSHk()')
        cfe_message = struct.pack("BBBBBBBB", 0x18, 0x97, 0xC0, 0x00, 0x00, 0x01, 0x00, 0x00)
        SBNPeer.send_all(cfe_message); # TODO: .send(0x1897, cfe_message))
        return response



    def convert_rosout_msg_level_to_packet_id(self, l):
        mid = ""
        if l == 10: # Debug
            mid = struct.pack("BB", 0x08, 0x98)
        elif l == 20: # Info
            mid = struct.pack("BB", 0x08, 0x99)
        elif l == 30: # WARN
            mid = struct.pack("BB", 0x08, 0x9A)
        elif l == 40: # ERROR
            mid = struct.pack("BB", 0x08, 0x9B)
        elif l == 50: # FATAL
            mid = struct.pack("BB", 0x08, 0x9C)
        else:
            # Unhandled is treated as FATAL
            mid = struct.pack("BB", 0x08, 0x9C)
        return mid

    def convert_rosout_msg_type_to_packet_seq_ctrl(self):
        return struct.pack("BB", 0xC0, 0x00)

    def convert_rosout_msg_to_cfe_msg_size(self):
        pri_header = 6
        sec_header = 6
        spare = 4
        name_bytes = 32
        msg_bytes = 128
        file_bytes = 64
        function_bytes = 32
        line_bytes = 4
        size = pri_header + sec_header + spare + name_bytes + msg_bytes + file_bytes + function_bytes + line_bytes
        packet_size_field = size - 7
        return struct.pack(">h", packet_size_field)

    def convert_rosout_name(self, n):
        # TODO Add the maxlen parameter as a setting in the config file -- this needs to match the CFE side
        truncated_flag = len(n) > 32
        # https://python-reference.readthedocs.io/en/latest/docs/functions/bytearray.html
        return struct.pack("?32s", truncated_flag, bytearray(n[-32:], 'utf-8'))

    def convert_rosout_msg(self, m):
        # TODO Add the maxlen parameter as a setting in the config file -- this needs to match the CFE side
        truncated_flag = len(m) > 128
        return struct.pack("?128s", truncated_flag, bytearray(m[-128:], 'utf-8'))

    def convert_rosout_file(self, f):
        # TODO Add the maxlen parameter as a setting in the config file -- this needs to match the CFE side
        truncated_flag = len(f) > 64
        return struct.pack("?64s", truncated_flag, bytearray(f[-64:], 'utf-8'))

    def convert_rosout_function(self, fn):
        # TODO Add the maxlen parameter as a setting in the config file -- this needs to match the CFE side
        truncated_flag = len(fn) > 32
        return struct.pack("?32s", truncated_flag, bytearray(fn[-32:], 'utf-8'))

    def convert_rosout_secondary_header(self, sec, nsec):
        # UNIX epoch is seconds since January 1, 1970 (midnight UTC/GMT)
        # CFE epoch is defined in the CFE build (by default it is seconds since Jan 1, 1980 midnight UTC/GMT)
        # Got the epoch delta from https://www.epochconverter.com/
        epoch_delta = 315532800  # TODO Get this from the configuration file somehow.
        met_seconds = int(sec - epoch_delta) & 0xffffffff
        # For subseconds, need to convert number of nanoseconds into number of 1/65536 seconds.
        conversion_factor_nsec_to_subsec = 65536.0 / 1000000000.0
        met_subseconds = int(float(nsec) * conversion_factor_nsec_to_subsec) & 0xffff
        return struct.pack(">IH", met_seconds, met_subseconds)

    def rosout_callback(self, msg):
        # if msg.level == 20:
            # self._node.get_logger().debug('Handling rosout message')
        packet_id = self.convert_rosout_msg_level_to_packet_id(msg.level)
        packet_seq_ctrl = self.convert_rosout_msg_type_to_packet_seq_ctrl()
        packet_data_length = self.convert_rosout_msg_to_cfe_msg_size()
        packet_secondary_header = self.convert_rosout_secondary_header(msg.stamp.sec, msg.stamp.nanosec)
        log_msg = ""
        log_msg = ( packet_id +
                    packet_seq_ctrl +
                    packet_data_length +
                    packet_secondary_header +
                    struct.pack("BBBB", 0, 0, 0, 0) + # 4 bytes of padding
                    struct.pack("IIB", msg.stamp.sec, msg.stamp.nanosec, msg.level) + # sec, nsec, level
                    self.convert_rosout_name(msg.name) +
                    self.convert_rosout_msg(msg.msg) +
                    self.convert_rosout_file(msg.file) +
                    self.convert_rosout_function(msg.function) +
                    struct.pack("I", msg.line)
                )
        SBNPeer.send_all(log_msg)  # DEBUG: Send unconditionally
        #SBNPeer.send( packet_id, log_msg ) # Send only if peer has subscribed

    def command_callback(self, command_info, message):
        key_name = command_info.get_key()
        self._node.get_logger().info('Handling cmd ' + key_name)
        cmd_ids = self._command_dict[key_name]
        packet = self._juicer_interface.parse_command(command_info, message, cmd_ids['cfe_mid'], cmd_ids['cmd_code'])
        SBNPeer.send_all(packet); # TODO: replace with send(packet_id,packet)
        self._node.get_logger().info('Command ' + ros_name + ' sent.')

    ## Message handler callback
    # @param msg Parse a received cFE CCSDS message
    # @param peer SBNPeer reference to originating peer (ie: for logging purposes)
    def telem_callback(self, msg, peer):
        # handle telemetry from cFE
        (key, msg) = self._telem_handler.handle_packet(msg)
        self._node.get_logger().info('Handling telemetry message for ' + key)
        self._recv_map[key] = msg
