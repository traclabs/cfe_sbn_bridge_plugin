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
        self._node.declare_parameter("plugin_params.epoch_delta", 315532800)
        self._epoch_delta = self._node.get_parameter('plugin_params.epoch_delta').get_parameter_value().integer_value
        self._node.get_logger().info("Epoch delta from config file: " + str(self._epoch_delta))
        self._node.declare_parameter("plugin_params.rosout_name_max_length", 32)
        self._rosout_name_max_length = self._node.get_parameter('plugin_params.rosout_name_max_length').get_parameter_value().integer_value
        self._node.declare_parameter("plugin_params.rosout_msg_max_length", 128)
        self._rosout_msg_max_length = self._node.get_parameter('plugin_params.rosout_msg_max_length').get_parameter_value().integer_value
        self._node.declare_parameter("plugin_params.rosout_file_max_length", 64)
        self._rosout_file_max_length = self._node.get_parameter('plugin_params.rosout_file_max_length').get_parameter_value().integer_value
        self._node.declare_parameter("plugin_params.rosout_function_max_length", 32)
        self._rosout_function_max_length = self._node.get_parameter('plugin_params.rosout_function_max_length').get_parameter_value().integer_value

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

        command_params = ["structure", "cfe_mid", "cmd_code", "topic_name", "port"]
        telemetry_params = ["structure", "cfe_mid", "topic_name", "port"]
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
        symbol_name_map = self._juicer_interface.get_symbol_ros_name_map()
        for ci in self._command_info:
            key = ci.get_key()
            cmd_ids = self._command_dict[key]
            msg_size = symbol_name_map[ci.get_msg_type()].get_size()
            ch = CommandHandler(self._node, ci, self.command_callback, int(cmd_ids['cfe_mid'], 16), cmd_ids['cmd_code'], msg_size)
            ci.set_callback_func(ch.process_callback)

        ###########################################################
        ###########################################################
        # initialize/configure connection to SBN application here
        ###########################################################
        ###########################################################
        self._node.get_logger().info("Setting up connection to SBN application!!")

        self._sbn_receiver = SBNReceiver(self._node, self._udp_ip, self._udp_receive_port, self.telem_callback)

        self._subscription_scanning_timer_period = 0.5
        self._subscription_scanning_timer = self._node.create_timer(self._subscription_scanning_timer_period, 
                                                                    self.subscription_scanning_timer_callback)

        # TODO: Update cfg yaml to define a list of peers.
        self._sbn_receiver.add_peer(self._udp_ip, self._udp_send_port, 0x42, 1)

        # Testing.  Subscribe to the /rosout topic to get the rosout messages.  This is on the flight side
        self._subscribe_srv = self._node.create_subscription(Log, '/rosout', self.rosout_callback, 10)

    def subscription_scanning_timer_callback(self):
        # self._node.get_logger().info('Subscription Timer Scanner()')
        for k in self._telemetry_dict.keys():
            topic = self._telemetry_dict[k]['topic_name']
            info = self._node.get_subscriptions_info_by_topic(topic)
            if len(info) > 0:
                # This means that a local node is subscribed to this topic.
                SBNPeer.send_all_subscription_msg(int(self._telemetry_dict[k]['cfe_mid'], 16))
            else:
                # No local node is subscribed to this topic.
                SBNPeer.send_all_unsubscription_msg(int(self._telemetry_dict[k]['cfe_mid'], 16))


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
        SBNPeer.send_all_subscription_msg(request.message_id)
        return response

    ## Called on receipt of ROS /cfe_sbn_bridge/unsubscribe
    # @param request.message_id CFE Message ID to unsubscribe to
    def unsubscribe_callback(self, request, response):
        self._node.get_logger().info('Unsubscribe()')
        SBNPeer.send_all_unsubscription_msg(request.message_id)
        return response

    def trigger_ros_hk_callback(self, request, response):
        self._node.get_logger().info('TriggerROSHk()')
        cfe_message = struct.pack("BBBBBBBB", 0x18, 0x97, 0xC0, 0x00, 0x00, 0x01, 0x00, 0x00)
        SBNPeer.send_all(cfe_message)  # TODO: .send(0x1897, cfe_message))
        return response

    def convert_rosout_msg_level_to_packet_id(self, l):
        mid = ""
        if l == 10:  # Debug
            mid = struct.pack("BB", 0x08, 0x98)
        elif l == 20:  # Info
            mid = struct.pack("BB", 0x08, 0x99)
        elif l == 30:  # WARN
            mid = struct.pack("BB", 0x08, 0x9A)
        elif l == 40:  # ERROR
            mid = struct.pack("BB", 0x08, 0x9B)
        elif l == 50:  # FATAL
            mid = struct.pack("BB", 0x08, 0x9C)
        else:
            # Unhandled is treated as FATAL
            mid = struct.pack("BB", 0x08, 0x9C)
        return mid

    def convert_rosout_msg_type_to_packet_seq_ctrl(self):
        return struct.pack("BB", 0xC0, 0x00)

    def convert_rosout_msg_to_cfe_msg_size(self, size):
        packet_size_field = size - 7
        return struct.pack(">h", packet_size_field)

    def convert_rosout_secondary_header(self, sec, nsec):
        # UNIX epoch is seconds since January 1, 1970 (midnight UTC/GMT)
        # CFE epoch is defined in the CFE build (by default it is seconds since Jan 1, 1980 midnight UTC/GMT)
        # Got the epoch delta from https://www.epochconverter.com/
        epoch_delta = self._epoch_delta
        met_seconds = int(sec - epoch_delta) & 0xffffffff
        # For subseconds, need to convert number of nanoseconds into number of 1/65536 seconds.
        conversion_factor_nsec_to_subsec = 65536.0 / 1000000000.0
        met_subseconds = int(float(nsec) * conversion_factor_nsec_to_subsec) & 0xffff
        return struct.pack(">IH", met_seconds, met_subseconds)

    def construct_rosout_header(self, msg, packet_size):
        rosout_header_obj = self._juicer_interface.get_symbol_info("CFE_MSG_TelemetryHeader")
        size = rosout_header_obj.get_size()

        pri_hdr = (self.convert_rosout_msg_level_to_packet_id(msg.level) +
                    self.convert_rosout_msg_type_to_packet_seq_ctrl() +
                    self.convert_rosout_msg_to_cfe_msg_size(packet_size))

        msg_mapping = {"Msg": pri_hdr,
                        "Sec": self.convert_rosout_secondary_header(msg.stamp.sec, msg.stamp.nanosec),
                        "Spare": struct.pack("BBBB", 0, 0, 0, 0)}

        # Form the message
        rosout_msg = bytes('', 'utf-8')
        for field in rosout_header_obj.get_fields():
            rosout_msg += msg_mapping[field.get_name()]

        # Verify the sizes match
        self._node.get_logger().debug('ROSOUT HEADER MSG: ' + str(size) + ", len: " + str(len(rosout_msg)))
        # assert(size == len(rosout_msg))
        return rosout_msg

    def construct_rosout_payload(self, msg):
        rosout_payload_obj = self._juicer_interface.get_symbol_info("ROS_APP_Rosout_Payload_t")
        size = rosout_payload_obj.get_size()
        msg_mapping = {'sec': struct.pack(">I", msg.stamp.sec),
                       'nsec': struct.pack(">I", msg.stamp.nanosec),
                       'level': struct.pack("B", msg.level),
                       'name_truncated': struct.pack("?", len(msg.name) > self._rosout_name_max_length),
                       'name': struct.pack(str(self._rosout_name_max_length) + "s", bytearray(msg.name[-self._rosout_name_max_length:], 'utf-8')),
                       'msg_truncated': struct.pack("?", len(msg.msg) > self._rosout_msg_max_length),
                       'msg': struct.pack(str(self._rosout_msg_max_length) + "s", bytearray(msg.msg[-self._rosout_msg_max_length:], 'utf-8')),
                       'file_truncated': struct.pack("?", len(msg.file) > self._rosout_file_max_length),
                       'file': struct.pack(str(self._rosout_file_max_length) + "s", bytearray(msg.file[-self._rosout_file_max_length:], 'utf-8')),
                       'function_truncated': struct.pack("?", len(msg.function) > self._rosout_function_max_length),
                       'function': struct.pack(str(self._rosout_function_max_length) + "s", bytearray(msg.function[-self._rosout_function_max_length:], 'utf-8')),
                       '_spare0': struct.pack("BBB", 0, 0, 0),
                       'line': struct.pack(">I", msg.line)}

        # Form the message
        rosout_msg = bytes('', 'utf-8')
        for field in rosout_payload_obj.get_fields():
            rosout_msg += msg_mapping[field.get_name()]

        # Verify the sizes match
        self._node.get_logger().debug('ROSOUT PAYLOAD MSG: ' + str(size) + ", len: " + str(len(rosout_msg)))
        # assert(size == len(rosout_msg))
        return rosout_msg

    def rosout_callback(self, msg):
        size = self._juicer_interface.get_symbol_info("ROS_APP_RosoutTlm_t").get_size()
        header = self.construct_rosout_header(msg, size)
        payload = self.construct_rosout_payload(msg)
        self._node.get_logger().debug('ROSOUT CALLBACK MSG: ' + str(header) + ", len: " + str(len(payload)))
        # assert(size == (len(header) + len(payload)))

        SBNPeer.send_all(header + payload)  # DEBUG: Send unconditionally
        #SBNPeer.send( packet_id, log_msg ) # Send only if peer has subscribed

    def command_callback(self, command_info, message):
        key_name = command_info.get_key()
        cmd_ids = self._command_dict[key_name]
        packet = self._juicer_interface.parse_command(command_info, message, cmd_ids['cfe_mid'], cmd_ids['cmd_code'])
        SBNPeer.send( int(cmd_ids['cfe_mid'], 16), packet )

    ## Message handler callback
    # @param msg Parse a received cFE CCSDS message
    # @param peer SBNPeer reference to originating peer (ie: for logging purposes)
    def telem_callback(self, msg, peer):
        # handle telemetry from cFE
        (key, msg) = self._telem_handler.handle_packet(msg)
        # self._node.get_logger().info('Handling telemetry message for ' + key)
        self._recv_map[key] = msg
