from struct import unpack
import importlib
import sys

import juicer_util.juicer_interface
from cfe_msgs.msg import BinaryPktPayload  # Default binary packet format


class TelemHandler():
    def __init__(self, node, msg_pkg, telem_info, juicer_interface):
        self._node = node
        self._juicer_interface = juicer_interface
        self._msg_pkg = msg_pkg
        self._tlm_map = {}
        self._key_map = {}
        self._ros_topic_map = {}
        for tlm in telem_info:
            self._tlm_map[telem_info[tlm]['cfe_mid']] = telem_info[tlm]['structure']
            self._ros_topic_map[tlm] = telem_info[tlm]['topic_name']
            self._key_map[telem_info[tlm]['cfe_mid']] = str(tlm)
        self._node.get_logger().info('telem map is ' + str(self._tlm_map))

    def handle_packet(self, datagram):
        packet_id = self.get_pkt_id(datagram)
        if packet_id in self._tlm_map:
            ros_name = self._tlm_map[packet_id]
            
            if not ros_name:
                # Packet is defined in configuration, but no structure was specified, implying we should use default binary encoding
                self._node._logger.warn("Received packet for pid without struct name! Handling Under testing: " + str(packet_id))

                #@ Create Generic Binary Packet containing payload
                # ROS doesn't seem to allow a msg definition of a simple bytes array, so we must split it
                #  into an array of individual byte objects (byte[])
                # VERIFY: Is this correct and/or the most efficient way to do this?
                tmp_data = datagram[16:]
                msg_data = [i.to_bytes(1, sys.byteorder) for i in tmp_data]
                msg = BinaryPktPayload(data=msg_data)
            else:
                # self._node.get_logger().info("Received packet for " + ros_name)
                MsgType = getattr(importlib.import_module(self._msg_pkg + ".msg"), ros_name)

                msg = MsgType()
                self._juicer_interface.parse_packet(datagram, 0, self._tlm_map[packet_id], msg, self._msg_pkg)
                
            msg_attrs = dir(msg)
            if "seq" in msg_attrs:
                setattr(msg, "seq", self.get_seq_count(datagram))
            
            key = self._key_map[packet_id]
            return (key, msg)
        else:
            self._node.get_logger().warn("Don't know how to handle message id " + packet_id)
            return (packet_id, None)

    def get_pkt_id(self, datagram):
        streamid = unpack(">H", datagram[:2])
        return hex(streamid[0])

    def get_seq_count(self, datagram):
        seqcount = unpack(">H", datagram[2:4])
        return seqcount[0] & 0x3FFF  # sequence count mask
