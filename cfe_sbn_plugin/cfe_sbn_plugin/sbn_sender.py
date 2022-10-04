import socket
import struct


class SBNSender():

    def __init__(self, node, udp_ip, udp_port):

        self._node = node
        self._udp_ip = udp_ip
        self._udp_port = udp_port
        self._rev_id_string = b'$Id: dccf6239093d99c4c9351e140c15b61a95d8fc37 $\x00'
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.send_protocol_msg()

    def write_half_word(self, hw, msg):
        bs = struct.pack(">H", hw)
        msg.extend(bs)
        return (msg)

    def write_full_word(self, fw, msg):
        bs = struct.pack(">L", fw)
        msg.extend(bs)
        return (msg)

    def write_bytes(self, values, msg):
        bs = []
        for v in values:
            b = struct.pack(">B", v & 255)
            bs.extend(b)
        msg.extend(bs)
        return (msg)

    def send_heartbeat(self):
        msg_size = 0
        msg_type = 0xA0
        processor_id = 0x2
        spacecraft_id = 0x42

        heartbeat_msg = []
        heartbeat_msg = self.write_half_word(msg_size, heartbeat_msg)
        heartbeat_msg = self.write_bytes([msg_type], heartbeat_msg)
        heartbeat_msg = self.write_full_word(processor_id, heartbeat_msg)
        heartbeat_msg = self.write_full_word(spacecraft_id, heartbeat_msg)

        self._sock.sendto(bytes(heartbeat_msg), (self._udp_ip, self._udp_port))

    def send_protocol_msg(self):
        msg_size = 1
        msg_type = 4
        processor_id = 0x2
        spacecraft_id = 0x42
        protocol_id = 11

        protocol_msg = []
        protocol_msg = self.write_half_word(msg_size, protocol_msg)
        protocol_msg = self.write_bytes([msg_type], protocol_msg)
        protocol_msg = self.write_full_word(processor_id, protocol_msg)
        protocol_msg = self.write_full_word(spacecraft_id, protocol_msg)
        protocol_msg = self.write_bytes([protocol_id], protocol_msg)

        self._sock.sendto(bytes(protocol_msg), (self._udp_ip, self._udp_port))

    def send_subscription_msg(self, mid):
        msg_size = 56
        msg_type = 1
        processor_id = 0x2
        spacecraft_id = 0x42
        sbn_sub_count = 1
        sbn_sub_msg_id = mid
        sbn_sub_qos_priority = 0
        sbn_sub_qos_reliability = 0

        subscription_msg = []
        subscription_msg = self.write_half_word(msg_size, subscription_msg)
        subscription_msg = self.write_bytes([msg_type], subscription_msg)
        subscription_msg = self.write_full_word(processor_id, subscription_msg)
        subscription_msg = self.write_full_word(spacecraft_id, subscription_msg)
        subscription_msg = self.write_bytes(self._rev_id_string, subscription_msg)
        subscription_msg = self.write_half_word(sbn_sub_count, subscription_msg)
        subscription_msg = self.write_full_word(sbn_sub_msg_id, subscription_msg)
        subscription_msg = self.write_bytes([sbn_sub_qos_priority, sbn_sub_qos_reliability],
                                            subscription_msg)

        self._sock.sendto(bytes(subscription_msg), (self._udp_ip, self._udp_port))

    def send_unsubscription_msg(self, mid):
        msg_size = 56
        msg_type = 2
        processor_id = 0x2
        spacecraft_id = 0x42
        sbn_sub_count = 1
        sbn_sub_msg_id = mid
        sbn_sub_qos_priority = 0
        sbn_sub_qos_reliability = 0

        subscription_msg = []
        subscription_msg = self.write_half_word(msg_size, subscription_msg)
        subscription_msg = self.write_bytes([msg_type], subscription_msg)
        subscription_msg = self.write_full_word(processor_id, subscription_msg)
        subscription_msg = self.write_full_word(spacecraft_id, subscription_msg)
        subscription_msg = self.write_bytes(self._rev_id_string, subscription_msg)
        subscription_msg = self.write_half_word(sbn_sub_count, subscription_msg)
        subscription_msg = self.write_full_word(sbn_sub_msg_id, subscription_msg)
        subscription_msg = self.write_bytes([sbn_sub_qos_priority, sbn_sub_qos_reliability],
                                            subscription_msg)

        self._sock.sendto(bytes(subscription_msg), (self._udp_ip, self._udp_port))
