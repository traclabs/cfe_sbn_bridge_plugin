"""! @brief SBN Receiver Implementation """

import socket
import struct

from cfe_sbn_plugin.sbn_peer import SBNPeer
from cfe_sbn_plugin.messages import *  # Provides SBNMessageHdr and assorted byte list utils


class SBNReceiver():

    def __init__(self, node, udp_ip, udp_port, tlm_callback):
        self._node = node
        self._udp_ip = udp_ip
        self._udp_port = udp_port
        self._timer_period = 0.1
        self._tlm_callback = tlm_callback
        self._recv_buff_size = 4096

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((self._udp_ip, self._udp_port))
        self._sock.setblocking(False)

        self._timer = self._node.create_timer(self._timer_period, self.timer_callback)

    ## Register a Peer (formerly sender)
    # Note: Peer requires a reference to receiver, hence the need for this wrapper.
    def add_peer(self, udp_ip, udp_port, sc_id, proc_id):
        return SBNPeer.add(self._node, udp_ip, udp_port, sc_id, proc_id, self._sock)

    def timer_callback(self):
        while True:
            try:
                # receive message
                data, addr = self._sock.recvfrom(self._recv_buff_size)

                # ignore data if not long enough (doesn't contain header)
                if len(data) < 6:
                    return

                self._node.get_logger().debug(str(self.handle_sbn_msg(data)))
            except socket.error:
                return

    ## Parse a raw subscription or unsubscription message
    def parse_sbn_sub_msg(self, msg):
        git_id, msg = read_bytes(msg, 48)
        subscription_count, msg = read_half_word(msg)
        subscriptions = []

        for i in range(subscription_count):
            message_id, msg = read_full_word(msg)
            qos_priority, msg = read_bytes(msg, 1)
            qos_reliability, msg = read_bytes(msg, 1)
            subscriptions.append({
                "mid": message_id,
                "qos_priority": int.from_bytes(qos_priority, byteorder='big'),
                "qos_reliability": int.from_bytes(qos_reliability, byteorder='big')
            })

        return (git_id, subscriptions)

    def process_sbn_subscription_msg(self, msg, peer):
        git_id, subscriptions = self.parse_sbn_sub_msg(msg)
        return peer.add_subscriptions(subscriptions)

    def process_sbn_unsubscription_msg(self, msg, peer):
        # Unsubscription messages are the same as subscription messages.
        git_id, subscriptions = self.parse_sbn_sub_msg(msg)
        return peer.del_subscriptions(subscriptions)

    def process_sbn_cfe_message_msg(self, msg):
        # cfe messages are just the payload of the message
        return msg

    def process_sbn_protocol_msg(self, msg):
        protocol_id = read_bytes(msg, 1)
        # TODO: Verify protocol_id matches expected

        return (protocol_id)

    def handle_sbn_msg(self, msg):
        hdr = SBNMessageHdr.from_bytes(msg)

        if hdr is not None:
            # Find peer definition (TODO: or .add if new? need src ip/port to do so).
            peer = SBNPeer.find(hdr.sc_id, hdr.proc_id)
            if peer is None:
                self._node.get_logger().error("Received message from unknown peer (" + str(hdr.sc_id) + ", " + str(hdr.proc_id) + "), discarding")

            # Update connection/heartbeat status for this peer
            peer.connected()

            # Process the message
            if hdr.sbn_type == 1:
                # subscription message
                return (hdr,
                        self.process_sbn_subscription_msg(hdr.msg, peer))
            elif hdr.sbn_type == 2:
                # unsubscription message
                return (hdr,
                        self.process_sbn_unsubscription_msg(hdr.msg, peer))
            elif hdr.sbn_type == 3:
                # sbn cfe message transfer
                self._tlm_callback(hdr.msg, peer)
                return None
            elif hdr.sbn_type == 4:
                # sbn protocol message
                return (hdr,
                        self.process_sbn_protocol_msg(hdr.msg))
            elif hdr.sbn_type == 0xA0:
                # SBN_UDP_HEARTBEAT_MSG
                # Send a heartbeat back
                peer.send_heartbeat()

                # Return the received heartbeat message fields.
                return hdr
            elif hdr.sbn_type == 0xA1:
                # SBN_UDP_ANNOUNCE_MSG
                return hdr

            elif hdr.sbn_type == 0xA2:
                # SBN_UDP_DISCONN_MSG
                return hdr
            else:
                self._node.get_logger().error("unknown message type (" + str(hdr.sbn_type) + ")")
