"""! @brief SBN Receiver Implementation """

import socket
import struct

from cfe_sbn_plugin.sbn_peer import SBNPeer

class SBNReceiver():

    def __init__(self, node, udp_ip, udp_port):
        self._node = node
        self._udp_ip = udp_ip
        self._udp_port = udp_port
        self._timer_period = 0.1

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((self._udp_ip, self._udp_port))

        # Original: timer based read, with added non-blocking logic
        self._sock.setblocking(False)
        self._timer = self._node.create_timer(self._timer_period, self.timer_callback)

    ## Register a Peer (formerly sender)
    def add_peer(self, udp_ip, udp_port, sc_id, proc_id):
        # TODO: replace self._sender with self._peers list and function to find by sc_id/proc_id
        self._sender = SBNPeer(self._node, udp_ip, udp_port, sc_id, proc_id, self._sock)
        return self._sender

    ## Find a Peer by sc_id and proc_id (TODO)
    def find_peer(self, sc_id, proc_id):
        # TODO: Validate sc_id/proc_id.  Throw an exception if peer is not recognized
        return self._sender # TODO: Only a single peer supported at present

    def timer_callback(self):
        try:
            data, addr = self._sock.recvfrom(1024)  # buffer size is 1024 bytes
            self._node.get_logger().info(str(self.handle_sbn_msg(data)))
        except socket.error:
            pass

    ## Convert integer msg_type to string name
    def get_msg_type_name(self, msg_type):
        msg_type_name = "Unknown"

        if msg_type == 0:
            msg_type_name = "SBN_NO_MSG"
        elif msg_type == 1:
            msg_type_name = "SBN_SUB_MSG"
        elif msg_type == 2:
            msg_type_name = "SBN_UNSUB_MSG"
        elif msg_type == 3:
            msg_type_name = "SBN_APP_MSG (payload is SB msg)"
        elif msg_type == 4:
            msg_type_name = "SBN_PROTO_MSG (payload is SBN protocol number)"
        elif msg_type == 0xA0:
            msg_type_name = "SBN_UDP_HEARTBEAT_MSG"
        elif msg_type == 0xA1:
            msg_type_name = "SBN_UDP_ANNOUNCE_MSG"
        elif msg_type == 0xA2:
            msg_type_name = "SBN_UDP_DISCONN_MSG"

        return msg_type_name

    def read_half_word(self, msg):
        (hw, ) = struct.unpack(">H", msg[0:2])
        return (hw, msg[2:])

    def read_full_word(self, msg):
        (fw, ) = struct.unpack(">L", msg[0:4])
        return (fw, msg[4:])

    def read_bytes(self, msg, num_bytes):
        bs = msg[0:num_bytes]
        return (bs, msg[num_bytes:])

    def parse_sbn_header(self, msg):
        if len(msg) >= 11:
            message_size, msg = self.read_half_word(msg)
            message_type, msg = self.read_bytes(msg, 1)
            processorID, msg = self.read_full_word(msg)
            spacecraftID, msg = self.read_full_word(msg)
            return (message_size,
                    int.from_bytes(message_type, byteorder='big'),
                    processorID,
                    spacecraftID,
                    msg)
        else:
            return None

    ## Parse a raw subscription or unsubscription message
    def parse_sbn_sub_msg(self, msg):
        git_id, msg = self.read_bytes(msg, 48)
        subscription_count, msg = self.read_half_word(msg)
        subscriptions = []

        for i in range(subscription_count):
            message_id, msg = self.read_full_word(msg)
            qos_priority, msg = self.read_bytes(msg, 1)
            qos_reliability, msg = self.read_bytes(msg, 1)
            subscriptions.append((message_id,
                                  int.from_bytes(qos_priority, byteorder='big'),
                                  int.from_bytes(qos_reliability, byteorder='big')))

        return (git_id, subscriptions)
    
    def process_sbn_subscription_msg(self, msg, peer):
        git_id,subscriptions = self.parse_sbn_sub_msg(msg)
        return peer.add_subscriptions(subscriptions)

    def process_sbn_unsubscription_msg(self, msg, peer):
        # Unsubscription messages are the same as subscription messages.
        git_id,subscriptions = self.parse_sbn_sub_msg(msg)
        return peer.del_subscriptions(subscriptions)
        
    def process_sbn_cfe_message_msg(self, msg):
        # cfe messages are just the payload of the message
        return msg

    def process_sbn_protocol_msg(self, msg):
        protocol_id = self.read_bytes(msg, 1)

        # TODO: Verify protocol_id matches expected

        return (protocol_id)

    def handle_sbn_msg(self, msg):
        results = self.parse_sbn_header(msg)
        if results is not None:

            (message_size, message_type, processorID, spacecraftID, remaining_msg) = results

            peer = self.find_peer(spacecraftID, processorID)

            # Update connection/heartbeat status for this peer
            peer.connected()

            # Process the message
            if message_type == 1:
                # subscription message
                return (self.get_msg_type_name(message_type),
                        processorID,
                        spacecraftID,
                        self.process_sbn_subscription_msg(remaining_msg, peer))
            elif message_type == 2:
                # unsubscription message
                return (self.get_msg_type_name(message_type),
                        processorID,
                        spacecraftID,
                        self.process_sbn_unsubscription_msg(remaining_msg, peer))
            elif message_type == 3:
                # sbn cfe message transfer
                return (self.get_msg_type_name(message_type),
                        processorID,
                        spacecraftID,
                        self.process_sbn_cfe_message_msg(remaining_msg))
            elif message_type == 4:
                # sbn protocol message
                return (self.get_msg_type_name(message_type),
                        processorID,
                        spacecraftID,
                        self.process_sbn_protocol_msg(remaining_msg))
            elif message_type == 0xA0:
                # SBN_UDP_HEARTBEAT_MSG
                # Send a heartbeat back 
                peer.send_heartbeat()

                # Return the received heartbeat message fields.
                return (self.get_msg_type_name(message_type),
                        processorID,
                        spacecraftID)
            elif message_type == 0xA1:
                # SBN_UDP_ANNOUNCE_MSG
                return (self.get_msg_type_name(message_type),
                        processorID,
                        spacecraftID)
            elif message_type == 0xA2:
                # SBN_UDP_DISCONN_MSG
                return (self.get_msg_type_name(message_type),
                        processorID,
                        spacecraftID)
            else:
                self._node.get_logger().error("unknown message type")
