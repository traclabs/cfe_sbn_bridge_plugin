import socket
import struct
import rclpy

## TODO: Rename from SBNSender to SBNPeer, an object representing remote SBN instances
class SBNSender():

    def __init__(self, node, udp_ip, udp_port, sc_id, proc_id, sock):

        self._node = node
        self._udp_ip = udp_ip
        self._udp_port = udp_port

        # Local Identifier
        self._spacecraft_id = self._node.get_parameter('plugin_params.spacecraft_id').get_parameter_value().integer_value
        self._processor_id = self._node.get_parameter('plugin_params.processor_id').get_parameter_value().integer_value

        # Peer Identifier (remote)
        self._peer_spacecraft_id = sc_id
        self._peer_processor_id = proc_id
        
        self._rev_id_string = b'$Id: dccf6239093d99c4c9351e140c15b61a95d8fc37 $\x00'

        self._connected = False
        self._last_heartbeat_rx = self._node.get_clock().now()
        self._last_tx           = self._node.get_clock().now()
        self._timer_period = 0.1
        self._timer = self._node.create_timer(self._timer_period, self.timer_callback);
        
        self._sock = sock  # We reuse common socket so reply port/address is correct
        
        self.send_protocol_msg()

    ## Timer validates received heartbeats and transmits when appropriate
    def timer_callback(self):
        if self._connected == False:
            return # No heartbeat processing needed if !connected
        
        # If we haven't received a heartbeat in a while, we should
        # say we aren't connected.
        hb_delta = self._node.get_clock().now() - self._last_heartbeat_rx
        if hb_delta > rclpy.time.Duration(seconds=10.0):
            self._connected = False
            self._node.get_logger().info("Disconnected from Peer " + str(self._peer_spacecraft_id) + ":" + str(self._peer_processor_id))

        # If we haven't sent a message in a while, we should send an explicit heartbeat
        # Note: If connection is stale, this may trigger remote to reconnect
        hb_delta = self._node.get_clock().now() - self._last_tx
        if hb_delta > rclpy.time.Duration(seconds=1.0): # TODO: Verify HB frequency
            self.send_heartbeat()
            
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

    def _send(self, msg):
        self._sock.sendto(bytes(msg), (self._udp_ip, self._udp_port))
        self._last_tx = self._node.get_clock().now()
    
    def send_heartbeat(self):
        msg_size = 0
        msg_type = 0xA0

        heartbeat_msg = []
        heartbeat_msg = self.write_half_word(msg_size, heartbeat_msg)
        heartbeat_msg = self.write_bytes([msg_type], heartbeat_msg)
        heartbeat_msg = self.write_full_word(self._processor_id, heartbeat_msg)
        heartbeat_msg = self.write_full_word(self._spacecraft_id, heartbeat_msg)

        self._send(bytes(heartbeat_msg))

    def send_protocol_msg(self):
        msg_size = 1
        msg_type = 4
        protocol_id = 11

        protocol_msg = []
        protocol_msg = self.write_half_word(msg_size, protocol_msg)
        protocol_msg = self.write_bytes([msg_type], protocol_msg)
        protocol_msg = self.write_full_word(self._processor_id, protocol_msg)
        protocol_msg = self.write_full_word(self._spacecraft_id, protocol_msg)
        protocol_msg = self.write_bytes([protocol_id], protocol_msg)

        self._send(bytes(protocol_msg))

    def send_subscription_msg(self, mid):
        msg_size = 56
        msg_type = 1
        sbn_sub_count = 1
        sbn_sub_msg_id = mid
        sbn_sub_qos_priority = 0
        sbn_sub_qos_reliability = 0

        subscription_msg = []
        subscription_msg = self.write_half_word(msg_size, subscription_msg)
        subscription_msg = self.write_bytes([msg_type], subscription_msg)
        subscription_msg = self.write_full_word(self._processor_id, subscription_msg)
        subscription_msg = self.write_full_word(self._spacecraft_id, subscription_msg)
        subscription_msg = self.write_bytes(self._rev_id_string, subscription_msg)
        subscription_msg = self.write_half_word(sbn_sub_count, subscription_msg)
        subscription_msg = self.write_full_word(sbn_sub_msg_id, subscription_msg)
        subscription_msg = self.write_bytes([sbn_sub_qos_priority, sbn_sub_qos_reliability],
                                            subscription_msg)

        self._send(bytes(subscription_msg))

    ## When a Peer is connected send required information
    def connected(self):
        self._last_heartbeat_rx = self._node.get_clock().now()
        
        if not self._connected:
            self._connected = True
        
            self.send_protocol_msg()
            #self.send_subscription_msg() # TODO: Resend all known subscriptions
            self._node.get_logger().info("Connected to Peer " + str(self._peer_spacecraft_id) + ":" + str(self._peer_processor_id))

    def send_unsubscription_msg(self, mid):
        msg_size = 56
        msg_type = 2
        sbn_sub_count = 1
        sbn_sub_msg_id = mid
        sbn_sub_qos_priority = 0
        sbn_sub_qos_reliability = 0

        subscription_msg = []
        subscription_msg = self.write_half_word(msg_size, subscription_msg)
        subscription_msg = self.write_bytes([msg_type], subscription_msg)
        subscription_msg = self.write_full_word(self._processor_id, subscription_msg)
        subscription_msg = self.write_full_word(self._spacecraft_id, subscription_msg)
        subscription_msg = self.write_bytes(self._rev_id_string, subscription_msg)
        subscription_msg = self.write_half_word(sbn_sub_count, subscription_msg)
        subscription_msg = self.write_full_word(sbn_sub_msg_id, subscription_msg)
        subscription_msg = self.write_bytes([sbn_sub_qos_priority, sbn_sub_qos_reliability],
                                            subscription_msg)

        self._send(bytes(subscription_msg))

    def send_cfe_message_msg(self, msg_bytes):
        msg_size = len(msg_bytes)
        msg_type = 3

        protocol_msg = []
        protocol_msg = self.write_half_word(msg_size, protocol_msg)
        protocol_msg = self.write_bytes([msg_type], protocol_msg)
        protocol_msg = self.write_full_word(self._processor_id, protocol_msg)
        protocol_msg = self.write_full_word(self._spacecraft_id, protocol_msg)
        protocol_msg = self.write_bytes(msg_bytes, protocol_msg)

        self._send(bytes(protocol_msg))

    def add_subscriptions(self, subscriptions):
        for sub in subscriptions:
            self._node.get_logger().info("TODO: Subscribe to "+str(sub[0]))
            # TODO: Add to list of subs requested by this peer if not already present and subscribe to topic in ROS
        return subscriptions

    def del_subscriptions(self, subscriptions):
        for sub in subscriptions:
            self._node.get_logger().info("TODO: Unsubscribe from "+str(sub[0]))
            # TODO: If subscription is in list for this peer, remove it. If no other peers have an active subscription to this topic, then unsubscribe in ros.
        return subscriptions
