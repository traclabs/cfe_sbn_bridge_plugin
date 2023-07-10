import socket
import struct
import rclpy

from cfe_sbn_plugin.messages import SBNMessageHdr, write_bytes, write_half_word, write_full_word


## SBNPeer class manages set of peer instances, where each instance represents a remote SBN instance.
class SBNPeer():

    peers = []

    ## Add a new Peer definition
    @classmethod
    def add(cls, node, udp_ip, udp_port, sc_id, proc_id, sock):

        peer = cls.find(sc_id, proc_id)
        if peer:
            # TODO: Log potential duplicate or ip/port update

            # Update definition
            peer.udp_ip = udp_ip
            peer.udp_port = udp_port
        else:
            peer = SBNPeer(node, udp_ip, udp_port, sc_id, proc_id, sock)
            cls.peers.append(peer)
        return peer

    ## Send given message to any peer that has previously requested it
    @classmethod
    def send(cls, mid, msg):
        for peer in cls.peers:
            peer.send_cfe_if_subscribed(mid, msg)

    ## Find a peer with matching sc_id and proc_id
    @classmethod
    def find(cls, sc_id, proc_id):
        for peer in cls.peers:
            if peer._peer_spacecraft_id == sc_id and peer._peer_processor_id == proc_id:
                return peer

        return None

    ## Unconditionally send message to all peers, ignoring subscription state
    @classmethod
    def send_all(cls, msg):
        for peer in cls.peers:
            peer.send_cfe_message_msg(msg)

    ## Send a subscription request to all peers
    @classmethod
    def send_all_subscription_msg(cls, message_id):
        for peer in cls.peers:
            if message_id not in peer._ros_subscriptions:
                peer._node.get_logger().info('Subscribing to MID 0x%04x over SBN' % message_id)
                peer._ros_subscriptions.append(message_id)
                peer.send_subscription_msg(message_id)

        ## Send a unsubscription request to all peers
    @classmethod
    def send_all_unsubscription_msg(cls, message_id):
        for peer in cls.peers:
            if message_id in peer._ros_subscriptions:
                peer._node.get_logger().info('Unsubscribing from MID 0x%04x over SBN' % message_id)
                peer._ros_subscriptions.remove(message_id)
                peer.send_unsubscription_msg(message_id)

    def __init__(self, node, udp_ip, udp_port, sc_id, proc_id, sock):
        self._node = node
        self._udp_ip = udp_ip
        self._udp_port = udp_port

        ## Local Spacecraft Identifier
        self._spacecraft_id = self._node.get_parameter('plugin_params.spacecraft_id').get_parameter_value().integer_value

        ## Local Processor Identifier
        self._processor_id = self._node.get_parameter('plugin_params.processor_id').get_parameter_value().integer_value

        ## Peer Spacecraft Identifier (remote)
        self._peer_spacecraft_id = sc_id

        ## Peer Processor Identifier (rz
        self._peer_processor_id = proc_id

        self._rev_id_string = b'$Id: dccf6239093d99c4c9351e140c15b61a95d8fc37 $\x00'

        self._connected = False
        self._last_heartbeat_rx = self._node.get_clock().now()
        self._last_tx = self._node.get_clock().now()
        self._timer_period = 0.1
        self._timer = self._node.create_timer(self._timer_period, self.timer_callback)

        self._subscriptions = {}  # Dictionary of active Peer subscriptions (received from peer)

        # Set of subscriptions ROS is requesting from peer(s)
        # Note: If multiple peers are supported, caller may share this
        # list across all peers, or may utilize distinct lists if
        # desired. CFE_SBN natively does not natively support
        # subscribing from a particular peer in FSW
        self._ros_subscriptions = []

        self._sock = sock  # We reuse common socket so reply port/address is correct

        self.send_protocol_msg()

    ## Timer validates received heartbeats and transmits when appropriate
    def timer_callback(self):
        if self._connected == False:
            self._subscriptions = {} # When we are not connected, we have no subscriptions from peers.
            return  # No heartbeat processing needed if !connected

        # If we haven't received a heartbeat in a while, we should
        # say we aren't connected.
        hb_delta = self._node.get_clock().now() - self._last_heartbeat_rx
        if hb_delta > rclpy.time.Duration(seconds=10.0):
            self._connected = False
            self._node.get_logger().info("Disconnected from Peer " + str(self._peer_spacecraft_id) + ":" + str(self._peer_processor_id))

        # If we haven't sent a message in a while, we should send an explicit heartbeat
        # Note: If connection is stale, this may trigger remote to reconnect
        hb_delta = self._node.get_clock().now() - self._last_tx
        if hb_delta > rclpy.time.Duration(seconds=1.0):  # TODO: Verify HB frequency
            self.send_heartbeat()

    ## Create a new SBN message of specified length
    def write_header(self, msg_size, msg_type, msg=None):
        msg = SBNMessageHdr(msg_size, msg_type, self._processor_id, self._spacecraft_id, msg)
        return msg.serialize()

    ## Send raw packet
    #  @returns bytes sent
    def _send(self, msg):
        self._last_tx = self._node.get_clock().now()
        return self._sock.sendto(bytes(msg), (self._udp_ip, self._udp_port))

    def send_heartbeat(self):
        msg_size = 0
        msg_type = 0xA0

        heartbeat_msg = self.write_header(msg_size, msg_type)

        self._send(heartbeat_msg)

    def send_protocol_msg(self):
        msg_size = 1  # VERIFY: this doesn't look right
        msg_type = 4
        protocol_id = 11

        protocol_msg = self.write_header(msg_size, msg_type)
        protocol_msg = write_bytes([protocol_id], protocol_msg)

        self._send(bytes(protocol_msg))

    ## Sends subscription request to this Peer
    ## @param mid A single message ID, or a list of message IDs
    def send_subscription_msg(self, mid):
        msg_size = 56  # Base size of message with a single subscription
        msg_type = 1
        sbn_sub_qos_priority = 0
        sbn_sub_qos_reliability = 0

        # Are we sending one subscription, or multiple?
        if type(mid) is list:
            # mid is an array of dictionaries defining subscription details
            sbn_sub_count = len(mid)
            msg_size += (sbn_sub_count - 1) * 6
            subscription_msg = self.write_header(msg_size, msg_type)
            subscription_msg = write_bytes(self._rev_id_string, subscription_msg)

            subscription_msg = write_half_word(sbn_sub_count, subscription_msg)

            for msg in mid:
                subscription_msg = write_full_word(msg, subscription_msg)

                subscription_msg = write_bytes([sbn_sub_qos_priority, sbn_sub_qos_reliability],
                                            subscription_msg)

            
        else:
            # mid is a single ID.

            subscription_msg = self.write_header(msg_size, msg_type)
            subscription_msg = write_bytes(self._rev_id_string, subscription_msg)

            # subscription count is 1
            subscription_msg = write_half_word(1, subscription_msg)

            subscription_msg = write_full_word(mid, subscription_msg)
            subscription_msg = write_bytes([sbn_sub_qos_priority, sbn_sub_qos_reliability],
                                            subscription_msg)

        self._send(bytes(subscription_msg))

    ## When a Peer is connected send required information
    def connected(self):
        self._last_heartbeat_rx = self._node.get_clock().now()

        if not self._connected:
            self._connected = True

            if len(self._ros_subscriptions) > 0:
                self.send_subscription_msg(self._ros_subscriptions)
            else:
                self.send_protocol_msg()

            self._node.get_logger().info("Connected to Peer " + str(self._peer_spacecraft_id) + ":" + str(self._peer_processor_id))

    def send_unsubscription_msg(self, mid):
        msg_size = 56
        msg_type = 2
        sbn_sub_count = 1
        sbn_sub_qos_priority = 0
        sbn_sub_qos_reliability = 0

        subscription_msg = self.write_header(msg_size, msg_type)
        subscription_msg = write_bytes(self._rev_id_string, subscription_msg)
        subscription_msg = write_half_word(sbn_sub_count, subscription_msg)
        subscription_msg = write_full_word(mid, subscription_msg)
        subscription_msg = write_bytes([sbn_sub_qos_priority, sbn_sub_qos_reliability],
                                            subscription_msg)

        self._send(bytes(subscription_msg))

    ## Send raw bytes is Peer has requested this MID
    #  @returns Bytes sent on success, None if not subscribed
    def send_cfe_if_subscribed(self, mid, msg_bytes):
        if mid in self._subscriptions:
            return self.send_cfe_message_msg(msg_bytes)
        else:
            return None

    ## Send m message to SBN peer. This functon does NOT check subscription state.
    def send_cfe_message_msg(self, msg_bytes):
        msg_size = len(msg_bytes)
        msg_type = 3

        protocol_msg = self.write_header(msg_size, msg_type)
        protocol_msg = write_bytes(msg_bytes, protocol_msg)

        self._send(bytes(protocol_msg))

    ## Record and process subscriptions requested from remote peer (called upon receipt of peer sub msg)
    ## @param subscriptions Is an array of subscriptions, where each sub is itself a dict of mid, priority, reliability
    def add_subscriptions(self, subscriptions):
        for sub in subscriptions:

            if sub['mid'] in self._subscriptions:
                self._node.get_logger().info("Ignoring duplicate sub to " + str(sub['mid']))
                # TODO: If we utilize QOS parameters, check if those have been updated
            else:
                # (FUTURE: For conditional ROS subscription) - Lookup spec and ensure corresponding rOS subscription is activated
                # spec = SBNMessageSpec.get_by_mid(sub['mid']);
                # if spec:
                #    spec.ros_subscribe( SBNPeer, self._node );

                # And log subscription details to this peer
                self._subscriptions[sub['mid']] = sub

        return subscriptions

    ## Record and process subscription deletionss requested from remote peer (called upon receipt of peer unsub msg)
    ## @param subscriptions Is an array of subscriptions, where each sub is itself an array of MID, priority, reliability
    def del_subscriptions(self, subscriptions):
        for sub in subscriptions:
            if sub['mid'] in self._subscriptions:
                # FUTURE: To enable conditional ROS-level subscriptions
                #spec = SBNMessageSpec.get_by_mid(sub['mid']);
                #if spec:
                #    spec.ros_unsubscribe( self._node );

                del self._subscriptions[sub['mid']]
        return subscriptions
