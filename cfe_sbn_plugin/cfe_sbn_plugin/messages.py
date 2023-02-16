#!/usr/bin/env python3

import struct

### Utility functions
def read_half_word(msg):
    (hw, ) = struct.unpack(">H", msg[0:2])
    return (hw, msg[2:])

def read_full_word(msg):
    (fw, ) = struct.unpack(">L", msg[0:4])
    return (fw, msg[4:])

def read_bytes(msg, num_bytes):
    bs = msg[0:num_bytes]
    return (bs, msg[num_bytes:])

def write_half_word(hw, msg):
    bs = struct.pack(">H", hw)
    msg.extend(bs)
    return (msg)

def write_full_word(fw, msg):
    bs = struct.pack(">L", fw)
    msg.extend(bs)
    return (msg)

def write_bytes(values, msg):
    bs = []
    for v in values:
        b = struct.pack(">B", v & 255)
        bs.extend(b)
    msg.extend(bs)
    return (msg)

## SBN Message Header
class SBNMessageHdr():
    def __init__(self, size, sbntype, proc_id, sc_id, msg=None):
        ## SBN Message size
        self.size = size

        ## SBN Message Type
        self.sbn_type = sbntype

        self.proc_id = proc_id

        self.sc_id = sc_id

        ## Message Body.  May be Rw Bytes or CCSDSHdr
        self.msg = msg

    ## Stringify object for display/debug usage
    def __str__ (self):
        return 'SBNMessageHdr(size=' \
           + str(self.size) + ',sbn_type=' \
           + str(self.sbn_type) + ', proc_id=' \
           + str(self.proc_id)+', sc_id=' \
           + str(sc_id)+', msg=' \
           + msg \
           + ')';

    ## Serialize to a byte stream
    # @param bytes If true, return values as bytes, otherwise returns an array of integer (byte-like) values.
    def serialize(self, bytes=False):
        msg = []
        msg = write_half_word(self.size, msg)
        msg = write_bytes([self.sbn_type], msg)
        msg = write_full_word(self.proc_id, msg)
        msg = write_full_word(self.sc_id, msg)
        if (self.msg):
            # If self.msg is array
            msg.extend(self.msg)
            
            # TODO: Support case where msg is already bytes
            # if (isinstance(self.msg, bytes): return  TODO: How to combine bytes(msg) with self.msg in this case?
            #   Or does msg.extend also work if input is already a bytes object? Alt: + to concatenate, or .join operator?
        if (bytes):
            return bytes(msg)
        else:
            return msg

    ## Parse a byte stream into a SBNMessageHdr instance
    # @param msg Input bytes (ie: received from socket)
    # @returns SBNMessageHdr
    @classmethod
    def from_bytes(cls, msg):
        if len(msg) >= 11:
            message_size, msg = read_half_word(msg)
            message_type, msg = read_bytes(msg, 1)
            processorID, msg =  read_full_word(msg)
            spacecraftID, msg = read_full_word(msg)
            return SBNMessageHdr( message_size,
                                  int.from_bytes(message_type, byteorder='big'),
                                  processorID,
                                  spacecraftID,
                                  msg );
        else:
            return None

    ## Get SBN Message type as user-friendly string. Access type property for raw integer value.
    def get_type_name(self):
        msg_type_name = "Unknown"

        if self.type == 0:
            msg_type_name = "SBN_NO_MSG"
        elif self.type == 1:
            msg_type_name = "SBN_SUB_MSG"
        elif self.type == 2:
            msg_type_name = "SBN_UNSUB_MSG"
        elif self.type == 3:
            msg_type_name = "SBN_APP_MSG (payload is SB msg)"
        elif self.type == 4:
            msg_type_name = "SBN_PROTO_MSG (payload is SBN protocol number)"
        elif self.type == 0xA0:
            msg_type_name = "SBN_UDP_HEARTBEAT_MSG"
        elif self.type == 0xA1:
            msg_type_name = "SBN_UDP_ANNOUNCE_MSG"
        elif self.type == 0xA2:
            msg_type_name = "SBN_UDP_DISCONN_MSG"

        return msg_type_name        
        
## CCSDS Header
# TODO
# constructor accepts binary arrray or integer mid
# .seq_count
# .raw       # Raw packet that a deserialize structure came from. Not set for new packets being built (until serialize is first called).  This is a cached value and may not represent current state of class if attributes have been updated since construction or last serialization

