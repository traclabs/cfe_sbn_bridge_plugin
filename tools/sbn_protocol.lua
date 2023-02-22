sbn_protocol = Proto("SBN",  "SBN Protocol")

message_size = ProtoField.int16("sbn.message_size", "messageSize", base.DEC)
message_type = ProtoField.uint8("sbn.message_type", "messageType", base.DEC)
processor_id = ProtoField.uint32("sbn.processor_id", "processorID", base.HEX)
spacecraft_id = ProtoField.uint32("sbn.spacecraft_id", "spacecraftID", base.HEX)

-- SBN Subscription data
sbn_git_id = ProtoField.string("sbn.sbn_git_id", "sbnGitID", base.ASCII)
sbn_sub_cnt = ProtoField.uint16("sbn.sbn_sub_cnt", "sbnSubCount", base.DEC)
sbn_sub_msg_id = ProtoField.uint32("sbn.sbn_sub_msg_id", "sbnSubMsgID", base.HEX)
sbn_sub_qos_priority = ProtoField.uint8("sbn.sub_qos_priority", "sbnQOSPriority", base.DEC)
sbn_sub_qos_reliability = ProtoField.uint8("sbn.sub_qos_reliability", "sbnQOSReliability", base.DEC)

-- SBN App Message data
sbn_cfe_msg = ProtoField.none  ("sbn.cfe_message_payload", "sbnCFEMessagePayload", base.HEX)

-- SBN protocol message data
sbn_protocol_number = ProtoField.uint8("sbn.sbn_protocol_number", "sbnProtocolNumber", base.DEC)

sbn_protocol.fields = { message_size, message_type, processor_id, spacecraft_id, -- Header
                        sbn_git_id, sbn_sub_cnt, sbn_sub_msg_id, sbn_sub_qos_priority, sbn_sub_qos_reliability, -- SBN_SUB_MSG 
                        sbn_cfe_msg, -- SBN App Message 
                        sbn_protocol_number -- SBN protocol message 
                     }

function get_msg_type_name(msg_type)
   local msg_type_name = "Unknown"

   if msg_type == 0 then msg_type_name = "SBN_NO_MSG"
   elseif msg_type == 1 then msg_type_name = "SBN_SUB_MSG"
   elseif msg_type == 2 then msg_type_name = "SBN_UNSUB_MSG"
   elseif msg_type == 3 then msg_type_name = "SBN_APP_MSG (payload is SB msg)"
   elseif msg_type == 4 then msg_type_name = "SBN_PROTO_MSG (payload is SBN protocol number)"
   elseif msg_type == 0xA0 then msg_type_name = "SBN_UDP_HEARTBEAT_MSG"
   elseif msg_type == 0xA1 then msg_type_name = "SBN_UDP_ANNOUNCE_MSG"
   elseif msg_type == 0xA2 then msg_type_name = "SBN_UDP_DISCONN_MSG"
   end

   return msg_type_name
end

function sbn_protocol.dissector(buffer, pinfo, tree)
  length = buffer:len()
  if length == 0 then return end

  pinfo.cols.protocol = sbn_protocol.name

  -- Header
  local subtree = tree:add(sbn_protocol, buffer(), "SBN Protocol Data")
  local msg_size = buffer(0,2):int()
  subtree:add(message_size, buffer(0,2))
  local msg_type_number =   buffer(2,1):uint()
  local msg_type_name = get_msg_type_name(msg_type_number)
  subtree:add(message_type, buffer(2,1)):append_text(" (" .. msg_type_name .. ")")
  subtree:add(processor_id, buffer(3,4))
  subtree:add(spacecraft_id, buffer(7,4))

   -- Payload for Subscription
   if msg_type_number == 1 then
      subtree:add(sbn_git_id,   buffer(11,48))
      subtree:add(sbn_sub_cnt, buffer(59, 2))
      local subscription_count = buffer(59,2):uint()
      for i = 1,subscription_count,1
      do
         subtree:add(sbn_sub_msg_id, buffer(61+(i-1)*6, 4))
         subtree:add(sbn_sub_qos_priority, buffer(65+(i-1)*6,1))
         subtree:add(sbn_sub_qos_reliability, buffer(66+(i-1)*6,1))   
      end
   end

   -- Payload for Unsubscription
   if msg_type_number == 2 then
      subtree:add(sbn_git_id,   buffer(11,48))
      subtree:add(sbn_sub_cnt, buffer(59, 2))
      local subscription_count = buffer(59,2):uint()
      for i = 1,subscription_count,1
      do
         subtree:add(sbn_sub_msg_id, buffer(61+(i-1)*6, 4))
         subtree:add(sbn_sub_qos_priority, buffer(65+(i-1)*6,1))
         subtree:add(sbn_sub_qos_reliability, buffer(66+(i-1)*6,1))   
      end
   end

   if msg_type_number == 3 then
      subtree:add(sbn_cfe_msg, buffer(11, msg_size))
   end

   if msg_type_number == 4 then
      subtree:add(sbn_protocol_number, buffer(11, 1))
   end

end

local udp_port = DissectorTable.get("udp.port")
udp_port:add(2234, sbn_protocol)
udp_port:add(2235, sbn_protocol)
udp_port:add(2236, sbn_protocol)