-- Definitions for CCSDS
brash_ccsds = Proto("BRASH_CCSDS", "CCSDS Headers")
ccsds_stream_id = ProtoField.uint16("ccsds.pri.streamid", "ccsdsPriStreamId", base.HEX)
ccsds_seq_cnt = ProtoField.uint16("ccsds.pri.seqcnt", "ccsdsSeqCnt", base.DEC)
ccsds_length = ProtoField.uint16("ccsds.pri.length", "ccsdsLen", base.DEC)
ccsds_time_sec = ProtoField.uint32("ccsds.sec.timeSec", "ccsdsSecTimeSec", base.DEC)
ccsds_time_subsec = ProtoField.uint16("ccsds.sec.timeSubsec", "ccsdsSecTimeSubsec", base.DEC)
ccsds_spare = ProtoField.uint32("ccsds.sec.spare", "ccsdsSecSpare", base.DEC)

brash_ccsds.fields = { 
   ccsds_stream_id,
   ccsds_seq_cnt,
   ccsds_length,
   ccsds_time_sec,
   ccsds_time_subsec,
   ccsds_spare
}

function brash_ccsds.dissector(buffer, pinfo, tree)
   length = buffer:len()
   if length == 0 then return end
 
   pinfo.cols.protocol = brash_ccsds.name

   local subtree = tree:add(brash_ccsds, buffer(), "CCSDS Headers")
   subtree:add(ccsds_stream_id, buffer(0, 2))
   subtree:add(ccsds_seq_cnt, buffer(2, 2))
   subtree:add(ccsds_length, buffer(4, 2))
   subtree:add(ccsds_time_sec, buffer(6, 4))
   subtree:add(ccsds_time_subsec, buffer(10, 2))
   subtree:add(ccsds_spare, buffer(12, 4))
end