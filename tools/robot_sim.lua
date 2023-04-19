-- Definitions for the High Rate Robot Sim State

robot_sim = Proto("ROBOTSIM",  "Robot Sim State")

robot_sim_joint0 = ProtoField.float("robot_sim.joint0")
robot_sim_joint1 = ProtoField.float("robot_sim.joint1")
robot_sim_joint2 = ProtoField.float("robot_sim.joint2")
robot_sim_joint3 = ProtoField.float("robot_sim.joint3")
robot_sim_joint4 = ProtoField.float("robot_sim.joint4")
robot_sim_joint5 = ProtoField.float("robot_sim.joint5")
robot_sim_joint6 = ProtoField.float("robot_sim.joint6")
robot_sim_kp = ProtoField.float("robot_sim.kp")
robot_sim_errors0 = ProtoField.float("robot_sim.errors0")
robot_sim_errors1 = ProtoField.float("robot_sim.errors1")
robot_sim_errors2 = ProtoField.float("robot_sim.errors2")
robot_sim_errors3 = ProtoField.float("robot_sim.errors3")
robot_sim_errors4 = ProtoField.float("robot_sim.errors4")
robot_sim_errors5 = ProtoField.float("robot_sim.errors5")
robot_sim_errors6 = ProtoField.float("robot_sim.errors6")

robot_sim.fields = { 
   robot_sim_joint0,
   robot_sim_joint1,
   robot_sim_joint2,
   robot_sim_joint3,
   robot_sim_joint4,
   robot_sim_joint5,
   robot_sim_joint6,
   robot_sim_kp,
   robot_sim_errors0,
   robot_sim_errors1,
   robot_sim_errors2,
   robot_sim_errors3,
   robot_sim_errors4,
   robot_sim_errors5,
   robot_sim_errors6
}

function robot_sim.dissector(buffer, pinfo, tree)
   length = buffer:len()
   if length == 0 then return end
 
   pinfo.cols.protocol = robot_sim.name

   local subtree = tree:add(robot_sim, buffer(), "Robot Sim State")
   subtree:add_le(robot_sim_joint0, buffer(0, 4))
   subtree:add_le(robot_sim_joint1, buffer(4, 4))
   subtree:add_le(robot_sim_joint2, buffer(8, 4))
   subtree:add_le(robot_sim_joint3, buffer(12, 4))
   subtree:add_le(robot_sim_joint4, buffer(16, 4))
   subtree:add_le(robot_sim_joint5, buffer(20, 4))
   subtree:add_le(robot_sim_joint6, buffer(24, 4))
   subtree:add_le(robot_sim_kp, buffer(28, 4))
   subtree:add_le(robot_sim_errors0, buffer(32, 4))
   subtree:add_le(robot_sim_errors1, buffer(36, 4))
   subtree:add_le(robot_sim_errors2, buffer(40, 4))
   subtree:add_le(robot_sim_errors3, buffer(44, 4))
   subtree:add_le(robot_sim_errors4, buffer(48, 4))
   subtree:add_le(robot_sim_errors5, buffer(52, 4))
   subtree:add_le(robot_sim_errors6, buffer(56, 4))
end
