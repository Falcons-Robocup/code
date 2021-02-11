Blockly.Python['motion_stop'] = function(block) {
  var code = 'rci.setRobotVelocity(0.0, 0.0, 0.0)\n';
  return code;
};

Blockly.Python['motion_teamplaymove'] = function(block) {
  var value_target = Blockly.Python.valueToCode(block, 'target', Blockly.Python.ORDER_ATOMIC);
  var dropdown_slow = block.getFieldValue('slow');
  var code = 'rci.setTeamplayAction("MOVE", {"target": "coord:" + str(' + value_target + '.x) + "," + str(' + value_target + '.y)})\n';
  return code;
};

Blockly.Python['motion_motionplanningmove'] = function(block) {
  var value_pose = Blockly.Python.valueToCode(block, 'pose', Blockly.Python.ORDER_ATOMIC);
  var code = 'rci.setMotionPlanningAction("MOVE", ' + value_pose + '.x, ' + value_pose + '.y, ' + value_pose + '.Rz, "NORMAL", True)\n';
  return code;
};

Blockly.Python['motion_velocitycontrolmove_pos_only'] = function(block) {
  var value_pose = Blockly.Python.valueToCode(block, 'pose', Blockly.Python.ORDER_ATOMIC);
  var code = 'rci.setRobotPosVel("POS_ONLY", ' + value_pose + '.x, ' + value_pose + '.y, ' + value_pose + '.Rz, 0.0, 0.0, 0.0, "NORMAL")\n';
  return code;
};

Blockly.Python['motion_velocitycontrolmove_posvel'] = function(block) {
  var value_pose = Blockly.Python.valueToCode(block, 'pose', Blockly.Python.ORDER_ATOMIC);
  var number_vx = block.getFieldValue('vx');
  var number_vy = block.getFieldValue('vy');
  var number_vRz = block.getFieldValue('vRz');
  var code = 'rci.setRobotPosVel("POSVEL", ' + value_pose + '.x, ' + value_pose + '.y, ' + value_pose + '.Rz, ' + number_vx + ', ' + number_vy + ', ' + number_vRz + ', "NORMAL")\n';
  return code;
};

Blockly.Python['motion_velocitycontrolmove_vel_only'] = function(block) {
  var number_vx = block.getFieldValue('vx');
  var number_vy = block.getFieldValue('vy');
  var number_vRz = block.getFieldValue('vRz');
  var number_time = block.getFieldValue('time');
  var code = 'rci.setRobotPosVel("VEL_ONLY", 0.0, 0.0, 0.0, ' + number_vx + ', ' + number_vy + ', ' + number_vRz + ', "NORMAL")\ntime.sleep(' + number_time + ')\nrci.setRobotPosVel("VEL_ONLY", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "NORMAL")\n';
  return code;
};

Blockly.Python['motion_robotvelocity'] = function(block) {
  var number_vx = block.getFieldValue('vx');
  var number_vy = block.getFieldValue('vy');
  var number_vRz = block.getFieldValue('vRz');
  var number_time = block.getFieldValue('time');
  var code = 'rci.setRobotVelocity(' + number_vx + ', ' + number_vy + ', ' + number_vRz + ')\ntime.sleep(' + number_time + ')\nrci.setRobotVelocity(0.0, 0.0, 0.0)\n';
  return code;
};

Blockly.Python['motion_motorsvelocity'] = function(block) {
  var number_vm1 = block.getFieldValue('vm1');
  var number_vm2 = block.getFieldValue('vm2');
  var number_vm3 = block.getFieldValue('vm3');
  var number_time = block.getFieldValue('time');
  var code = 'rci.setMotorsVelocity(' + number_vm1 + ', ' + number_vm2 + ', ' + number_vm3 + ')\ntime.sleep(' + number_time + ')\nrci.setMotorsVelocity(0.0, 0.0, 0.0)\n';
  return code;
};

Blockly.Python['motion_ballhandlers'] = function(block) {
  var dropdown_bhenabled = block.getFieldValue('bhEnabled');
  var code = 'rci.setBallHandlers(' + dropdown_bhenabled + ')\n';
  return code;
};

Blockly.Python['motion_keeperframe'] = function(block) {
  var dropdown_direction = block.getFieldValue('direction');
  var code = 'rci.setKeeperFrameExtend("' + dropdown_direction + '")\n';
  return code;
};

Blockly.Python['motion_blockuntilvelocitysettled'] = function(block) {
  var number_settletime = block.getFieldValue('settleTime');
  var code = 'rci.blockUntilVelocitySettled(' + number_settletime + ')\n';
  return code;
};
