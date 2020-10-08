
Blockly.Python['pos_hasreachedposition'] = function(block) {
  var value_pos = Blockly.Python.valueToCode(block, 'pos', Blockly.Python.ORDER_ATOMIC);
  var value_targetpos = Blockly.Python.valueToCode(block, 'targetPos', Blockly.Python.ORDER_ATOMIC);
  var number_xythreshold = block.getFieldValue('xyThreshold');

  // (abs(pos[x] - targetpos[x]) < xythreshold) and (abs(pos[y] - targetpos[y]) < xythreshold)
  var code = '(abs(' + value_pos + '.x - ' + value_targetpos + '.x) < ' + number_xythreshold + ') and (abs(' + value_pos + '.y - ' + value_targetpos + '.y) < ' + number_xythreshold + ')';
  
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['pos_hasreachedpose'] = function(block) {
  var value_pose = Blockly.Python.valueToCode(block, 'pose', Blockly.Python.ORDER_ATOMIC);
  var value_targetpose = Blockly.Python.valueToCode(block, 'targetPose', Blockly.Python.ORDER_ATOMIC);
  var number_xythreshold = block.getFieldValue('xyThreshold');
  var number_phithreshold = block.getFieldValue('phiThreshold');

  // (abs(pos[x] - targetpos[x]) < xythreshold) and (abs(pos[y] - targetpos[y]) < xythreshold) and (abs(pos[Rz] - targetpos[Rz]) < phiThreshold)
  var code = '(abs(' + value_pose + '.x - ' + value_targetpose + '.x) < ' + number_xythreshold + ') and (abs(' + value_pose + '.y - ' + value_targetpose + '.y) < ' + number_xythreshold + ') and (abs(' + value_pose + '.Rz - ' + value_targetpose + '.Rz) < ' + number_phithreshold + ')';
  
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['pos_getpositionfrompoi'] = function(block) {
  var dropdown_poi = block.getFieldValue('poi');
  var code = 'EnvironmentField.cEnvironmentField.getInstance().getFieldPOI( EnvironmentField.' + dropdown_poi + ' )';
  return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python['pos_getarea'] = function(block) {
  var dropdown_area = block.getFieldValue('area');
  var code = 'EnvironmentField.cEnvironmentField.getInstance().getFieldArea( EnvironmentField.' + dropdown_area + ' )';
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['pos_ispositioninarea'] = function(block) {
  var value_pos = Blockly.Python.valueToCode(block, 'pos', Blockly.Python.ORDER_ATOMIC);
  var value_area = Blockly.Python.valueToCode(block, 'area', Blockly.Python.ORDER_ATOMIC);
  var code = 'EnvironmentField.cEnvironmentField.getInstance().isPositionInArea( ' + value_pos + '.x, ' + value_pos + '.y, ' + value_area + ' )';
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['pos_createposition'] = function(block) {
  var value_x = Blockly.Python.valueToCode(block, 'x', Blockly.Python.ORDER_ATOMIC);
  var value_y = Blockly.Python.valueToCode(block, 'y', Blockly.Python.ORDER_ATOMIC);
  // TODO EKPC: No Position defined, use RobotPose for now
  var code = 'RobotPose( ' + value_x + ', ' + value_y + ' )';
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['pos_createpose'] = function(block) {
  var value_x = Blockly.Python.valueToCode(block, 'x', Blockly.Python.ORDER_ATOMIC);
  var value_y = Blockly.Python.valueToCode(block, 'y', Blockly.Python.ORDER_ATOMIC);
  var value_rz = Blockly.Python.valueToCode(block, 'Rz', Blockly.Python.ORDER_ATOMIC);
  var code = 'RobotPose( ' + value_x + ', ' + value_y + ', ' + value_rz + ' )';
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['pos_getposefromposition'] = function(block) {
  var value_pos = Blockly.Python.valueToCode(block, 'pos', Blockly.Python.ORDER_ATOMIC);
  var value_rz = Blockly.Python.valueToCode(block, 'Rz', Blockly.Python.ORDER_ATOMIC);
  var code = 'RobotPose( ' + value_pos + '.x, ' + value_pos + '.y, ' + value_rz + ' )';
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['pos_getpositionfrompose'] = function(block) {
  var value_pose = Blockly.Python.valueToCode(block, 'pose', Blockly.Python.ORDER_ATOMIC);
  // TODO EKPC: No Position defined, use RobotPose for now
  var code = 'RobotPose( ' + value_pose + '.x, ' + value_pose + '.y )';
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['pos_anglebetweenpositions'] = function(block) {
  var value_pos1 = Blockly.Python.valueToCode(block, 'pos1', Blockly.Python.ORDER_ATOMIC);
  var value_pos2 = Blockly.Python.valueToCode(block, 'pos2', Blockly.Python.ORDER_ATOMIC);
  var code = 'angle_between_two_points_0_2pi( ' + value_pos1 + '.x, ' + value_pos1 + '.y, ' + value_pos2 + '.x, ' + value_pos2 + '.y )';
  return [code, Blockly.Python.ORDER_NONE];
};
