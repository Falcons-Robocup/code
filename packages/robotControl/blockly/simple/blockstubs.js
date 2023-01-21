Blockly.Python['field_position'] = function(block) {
  var dropdown_poi = block.getFieldValue('poi');
  var code = 'EnvironmentField.cEnvironmentField.getInstance().getFieldPOI( EnvironmentField.' + dropdown_poi + ' )';
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['field_area'] = function(block) {
  var dropdown_area = block.getFieldValue('area');
  var code = 'EnvironmentField.cEnvironmentField.getInstance().getFieldArea( EnvironmentField.' + dropdown_area + ' )';
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['position_in_area'] = function(block) {
  var value_pos = Blockly.Python.valueToCode(block, 'pos', Blockly.Python.ORDER_ATOMIC);
  var value_area = Blockly.Python.valueToCode(block, 'area', Blockly.Python.ORDER_ATOMIC);
  var code = 'EnvironmentField.cEnvironmentField.getInstance().isPositionInArea( ' + value_pos + '.x, ' + value_pos + '.y, ' + value_area + ' )';
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['ball_position'] = function(block) {
  var code = 'ws.getBallPosition()';
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['robot_has_ball'] = function(block) {
  var code = 'ws.hasBall(myRobotId)';
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['robot_sees_ball'] = function(block) {
  var code = '(ws.getBallPosition() != None)';
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['get_ball'] = function(block) {
  var code = 'rci.setMotionPlanningAction("GET_BALL", 0.0, 0.0, 0.0, "NORMAL", True)\nrci.blockUntilMPPassedOrFailed()\n';
  return code;
};

Blockly.Python['intercept_ball'] = function(block) {
  var code = 'rci.setMotionPlanningAction("INTERCEPT_BALL", 0.0, 0.0, 0.0, "NORMAL", True)\nrci.blockUntilMPPassedOrFailed()\n';
  return code;
};

Blockly.Python['robot_position'] = function(block) {
  var dropdown_name = block.getFieldValue('NAME');
  var code = 'ws.getRobotPosition(' + dropdown_name + ')';
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['move'] = function(block) {
  var value_pos = Blockly.Python.valueToCode(block, 'pos', Blockly.Python.ORDER_ATOMIC);
  var code = 'rci.setMotionPlanningAction("MOVE", ' + value_pos + '.x, ' + value_pos + '.y, 0.0, "NORMAL", True)\nrci.blockUntilMPPassedOrFailed()\n';
  return code;
};

Blockly.Python['shoot'] = function(block) {
  var dropdown_shoottype = block.getFieldValue('shootType');
  var value_pos = Blockly.Python.valueToCode(block, 'pos', Blockly.Python.ORDER_ATOMIC);
  // Guard with 'hasBall' to ensure we only shoot when we have the ball
  var code = 'if ws.hasBall(myRobotId): rci.setMotionPlanningAction("' + dropdown_shoottype + '", ' + value_pos + '.x, ' + value_pos + '.y, 0.0, "NORMAL", True)\nif ws.hasBall(myRobotId): rci.blockUntilMPPassedOrFailed()\n';
  return code;
};

Blockly.Python['kick'] = function(block) {
  var number_power = block.getFieldValue('power');
  var number_height = block.getFieldValue('height');
  // NOTE: no 'hasBall' guard! (use with caution) 
  var code = 'rci.setMotionPlanningAction("KICK", ' + number_power + ', ' + number_height + ', 0.0, "NORMAL", True)\nrci.blockUntilMPPassedOrFailed()\n';
  return code;
};

Blockly.Python['movexyrz'] = function(block) {
  var number_x = block.getFieldValue('x');
  var number_y = block.getFieldValue('y');
  var angle_rz = block.getFieldValue('Rz');

  var code = 'rci.setMotionPlanningAction("MOVE", ' + number_x + ', ' + number_y + ', ' + parseFloat(angle_rz) * (Math.PI/180.0) + ', "NORMAL", True)\nrci.blockUntilMPPassedOrFailed()\n';
  return code;
};

Blockly.Python['sleep'] = function(block) {
  var number_duration = block.getFieldValue('duration');
  var code = 'rci.sleep(' + number_duration + ')\n';
  return code;
};

