Blockly.Python['wm_getobstaclepositions'] = function(block) {
  var code = 'ws.getObstacles()';
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['wm_getobstaclevelocities'] = function(block) {
  var code = 'ws.getObstacleVelocities()';
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['wm_getnearestobstacletoposition'] = function(block) {
  var value_pos = Blockly.Python.valueToCode(block, 'pos', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = '...';
  // TODO: Change ORDER_NONE to the correct strength.
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['wm_getballposition'] = function(block) {
  var code = 'ws.getBallPosition()';
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['wm_getballvelocity'] = function(block) {
  var code = 'ws.getBallVelocity()';
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['wm_teamhasball'] = function(block) {
  var code = '(ws.ballPossession() == sharedTypes.ballPossessionTypeEnum.TEAM.value)';
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['wm_opponenthasball'] = function(block) {
  var code = '(ws.ballPossession() == sharedTypes.ballPossessionTypeEnum.OPPONENT.value)';
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['wm_robothasball'] = function(block) {
  var value_robotid = Blockly.Python.valueToCode(block, 'robotId', Blockly.Python.ORDER_ATOMIC);
  var code = 'ws.hasBall(' + value_robotid + ')';
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['wm_seeball'] = function(block) {
  var code = '(ws.getBallPosition() != None)';
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['wm_myrobotid'] = function(block) {
  var code = 'ws.getRobotId()';
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['wm_getrobotpose'] = function(block) {
  var value_robotid = Blockly.Python.valueToCode(block, 'robotId', Blockly.Python.ORDER_ATOMIC);
  var code = 'ws.getRobotPosition(' + value_robotid + ')';
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['wm_getrobotvelocity'] = function(block) {
  var value_robotid = Blockly.Python.valueToCode(block, 'robotId', Blockly.Python.ORDER_ATOMIC);
  var code = 'ws.getRobotVelocity(' + value_robotid + ')';
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['wm_getactiverobots'] = function(block) {
  var code = 'ws.activeRobots()';
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['wm_getteammembers'] = function(block) {
  var code = '[r for r in ws.activeRobots() if r != ws.getRobotId()]';
  return [code, Blockly.Python.ORDER_NONE];
};
