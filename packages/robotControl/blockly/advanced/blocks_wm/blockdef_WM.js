Blockly.Blocks['wm_getobstaclepositions'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("getObstaclePositions");
    this.setOutput(true, "Array");
    this.setColour(230);
 this.setTooltip("[List] getObstaclePositions()");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['wm_getobstaclevelocities'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("getObstacleVelocities");
    this.setOutput(true, "Array");
    this.setColour(230);
 this.setTooltip("[List] getObstacleVelocities()");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['wm_getballposition'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("getBallPosition");
    this.setOutput(true, "Position");
    this.setColour(230);
 this.setTooltip("[Position] getBallPosition()");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['wm_getballvelocity'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("getBallVelocity");
    this.setOutput(true, "Velocity");
    this.setColour(230);
 this.setTooltip("[Velocity] getBallVelocity()");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['wm_teamhasball'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("teamHasBall");
    this.setOutput(true, "Boolean");
    this.setColour(230);
 this.setTooltip("[Boolean] teamHasBall()");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['wm_opponenthasball'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("opponentHasBall");
    this.setOutput(true, "Boolean");
    this.setColour(230);
 this.setTooltip("[Boolean] opponentHasBall()");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['wm_robothasball'] = {
  init: function() {
    this.appendValueInput("robotId")
        .setCheck("Number")
        .appendField("robotHasBall");
    this.setOutput(true, "Boolean");
    this.setColour(230);
 this.setTooltip("[Boolean] robotHasBall( [Int] robotId )");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['wm_seeball'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("seeBall");
    this.setOutput(true, "Boolean");
    this.setColour(230);
 this.setTooltip("[Boolean] seeBall()");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['wm_myrobotid'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("myRobotId");
    this.setOutput(true, "Number");
    this.setColour(230);
 this.setTooltip("[Int] myRobotId()");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['wm_getrobotpose'] = {
  init: function() {
    this.appendValueInput("robotId")
        .setCheck("Number")
        .appendField("getRobotPose");
    this.setOutput(true, "Pose");
    this.setColour(230);
 this.setTooltip("[Position] getRobotPose( [Int] robotId )");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['wm_getrobotvelocity'] = {
  init: function() {
    this.appendValueInput("robotId")
        .setCheck("Number")
        .appendField("getRobotVelocity");
    this.setOutput(true, "Velocity");
    this.setColour(230);
 this.setTooltip("[Velocity] getRobotVelocity( [Int] robotId )");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['wm_getactiverobots'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("getActiveRobots");
    this.setOutput(true, "Array");
    this.setColour(230);
 this.setTooltip("[List] getActiveRobots()");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['wm_getteammembers'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("getTeamMembers");
    this.setOutput(true, "Array");
    this.setColour(230);
 this.setTooltip("[List] getTeamMembers()");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['wm_getnearestobstacletoposition'] = {
  init: function() {
    this.appendValueInput("pos")
        .setCheck("Position")
        .appendField("getNearestObstacleToPosition");
    this.setOutput(true, "Obstacle");
    this.setColour(230);
 this.setTooltip("[Obstacle] getNearestObstacleToPosition( [Position] pos )");
 this.setHelpUrl("");
  }
};
