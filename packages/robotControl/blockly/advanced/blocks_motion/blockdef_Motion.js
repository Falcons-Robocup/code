Blockly.Blocks['motion_stop'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Stop");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("Stop");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['motion_movetoposition'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Move to");
    this.appendValueInput("target")
        .setCheck("Position")
        .appendField("pos=");
    this.appendDummyInput()
        .appendField("at")
        .appendField(new Blockly.FieldDropdown([["slow","True"], ["normal","False"]]), "slow")
        .appendField("speed");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("moveToPosition ( [Position] pos, [Boolean] slow ) NOTE: Will always face ball");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['motion_movetopose'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("moveToPose");
    this.appendValueInput("pose")
        .setCheck("Pose")
        .appendField("pose=");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("moveToPose( [Pose] pose )");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['motion_robotvelocity'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Move with vx=")
        .appendField(new Blockly.FieldNumber(0, -1, 1, 0.01), "vx")
        .appendField("m/s, vy=")
        .appendField(new Blockly.FieldNumber(0, -1, 1, 0.01), "vy")
        .appendField("m/s, vphi=")
        .appendField(new Blockly.FieldNumber(0, -2, 2, 0.01), "vphi")
        .appendField("rad/s for")
        .appendField(new Blockly.FieldNumber(1, 0, 5), "time")
        .appendField("second(s)");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("RobotVelocity( [Int] vx, [Int] vy, [Int] vphi )");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['motion_motorsvelocity'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Set Motors Velocity to m1=")
        .appendField(new Blockly.FieldNumber(0, -1, 1, 0.01), "vm1")
        .appendField("m/s, m2=")
        .appendField(new Blockly.FieldNumber(0, -1, 1, 0.01), "vm2")
        .appendField("m/s, m3=")
        .appendField(new Blockly.FieldNumber(0, -1, 1, 0.01), "vm3")
        .appendField("m/s for")
        .appendField(new Blockly.FieldNumber(1, 0, 5), "time")
        .appendField("second(s)");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("MotorsVelocity( [Int] left, [Int] right, [Int] rear )");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['motion_ballhandlers'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Set BallHandlers to")
        .appendField(new Blockly.FieldDropdown([["enabled","True"], ["disabled","False"]]), "bhEnabled");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("BallHandlers( [enabled/disabled] )");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['motion_keeperframe'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Extend KeeperFrame")
        .appendField(new Blockly.FieldDropdown([["left","LEFT"], ["right","RIGHT"], ["up","UP"]]), "direction");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("KeeperFrame( [left/right/up] )");
 this.setHelpUrl("");
  }
};

