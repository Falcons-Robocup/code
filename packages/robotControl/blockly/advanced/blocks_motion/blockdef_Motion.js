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

Blockly.Blocks['motion_motionplanning_action'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("[MotionPlanning] Action ")
        .appendField(new Blockly.FieldDropdown([["Move","MOVE"], ["Pass","PASS"], ["Shoot","SHOOT"], ["Lob","LOB"], ["Stop","STOP"], ["GetBall","GET_BALL"], ["TurnAwayFromOpponent","TURN_AWAY_FROM_OPPONENT"], ["KeeperMove","KEEPER_MOVE"], ["Kick","KICK"], ["InterceptBall","INTERCEPT_BALL"]]), "action");
    this.appendValueInput("pos")
        .setCheck("Pose")
        .appendField("Pos=");
    this.appendDummyInput()
        .appendField("MotionType=")
        .appendField(new Blockly.FieldDropdown([["Normal","NORMAL"], ["WithBall","WITH_BALL"], ["Accurate","ACCURATE"], ["Intercept","INTERCEPT"], ["Slow","SLOW"], ["Sprint","SPRINT"]]), "motionType")
    this.appendDummyInput()
        .appendField("BallHandlersEnabled=")
        .appendField(new Blockly.FieldDropdown([["Enabled","True"], ["Disabled","False"]]), "bhEnabled");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("motionPlanningMove( [Pose] pose )");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['motion_blockuntilpassedorfailed'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Block until MotionPlanning PASSED or FAILED");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
    this.setTooltip("Blocking until PASSED or FAILED is returned by MotionPlanning");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['motion_velocitycontrolmove_pos_only'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("[VelocityControl] Move to");
    this.appendValueInput("pose")
        .setCheck("Pose")
        .appendField("pose=");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("velocityControlMove( [Pose] pose )");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['motion_velocitycontrolmove_posvel'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("[VelocityControl] Move to");
    this.appendValueInput("pose")
        .setCheck("Pose")
        .appendField("pose=");
    this.appendDummyInput()
        .appendField("with vx=")
        .appendField(new Blockly.FieldNumber(0, -1, 1, 0.01), "vx")
        .appendField("m/s, vy=")
        .appendField(new Blockly.FieldNumber(0, -1, 1, 0.01), "vy")
        .appendField("m/s, vRz=")
        .appendField(new Blockly.FieldNumber(0, -2, 2, 0.01), "vRz")
        .appendField("rad/s at target position");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("velocityControlMove( [Pose] pose, [Int] vx, [Int] vy, [Int] vRz )");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['motion_velocitycontrolmove_vel_only'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("[VelocityControl] Move with vx=")
        .appendField(new Blockly.FieldNumber(0, -1, 1, 0.01), "vx")
        .appendField("m/s, vy=")
        .appendField(new Blockly.FieldNumber(0, -1, 1, 0.01), "vy")
        .appendField("m/s, vRz=")
        .appendField(new Blockly.FieldNumber(0, -2, 2, 0.01), "vRz")
        .appendField("rad/s for")
        .appendField(new Blockly.FieldNumber(1, 0, 5), "time")
        .appendField("second(s)");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("velocityControlMove( [Int] vx, [Int] vy, [Int] vRz )");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['motion_robotvelocity'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("[VelocityTransform] Move with vx=")
        .appendField(new Blockly.FieldNumber(0, -1, 1, 0.01), "vx")
        .appendField("m/s, vy=")
        .appendField(new Blockly.FieldNumber(0, -1, 1, 0.01), "vy")
        .appendField("m/s, vRz=")
        .appendField(new Blockly.FieldNumber(0, -2, 2, 0.01), "vRz")
        .appendField("rad/s for")
        .appendField(new Blockly.FieldNumber(1, 0, 5), "time")
        .appendField("second(s)");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("RobotVelocity( [Int] vx, [Int] vy, [Int] vRz )");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['motion_motorsvelocity'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("[PeripheralsInterface] Set Motors Velocity to m1=")
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

Blockly.Blocks['motion_blockuntilvelocitysettled'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Block until RobotVelocity settled for ")
        .appendField(new Blockly.FieldNumber(1.0, 0.1, 10.0, 0.01), "settleTime")
        .appendField("second(s)");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("BlockUntilVelocitySettled( [Float] settleTime )");
 this.setHelpUrl("");
  }
};
