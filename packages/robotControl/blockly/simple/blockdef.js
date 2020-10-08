Blockly.Blocks['field_position'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("field position")
        .appendField(new Blockly.FieldDropdown([["Field Center","P_CENTER"], ["Field Center Left","P_CENTER_LEFT"], ["Field Center Right","P_CENTER_RIGHT"], ["Field Circle Left","P_CIRCLE_INTERSECT_LINE_LEFT"], ["Field Circle Right","P_CIRCLE_INTERSECT_LINE_RIGHT"], ["Own Half Safety Border Corner Left","P_OWN_SB_CORNER_LEFT"], ["Own Half Safety Border Corner Right","P_OWN_SB_CORNER_RIGHT"], ["Opponent Half Safety Border Corner Left","P_OPP_SB_CORNER_LEFT"], ["Opponent Half Safety Border Corner Right","P_OPP_SB_CORNER_RIGHT"], ["Own Half Corner Left","P_OWN_CORNER_LEFT"], ["Own Half Corner Right","P_OWN_CORNER_RIGHT"], ["Opponent Half Corner Left","P_OPP_CORNER_LEFT"], ["Opponent Half Corner Right","P_OPP_CORNER_RIGHT"], ["Own Half Penalty Spot","P_OWN_PENALTY_SPOT"], ["Opponent Half Penalty Spot","P_OPP_PENALTY_SPOT"], ["Own Goalpost Left","P_OWN_GOALPOST_LEFT"], ["Own Goalpost Right","P_OWN_GOALPOST_RIGHT"], ["Own Goal Backside Left","P_OWN_GOALPOST_LEFTBACK"], ["Own Goal Backside Right","P_OWN_GOALPOST_RIGHTBACK"], ["Own Goalline Center","P_OWN_GOALLINE_CENTER"], ["Opponent Goalpost Left","P_OPP_GOALPOST_LEFT"], ["Opponent Goalpost Right","P_OPP_GOALPOST_RIGHT"], ["Opponent Goal Backside Left","P_OPP_GOALPOST_LEFTBACK"], ["Opponent Goal Backside Right","P_OPP_GOALPOST_RIGHTBACK"], ["Opponent Goalline Center","P_OPP_GOALLINE_CENTER"], ["Own Half, Sideline Left, Middle","P_OWN_MID_LEFT"], ["Own Half, Sideline Right, Middle","P_OWN_MID_RIGHT"], ["Own Half, Center","P_OWN_MID_CENTER"], ["Own Half, Goal Area, Inner Left Corner","P_OWN_GOALAREA_CORNER_LEFT"], ["Own Half, Goal Area, Inner Right Corner","P_OWN_GOALAREA_CORNER_RIGHT"], ["Own Half, Goal Area, Outer Left Corner","P_OWN_GOALAREA_GOALLINE_LEFT"], ["Own Half, Goal Area, Outer Right Corner","P_OWN_GOALAREA_GOALLINE_RIGHT"], ["Own Half, Penalty Area, Inner Left Corner","P_OWN_PENALTYAREA_CORNER_LEFT"], ["Own Half, Penalty Area, Inner Right Corner","P_OWN_PENALTYAREA_CORNER_RIGHT"], ["Own Half, Penalty Area, Outer Left Corner","P_OWN_PENALTYAREA_GOALLINE_LEFT"], ["Own Half, Penalty Area, Outer Right Corner","P_OWN_PENALTYAREA_GOALLINE_RIGHT"], ["Opponent Half, Sideline Left, Middle","P_OPP_MID_LEFT"], ["Opponent Half, Sideline Right, Middle","P_OPP_MID_RIGHT"], ["Opponent Half, Center","P_OPP_MID_CENTER"], ["Opponent Half, Goal Area, Inner Left Corner","P_OPP_GOALAREA_CORNER_LEFT"], ["Opponent Half, Goal Area, Inner Right Corner","P_OPP_GOALAREA_CORNER_RIGHT"], ["Opponent Half, Goal Area, Outer Left Corner","P_OPP_GOALAREA_GOALLINE_LEFT"], ["Opponent Half, Goal Area, Outer Right Corner","P_OPP_GOALAREA_GOALLINE_RIGHT"], ["Opponent Half, Penalty Area, Inner Left Corner","P_OPP_PENALTYAREA_CORNER_LEFT"], ["Opponent Half, Penalty Area, Inner Right Corner","P_OPP_PENALTYAREA_CORNER_RIGHT"], ["Opponent Half, Penalty Area, Outer Left Corner","P_OPP_PENALTYAREA_GOALLINE_LEFT"], ["Opponent Half, Penalty Area, Outer Right Corner","P_OPP_PENALTYAREA_GOALLINE_RIGHT"], ["Tip-in Position","P_TIP_IN"]]), "poi");
    this.setOutput(true, "Position");
    this.setColour(0);
 this.setTooltip("returns a Position on the field");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['field_area'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("field area")
        .appendField(new Blockly.FieldDropdown([["Field","A_FIELD"], ["Left Half Field","A_FIELD_LEFT"], ["Right Half Field","A_FIELD_RIGHT"], ["Center Circle","A_CENTER_CIRCLE"], ["Field Safety Borders","A_FIELD_SAFETY_BOUNDARIES"], ["Own Half","A_OWN_SIDE"], ["Opponent Half","A_OPP_SIDE"], ["Own Half, Left Quarter","A_OWN_LEFT_SIDE"], ["Own Half, Right Quarter","A_OWN_RIGHT_SIDE"], ["Opponent Half, Left Quarter","A_OPP_LEFT_SIDE"], ["Opponent Half, Right Quarter","A_OPP_RIGHT_SIDE"], ["Own GoalArea","A_OWN_GOALAREA"], ["Own Goal","A_OWN_GOAL"], ["Own PenaltyArea","A_OWN_PENALTYAREA"], ["Opponent GoalArea","A_OPP_GOALAREA"], ["Opponent Goal","A_OPP_GOAL"], ["Opponent PenaltyArea","A_OPP_PENALTYAREA"]]), "area");
    this.setOutput(true, "Area");
    this.setColour(0);
 this.setTooltip("returns an Area on the field");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['position_in_area'] = {
  init: function() {
    this.appendValueInput("pos")
        .setCheck("Position")
        .appendField("position");
    this.appendValueInput("area")
        .setCheck("Area")
        .appendField("is in area");
    this.setInputsInline(true);
    this.setOutput(true, "Boolean");
    this.setColour(230);
 this.setTooltip("returns whether the Position is in the Area");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['ball_position'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("ball position");
    this.setOutput(true, "Position");
    this.setColour(0);
 this.setTooltip("returns the position of the Ball on the field");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['robot_has_ball'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("robot has ball");
    this.setOutput(true, "Boolean");
    this.setColour(230);
 this.setTooltip("returns whether the Robot has the Ball in its possession");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['robot_sees_ball'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("robot sees ball");
    this.setOutput(true, "Boolean");
    this.setColour(230);
 this.setTooltip("returns whether the Robot sees the Ball on the field");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['get_ball'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("robot: get ball");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("instructs the Robot to get the Ball");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['intercept_ball'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("robot: intercept ball");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("instructs the Robot to intercept the Ball");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['robot_position'] = {
  init: function() {
    this.appendDummyInput()
        .appendField(new Blockly.FieldDropdown([["my","myRobotId"], ["r1","1"], ["r2","2"], ["r3","3"], ["r4","4"], ["r5","5"], ["r6","6"], ["r7","7"]]), "NAME")
        .appendField("robot position");
    this.setOutput(true, "Position");
    this.setColour(0);
 this.setTooltip("returns the position of the Robot on the field");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['move'] = {
  init: function() {
    this.appendValueInput("pos")
        .setCheck("Position")
        .appendField("robot: move to position");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("instructs the Robot to move to the position on the field");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['shoot'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("robot:")
        .appendField(new Blockly.FieldDropdown([["shoot","SHOOT"], ["pass","PASS"], ["lob","LOB"]]), "shootType");
    this.appendValueInput("pos")
        .setCheck("Position")
        .appendField("towards position");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("instructs the Robot to shoot/pass/lob towards a position on the field");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['kick'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("robot: kick with power")
        .appendField(new Blockly.FieldNumber(20, 0, 180, 0.1), "power")
        .appendField("and height")
        .appendField(new Blockly.FieldNumber(0, 0, 180, 0.1), "height");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("instructs the Robot to kick");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['movexyrz'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("robot: move to x=")
        .appendField(new Blockly.FieldNumber(0, -6, 6, 0.01), "x")
        .appendField(", y=")
        .appendField(new Blockly.FieldNumber(0, -9, 9, 0.01), "y")
        .appendField(", Rz=")
        .appendField(new Blockly.FieldAngle(90), "Rz");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("instructs the Robot to move to the (x,y) position on the field with orientation Rz");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['sleep'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("sleep: duration=")
        .appendField(new Blockly.FieldNumber(1, 0, 999, 0.01), "duration")
        .appendField("seconds");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(65);
 this.setTooltip("will make the Robot do nothing for a while");
 this.setHelpUrl("");
  }
};

