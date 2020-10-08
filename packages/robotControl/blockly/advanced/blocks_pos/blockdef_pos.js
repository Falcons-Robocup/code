Blockly.Blocks['pos_hasreachedposition'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("hasReachedPosition");
    this.appendValueInput("pos")
        .setCheck("Position")
        .appendField("pos=");
    this.appendValueInput("targetPos")
        .setCheck("Position")
        .appendField("targetPos=");
    this.appendDummyInput()
        .appendField("( xyThreshold=")
        .appendField(new Blockly.FieldNumber(0.03, 0, 1, 0.01), "xyThreshold")
        .appendField("m )");
    this.setInputsInline(true);
    this.setOutput(true, "Boolean");
    this.setColour(230);
 this.setTooltip("[Boolean] hasReachedPosition( [Position] pos, [Position] targetPos, [Float] xyThreshold )");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['pos_hasreachedpose'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("hasReachedPose");
    this.appendValueInput("pose")
        .setCheck("Pose")
        .appendField("pose=");
    this.appendValueInput("targetPose")
        .setCheck("Pose")
        .appendField("targetPose=");
    this.appendDummyInput()
        .appendField("( xyThreshold=")
        .appendField(new Blockly.FieldNumber(0.03, 0, 1, 0.01), "xyThreshold")
        .appendField("m, ");
    this.appendDummyInput()
        .appendField("RzThreshold=")
        .appendField(new Blockly.FieldNumber(0.01, 0, 1, 0.01), "phiThreshold")
        .appendField("m )");
    this.setInputsInline(true);
    this.setOutput(true, "Boolean");
    this.setColour(230);
 this.setTooltip("[Boolean] hasReachedPose( [Pose] pose, [Pose] targetPose, [Float] xyThreshold, [Float] phiThreshold )");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['pos_getpositionfrompoi'] = {
  init: function() {
    this.appendDummyInput("poi")
        .appendField("getPositionFromPOI")
        .appendField(new Blockly.FieldDropdown([["Field Center","P_CENTER"], ["Field Center Left","P_CENTER_LEFT"], ["Field Center Right","P_CENTER_RIGHT"], ["Field Circle Left","P_CIRCLE_INTERSECT_LINE_LEFT"], ["Field Circle Right","P_CIRCLE_INTERSECT_LINE_RIGHT"], ["Own Half Safety Border Corner Left","P_OWN_SB_CORNER_LEFT"], ["Own Half Safety Border Corner Right","P_OWN_SB_CORNER_RIGHT"], ["Opponent Half Safety Border Corner Left","P_OPP_SB_CORNER_LEFT"], ["Opponent Half Safety Border Corner Right","P_OPP_SB_CORNER_RIGHT"], ["Own Half Corner Left","P_OWN_CORNER_LEFT"], ["Own Half Corner Right","P_OWN_CORNER_RIGHT"], ["Opponent Half Corner Left","P_OPP_CORNER_LEFT"], ["Opponent Half Corner Right","P_OPP_CORNER_RIGHT"], ["Own Half Penalty Spot","P_OWN_PENALTY_SPOT"], ["Opponent Half Penalty Spot","P_OPP_PENALTY_SPOT"], ["Own Goalpost Left","P_OWN_GOALPOST_LEFT"], ["Own Goalpost Right","P_OWN_GOALPOST_RIGHT"], ["Own Goal Backside Left","P_OWN_GOALPOST_LEFTBACK"], ["Own Goal Backside Right","P_OWN_GOALPOST_RIGHTBACK"], ["Own Goalline Center","P_OWN_GOALLINE_CENTER"], ["Opponent Goalpost Left","P_OPP_GOALPOST_LEFT"], ["Opponent Goalpost Right","P_OPP_GOALPOST_RIGHT"], ["Opponent Goal Backside Left","P_OPP_GOALPOST_LEFTBACK"], ["Opponent Goal Backside Right","P_OPP_GOALPOST_RIGHTBACK"], ["Opponent Goalline Center","P_OPP_GOALLINE_CENTER"], ["Own Half, Sideline Left, Middle","P_OWN_MID_LEFT"], ["Own Half, Sideline Right, Middle","P_OWN_MID_RIGHT"], ["Own Half, Center","P_OWN_MID_CENTER"], ["Own Half, Goal Area, Inner Left Corner","P_OWN_GOALAREA_CORNER_LEFT"], ["Own Half, Goal Area, Inner Right Corner","P_OWN_GOALAREA_CORNER_RIGHT"], ["Own Half, Goal Area, Outer Left Corner","P_OWN_GOALAREA_GOALLINE_LEFT"], ["Own Half, Goal Area, Outer Right Corner","P_OWN_GOALAREA_GOALLINE_RIGHT"], ["Own Half, Penalty Area, Inner Left Corner","P_OWN_PENALTYAREA_CORNER_LEFT"], ["Own Half, Penalty Area, Inner Right Corner","P_OWN_PENALTYAREA_CORNER_RIGHT"], ["Own Half, Penalty Area, Outer Left Corner","P_OWN_PENALTYAREA_GOALLINE_LEFT"], ["Own Half, Penalty Area, Outer Right Corner","P_OWN_PENALTYAREA_GOALLINE_RIGHT"], ["Opponent Half, Sideline Left, Middle","P_OPP_MID_LEFT"], ["Opponent Half, Sideline Right, Middle","P_OPP_MID_RIGHT"], ["Opponent Half, Center","P_OPP_MID_CENTER"], ["Opponent Half, Goal Area, Inner Left Corner","P_OPP_GOALAREA_CORNER_LEFT"], ["Opponent Half, Goal Area, Inner Right Corner","P_OPP_GOALAREA_CORNER_RIGHT"], ["Opponent Half, Goal Area, Outer Left Corner","P_OPP_GOALAREA_GOALLINE_LEFT"], ["Opponent Half, Goal Area, Outer Right Corner","P_OPP_GOALAREA_GOALLINE_RIGHT"], ["Opponent Half, Penalty Area, Inner Left Corner","P_OPP_PENALTYAREA_CORNER_LEFT"], ["Opponent Half, Penalty Area, Inner Right Corner","P_OPP_PENALTYAREA_CORNER_RIGHT"], ["Opponent Half, Penalty Area, Outer Left Corner","P_OPP_PENALTYAREA_GOALLINE_LEFT"], ["Opponent Half, Penalty Area, Outer Right Corner","P_OPP_PENALTYAREA_GOALLINE_RIGHT"], ["Tip-in Position","P_TIP_IN"]]), "poi");
    this.setOutput(true, "Position");
    this.setColour(230);
 this.setTooltip("[Position] getPositionFromPOI( [POI] poi )");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['pos_getarea'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("getArea")
        .appendField(new Blockly.FieldDropdown([["Field","A_FIELD"], ["Left Half Field","A_FIELD_LEFT"], ["Right Half Field","A_FIELD_RIGHT"], ["Center Circle","A_CENTER_CIRCLE"], ["Field Safety Borders","A_FIELD_SAFETY_BOUNDARIES"], ["Own Half","A_OWN_SIDE"], ["Opponent Half","A_OPP_SIDE"], ["Own Half, Left Quarter","A_OWN_LEFT_SIDE"], ["Own Half, Right Quarter","A_OWN_RIGHT_SIDE"], ["Opponent Half, Left Quarter","A_OPP_LEFT_SIDE"], ["Opponent Half, Right Quarter","A_OPP_RIGHT_SIDE"], ["Own GoalArea","A_OWN_GOALAREA"], ["Own Goal","A_OWN_GOAL"], ["Own PenaltyArea","A_OWN_PENALTYAREA"], ["Opponent GoalArea","A_OPP_GOALAREA"], ["Opponent Goal","A_OPP_GOAL"], ["Opponent PenaltyArea","A_OPP_PENALTYAREA"]]), "area");
    this.setOutput(true, "Area");
    this.setColour(230);
 this.setTooltip("[Area] getArea( [Areas] )");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['pos_ispositioninarea'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("isPositionInArea");
    this.appendValueInput("pos")
        .setCheck("Position")
        .appendField("Position=");
    this.appendValueInput("area")
        .setCheck("Area")
        .appendField("Area=");
    this.setInputsInline(true);
    this.setOutput(true, "Boolean");
    this.setColour(230);
 this.setTooltip("[Boolean] isPositionInArea( [Position], [Area] )");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['pos_createposition'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("createPosition");
    this.appendValueInput("x")
        .setCheck("Number")
        .appendField("x=");
    this.appendValueInput("y")
        .setCheck("Number")
        .appendField("y=");
    this.setInputsInline(true);
    this.setOutput(true, "Position");
    this.setColour(230);
 this.setTooltip("[Position] createPosition( x, y )");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['pos_createpose'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("createPose");
    this.appendValueInput("x")
        .setCheck("Number")
        .appendField("x=");
    this.appendValueInput("y")
        .setCheck("Number")
        .appendField("y=");
    this.appendValueInput("Rz")
        .setCheck("Number")
        .appendField("Rz=");
    this.setInputsInline(true);
    this.setOutput(true, "Pose");
    this.setColour(230);
 this.setTooltip("[Pose] createPose(x, y, Rz)");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['pos_getposefromposition'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("getPoseFromPosition");
    this.appendValueInput("pos")
        .setCheck("Position")
        .appendField("pos=");
    this.appendValueInput("Rz")
        .setCheck("Number")
        .appendField("Rz=");
    this.setInputsInline(true);
    this.setOutput(true, "Pose");
    this.setColour(230);
 this.setTooltip("");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['pos_getpositionfrompose'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("getPositionFromPose");
    this.appendValueInput("pose")
        .setCheck("Pose")
        .appendField("pose=");
    this.setInputsInline(true);
    this.setOutput(true, "Position");
    this.setColour(230);
 this.setTooltip("[Position] getPositionFromPose( pose )");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['pos_anglebetweenpositions'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("angleBetweenPositions");
    this.appendValueInput("pos1")
        .setCheck("Position")
        .appendField("pos1=");
    this.appendValueInput("pos2")
        .setCheck("Position")
        .appendField("pos2=");
    this.setInputsInline(true);
    this.setOutput(true, "Number");
    this.setColour(230);
 this.setTooltip("[Number] angleBetweenPositions( [Position] pos1, [Position] pos2 )");
 this.setHelpUrl("");
  }
};
