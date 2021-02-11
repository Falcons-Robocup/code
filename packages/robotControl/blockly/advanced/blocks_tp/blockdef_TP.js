Blockly.Blocks['tp_action'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Execute Action:")
        .appendField(new Blockly.FieldDropdown([["Stop","STOP"], ["Move","MOVE"], ["Shoot","SHOOT"], ["Pass","PASS"], ["Position Before POI","POSITION_BEFORE_POI"], ["Position Behind POI","POSITION_BEHIND_POI"], ["Position For Opponent SetPiece","POSITION_FOR_OPP_SETPIECE"], ["Position For Own SetPiece","POSITION_FOR_OWN_SETPIECE"], ["Get Ball","GET_BALL"], ["Goalkeeper","GOALKEEPER"], ["Move To Free Spot","MOVE_TO_FREE_SPOT"], ["Intercept Ball","INTERCEPT_BALL"], ["Avoid POI","AVOID_POI"], ["Defend Penalty Area","DEFEND_PENALTY_AREA"], ["Turn Away From Opponent","TURN_AWAY_FROM_OPPONENT"], ["Defend Attacking Opponent","DEFEND_ATTACKING_OPPONENT"], ["Dribble For Pass","DRIBBLE_FOR_PASS"], ["Dribble For Shot","DRIBBLE_FOR_SHOT"]]), "action");
    this.appendValueInput("params")
        .setCheck("Parameters")
        .appendField("Parameters:");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
	this.setTooltip("Action( [Action] action, [Dictionary] params )");
	this.setHelpUrl("");
  }
};


Blockly.Blocks['tp_behavior'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Execute Behavior:")
        .appendField(new Blockly.FieldDropdown([["RobotStop","B_ROBOT_STOP"], ["Goalkeeper","B_GOALKEEPER"], ["Attacker Pass Ball Strategy","ATTACKER_PASS_BALL_STRATEGY"], ["Attack Assist","ATTACK_ASSIST"], ["Attack Main","ATTACK_MAIN"], ["Defend Assist","DEFEND_ASSIST"], ["Defender Potential Opponent Attacker","DEFEND_POTENTIAL_OPP_ATTACKER"], ["Defend Main","DEFEND_MAIN"], ["Defender Pass Ball Strategy","DEFENDER_PASS_BALL_STRATEGY"], ["Get Ball","GET_BALL"], ["Pass Ball To Closest Attacker","PASS_BALL_TO_CLOSEST_ATTACKER"], ["Pass Ball To Furthest Attacker","PASS_BALL_TO_FURTHEST_ATTACKER"], ["Pass Ball To Furthest Defender","PASS_BALL_TO_FURTHEST_DEFENDER"], ["Pass Ball To Closest Teammember","PASS_BALL_TO_CLOSEST_TEAMMEMBER"], ["Pass Ball To Closest Attacker On Opponent Half","PASS_BALL_TO_CLOSEST_ATTACKER_ON_OPP_HALF"], ["Position To Free Spot","POSITION_TO_FREE_SPOT"], ["Position To Shoot","POSITION_TO_SHOOT"], ["Position To Pass","POSITION_TO_PASS"], ["Receive Pass","RECEIVE_PASS"], ["Search Ball","SEARCH_BALL"], ["SetPiece","SETPIECE"], ["SetPiece Search Ball","SETPIECE_SEARCH_BALL"], ["Shoot At Goal","SHOOT_AT_GOAL"], ["Lob Shot On Goal","LOB_SHOT_ON_GOAL"], ["Tip-in Assist","TIP_IN_ASSIST"], ["Tip-in Execute","TIP_IN_EXECUTE"], ["Turn Away From Closest Opponent","TURN_AWAY_FROM_CLOSEST_OPPONENT"]]), "behavior");
    this.appendValueInput("params")
        .setCheck("Parameters")
        .appendField("Parameters:");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("Behavior( [Behavior] behavior, [Dictionary] params )");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['tp_setpiecebehavior'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Execute SetPiece Behavior:");
    this.appendDummyInput("ownopp")
        .appendField(new Blockly.FieldDropdown([["Own","OWN"], ["Opp","OPP"]]), "ownopp");
    this.appendDummyInput("setpiece")
        .appendField(new Blockly.FieldDropdown([["Kickoff","KICKOFF"], ["Freekick","FREEKICK"], ["Goalkick","GOALKICK"], ["Throw-in","THROWIN"], ["Corner","CORNER"], ["Penalty","PENALTY"], ["DropBall","DROPBALL"]], this.validator), "setpiece");
    this.appendDummyInput("prepexecutesearch")
        .appendField(new Blockly.FieldDropdown([["Prepare","PREPARE"], ["Execute","EXECUTE"], ["Search","SEARCH"]]), "prepexecutesearch");
    this.appendValueInput("params")
        .setCheck("Parameters")
        .appendField("Parameters:");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("SetPieceBehavior( [Setpieces], [Dictionary] params )");
 this.setHelpUrl("");
  },

  validator: function(newValue) {
    // Add/Remove Setpiece options when (de)selected
    if (newValue == "DROPBALL")
    {
        this.sourceBlock_.removeInput("ownopp", /* no error */ true);
    }
    else
    {
        if (this.sourceBlock_.getInput("ownopp") == null)
        {
            this.sourceBlock_.appendDummyInput("ownopp")
              .appendField(new Blockly.FieldDropdown([["Own","OWN"], ["Opponent","OPP"]]), "ownopp")

            // Move the new input before the parameters.
            this.sourceBlock_.moveInputBefore("ownopp", "setpiece");
        }
    }
  }
};

Blockly.Blocks['tp_role'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Execute Role:")
        .appendField(new Blockly.FieldDropdown([["RobotStop","R_ROBOT_STOP"], ["Goalkeeper","R_GOALKEEPER"], ["Attacker Main","ATTACKER_MAIN"], ["Attacker Assist","ATTACKER_ASSIST"], ["Defender Main","DEFENDER_MAIN"], ["Defender Assist","DEFENDER_ASSIST"]]), "role");
    this.appendValueInput("params")
        .setCheck("Parameters")
        .appendField("Parameters:");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("Role( [Role] role, [Dictionary] params )");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['tp_gamestate'] = {

  init: function() {
    this.appendDummyInput()
        .appendField("Execute GameState:");
    this.appendDummyInput("inMatch")
        .appendField(new Blockly.FieldDropdown([["In Match","IN_MATCH"], ["Out of Match","OUT_OF_MATCH"]], this.in_match_validator), "inMatch");
    this.appendDummyInput("gamestate")
        .appendField(new Blockly.FieldDropdown([["Stopped","NEUTRAL_STOPPED"], ["Playing","NEUTRAL_PLAYING"], ["SetPiece","SETPIECE"]], this.gamestate_validator), "gamestate");
    this.appendValueInput("params")
        .setCheck("Parameters")
        .appendField("Parameters:");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
    this.setTooltip("GameState( [GameState] gamestate, [Dictionary] params )");
    this.setHelpUrl("");

    // Default to Setpiece
    this.setFieldValue( "SETPIECE", "gamestate" );

    // Add setpiece items
    if (this.getFieldValue("gamestate") == "SETPIECE")
    {
        this.appendDummyInput("in_ownopp")
          .appendField(new Blockly.FieldDropdown([["Own","OWN"], ["Opponent","OPP"]]), "ownopp_f");
        this.appendDummyInput("in_setpiece")
          .appendField(new Blockly.FieldDropdown([["Kickoff","KICKOFF"], ["Freekick","FREEKICK"], ["Goalkick","GOALKICK"], ["Throw-in","THROWIN"], ["Corner","CORNER"], ["Penalty","PENALTY"], ["DropBall","DROPPED_BALL"]], this.setpieceoptions_validator), "setpiece");
        this.appendDummyInput("in_prepexecute")
          .appendField(new Blockly.FieldDropdown([["Prepare","PREPARE"], ["Execute","EXECUTE"]]), "prepexecute");

        // Move the new inputs before the parameters.
        this.moveInputBefore("in_ownopp", "params");
        this.moveInputBefore("in_setpiece", "params");
        this.moveInputBefore("in_prepexecute", "params");
    }
  },

  in_match_validator: function(newValue) {
    // Add/Remove InMatch / OutOfMatch options when (de)selected
    if (newValue == "OUT_OF_MATCH")
    {
        // Replace gamestate with OutOfMatch gamestates
        this.sourceBlock_.removeInput("gamestate", false);

        // Also remove setpieceoptions if it exists
        if (this.sourceBlock_.getInput("in_ownopp") != null)
        {
            this.sourceBlock_.removeInput("in_ownopp", false);
        }
        if (this.sourceBlock_.getInput("in_setpiece") != null)
        {
            this.sourceBlock_.removeInput("in_setpiece", false);
        }
        if (this.sourceBlock_.getInput("in_prepexecute") != null)
        {
            this.sourceBlock_.removeInput("in_prepexecute", false);
        }

        this.sourceBlock_.appendDummyInput("gamestate")
          .appendField(new Blockly.FieldDropdown([["Stopped","NEUTRAL_STOPPED"], ["Own Penalty Prepare","OWN_PENALTY_PREPARE"], ["Own Penalty Execute","OWN_PENALTY_EXECUTE"], ["Opponent Penalty Prepare","OPP_PENALTY_PREPARE"], ["Opponent Penalty Execute","OPP_PENALTY_EXECUTE"]]), "gamestate")

        // Move the new input before the parameters.
        this.sourceBlock_.moveInputBefore("gamestate", "params");
    }
    else //IN_MATCH
    {
        // Replace gamestate with InMatch gamestates
        this.sourceBlock_.removeInput("gamestate", false);

        this.sourceBlock_.appendDummyInput("gamestate")
            .appendField(new Blockly.FieldDropdown([["Stopped","NEUTRAL_STOPPED"], ["Playing","NEUTRAL_PLAYING"], ["SetPiece","SETPIECE"]], this.sourceBlock_.gamestate_validator), "gamestate");

        // Move the new input before the parameters.
        this.sourceBlock_.moveInputBefore("gamestate", "params");
    }
  },

  gamestate_validator: function(newValue) {
    // Add/Remove Setpiece options when (de)selected
    if (newValue == "SETPIECE")
    {
        this.sourceBlock_.appendDummyInput("in_ownopp")
          .appendField(new Blockly.FieldDropdown([["Own","OWN"], ["Opponent","OPP"]]), "ownopp_f");
        this.sourceBlock_.appendDummyInput("in_setpiece")
          .appendField(new Blockly.FieldDropdown([["Kickoff","KICKOFF"], ["Freekick","FREEKICK"], ["Goalkick","GOALKICK"], ["Throw-in","THROWIN"], ["Corner","CORNER"], ["Penalty","PENALTY"], ["DropBall","DROPPED_BALL"]], this.sourceBlock_.setpieceoptions_validator), "setpiece");
        this.sourceBlock_.appendDummyInput("in_prepexecute")
          .appendField(new Blockly.FieldDropdown([["Prepare","PREPARE"], ["Execute","EXECUTE"]]), "prepexecute");

        // Move the new inputs before the parameters.
        this.sourceBlock_.moveInputBefore("in_ownopp", "params");
        this.sourceBlock_.moveInputBefore("in_setpiece", "params");
        this.sourceBlock_.moveInputBefore("in_prepexecute", "params");
    }
    else
    {
        if (this.sourceBlock_.getInput("in_ownopp") != null)
        {
            this.sourceBlock_.removeInput("in_ownopp", false);
        }
        if (this.sourceBlock_.getInput("in_setpiece") != null)
        {
            this.sourceBlock_.removeInput("in_setpiece", false);
        }
        if (this.sourceBlock_.getInput("in_prepexecute") != null)
        {
            this.sourceBlock_.removeInput("in_prepexecute", false);
        }
    }
  },

  setpieceoptions_validator: function(newValue) {
    // Add/Remove Setpiece options when (de)selected
    if (newValue == "DROPPED_BALL")
    {
        // Remove "ownopp"
        this.sourceBlock_.removeInput("in_ownopp", false);
    }
    else
    {
        if (this.sourceBlock_.getInput("in_ownopp") == null)
        {
            this.sourceBlock_.appendDummyInput("in_ownopp")
              .appendField(new Blockly.FieldDropdown([["Own","OWN"], ["Opponent","OPP"]]), "ownopp_f")

            // Move the new input before the parameters.
            this.sourceBlock_.moveInputBefore("in_ownopp", "in_setpiece");
        }
    }
  }

};




Blockly.Blocks['tp_dictitem'] = {
  init: function() {

    this.appendValueInput("key")
        .setCheck("String")
        .appendField("key:");
        //.connection.connect(_keyBlock.outputConnection);
    this.appendValueInput("value")
        .setCheck("String")
        .appendField("value:");
        //.connection.connect(_valueBlock.outputConnection);
    this.setInputsInline(true);
    this.setOutput(true, null);
    this.setColour(230);
    this.setTooltip("");
    this.setHelpUrl("");
  }
};


Blockly.Blocks['tp_blockuntilpassedorfailed'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Block until PASSED or FAILED");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
    this.setTooltip("Blocking until PASSED or FAILED is returned by Teamplay");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['tp_teleportball'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("teleportBall");
    this.appendValueInput("x")
        .setCheck("Number")
        .appendField("x=");
    this.appendValueInput("y")
        .setCheck("Number")
        .appendField("y=");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("teleportBall( x, y )");
 this.setHelpUrl("");
  }
};
