
Blockly.Blocks['tp_role'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Execute Role:")
        .appendField(new Blockly.FieldDropdown([["RobotStop","ROBOT_STOP"], ["Goalkeeper","GOALKEEPER"], ["Attacker Main","ATTACKER_MAIN"], ["Attacker Assist","ATTACKER_ASSIST"], ["Defender Main","DEFENDER_MAIN"], ["Defender Assist","DEFENDER_ASSIST"]]), "role");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("Role( [Role] role )");
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
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
    this.setTooltip("GameState( [GameState] gamestate )");
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
          .appendField(new Blockly.FieldDropdown([["Stopped","OUT_OF_MATCH_NEUTRAL_STOPPED"], ["Own Penalty Prepare","OWN_PENALTY_SHOOTOUT_PREPARE"], ["Own Penalty Execute","OWN_PENALTY_SHOOTOUT_EXECUTE"], ["Opponent Penalty Prepare","OPP_PENALTY_SHOOTOUT_PREPARE"], ["Opponent Penalty Execute","OPP_PENALTY_SHOOTOUT_EXECUTE"], ["Park","PARKING"]]), "gamestate")
    }
    else //IN_MATCH
    {
        // Replace gamestate with InMatch gamestates
        this.sourceBlock_.removeInput("gamestate", false);

        this.sourceBlock_.appendDummyInput("gamestate")
            .appendField(new Blockly.FieldDropdown([["Stopped","NEUTRAL_STOPPED"], ["Playing","NEUTRAL_PLAYING"], ["SetPiece","SETPIECE"]], this.sourceBlock_.gamestate_validator), "gamestate");
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



Blockly.Blocks['tp_blockuntilpassedorfailed'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Block until Teamplay PASSED or FAILED");
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
