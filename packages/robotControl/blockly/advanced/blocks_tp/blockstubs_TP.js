
Blockly.Python['tp_role'] = function(block) {
    //  UNDEFINED = 0,
    //  GOALKEEPER = 1,
    //  ATTACKER_MAIN = 2,
    //  ATTACKER_ASSIST = 3,
    //  ATTACKER_GENERIC = 4,
    //  DEFENDER_MAIN = 5,
    //  DEFENDER_GENERIC = 6,
    //  DISABLED_OUT = 7,
    //  DISABLED_IN = 8
  var dropdown_role = block.getFieldValue('role');
  var code = 'rci.setTeamplayRole("' + dropdown_role + '")\n';
  return code;
};

Blockly.Python['tp_gamestate'] = function(block) {
  var dropdown_inmatch = block.getFieldValue('inMatch');
  var dropdown_gamestate = block.getFieldValue('gamestate');

  // OUT_OF_MATCH_NEUTRAL_STOPPED
  // OWN_PENALTY_SHOOTOUT_EXECUTE
  // OWN_PENALTY_SHOOTOUT_PREPARE
  // OPP_PENALTY_SHOOTOUT_EXECUTE
  // OPP_PENALTY_SHOOTOUT_PREPARE
  // PARKING
  // INVALID
  // DROPPED_BALL_EXECUTE
  // DROPPED_BALL_PREPARE
  // NEUTRAL_PLAYING
  // NEUTRAL_STOPPED
  // OPP_CORNER_EXECUTE
  // OPP_CORNER_PREPARE
  // OPP_FREEKICK_EXECUTE
  // OPP_FREEKICK_PREPARE
  // OPP_GOALKICK_EXECUTE
  // OPP_GOALKICK_PREPARE
  // OPP_KICKOFF_EXECUTE
  // OPP_KICKOFF_PREPARE
  // OPP_PENALTY_EXECUTE
  // OPP_PENALTY_PREPARE
  // OPP_THROWIN_EXECUTE
  // OPP_THROWIN_PREPARE
  // OWN_CORNER_EXECUTE
  // OWN_CORNER_PREPARE
  // OWN_FREEKICK_EXECUTE
  // OWN_FREEKICK_PREPARE
  // OWN_GOALKICK_EXECUTE
  // OWN_GOALKICK_PREPARE
  // OWN_KICKOFF_EXECUTE
  // OWN_KICKOFF_PREPARE
  // OWN_PENALTY_EXECUTE
  // OWN_PENALTY_PREPARE
  // OWN_THROWIN_EXECUTE
  // OWN_THROWIN_PREPARE

  var enum_name = "";
  if (dropdown_gamestate == "SETPIECE")
  {
      var dropdown_ownopp = block.getFieldValue('ownopp_f');
      var dropdown_setpiece = block.getFieldValue('setpiece');
      var dropdown_prepexecute = block.getFieldValue('prepexecute');
      if (dropdown_setpiece == "DROPPED_BALL")
      {
          enum_name = dropdown_setpiece + "_" + dropdown_prepexecute;
      }
      else
      {
          enum_name = dropdown_ownopp + "_" + dropdown_setpiece + "_" + dropdown_prepexecute;
      }
  }
  else
  {
      enum_name = dropdown_gamestate;
  }

  var code = 'rci.setTeamplayGameState("' + enum_name + '")\n';
  return code;
};



Blockly.Python['tp_blockuntilpassedorfailed'] = function(block) {
  var code = 'rci.blockUntilTPOverridePassedOrFailed()\n';
  return code;
};

Blockly.Python['tp_teleportball'] = function(block) {
  var value_x = Blockly.Python.valueToCode(block, 'x', Blockly.Python.ORDER_ATOMIC);
  var value_y = Blockly.Python.valueToCode(block, 'y', Blockly.Python.ORDER_ATOMIC);
  var code = 'simScene.run(simScene.TeleportBall(' + value_x + ', ' + value_y + '))\n';
  return code;
};
