
Blockly.Python['tp_action'] = function(block) {
  var dropdown_action = block.getFieldValue('action');
  var value_params = Blockly.Python.valueToCode(block, 'params', Blockly.Python.ORDER_ATOMIC);
  var code = 'rci.setTeamplayAction("' + dropdown_action + '", ' + value_params + ')\n';
  return code;
};

Blockly.Python['tp_behavior'] = function(block) {
  var dropdown_behavior = block.getFieldValue('behavior');
  var value_params = Blockly.Python.valueToCode(block, 'params', Blockly.Python.ORDER_ATOMIC);
  var code = 'rci.setTeamplayBehavior("' + dropdown_behavior + '", ' + value_params + ')\n';
  return code;
};

Blockly.Python['tp_setpiecebehavior'] = function(block) {
  var dropdown_ownopp = block.getFieldValue('ownopp');
  var dropdown_setpiece = block.getFieldValue('setpiece');
  var dropdown_prepexecutesearch = block.getFieldValue('prepexecutesearch');
  var value_params = Blockly.Python.valueToCode(block, 'params', Blockly.Python.ORDER_ATOMIC);

  var enum_name = "";
  if (dropdown_setpiece == "DROPBALL")
  {
      enum_name = dropdown_setpiece + "_" + dropdown_prepexecutesearch;
  }
  else
  {
      enum_name = dropdown_ownopp + "_" + dropdown_setpiece + "_" + dropdown_prepexecutesearch;
  }

  var code = 'rci.setTeamplayBehavior("' + enum_name + '", ' + value_params + ')\n';
  return code;
};

Blockly.Python['tp_role'] = function(block) {
  var dropdown_role = block.getFieldValue('role');
  var value_params = Blockly.Python.valueToCode(block, 'params', Blockly.Python.ORDER_ATOMIC);
  var code = 'rci.setTeamplayRole("' + dropdown_role + '", ' + value_params + ')\n';
  return code;
};

Blockly.Python['tp_gamestate'] = function(block) {
  var dropdown_inmatch = block.getFieldValue('inMatch');
  var dropdown_gamestate = block.getFieldValue('gamestate');
  var value_params = Blockly.Python.valueToCode(block, 'params', Blockly.Python.ORDER_ATOMIC);

  var enum_name = "";
  if (dropdown_gamestate == "SETPIECE")
  {
      var dropdown_ownopp = block.getFieldValue('ownopp_f');
      var dropdown_setpiece = block.getFieldValue('setpiece');
      var dropdown_prepexecute = block.getFieldValue('prepexecute');
      if (dropdown_setpiece == "DROPPED_BALL")
      {
          enum_name = dropdown_inmatch + "_" + dropdown_setpiece + "_" + dropdown_prepexecute + "_NEUTRAL";
      }
      else
      {
          enum_name = dropdown_inmatch + "_" + dropdown_ownopp + "_" + dropdown_setpiece + "_" + dropdown_prepexecute + "_NEUTRAL";
      }
  }
  else
  {
      enum_name = dropdown_inmatch + "_" + dropdown_gamestate + "_NEUTRAL";
  }

  var code = 'rci.setTeamplayGameState("' + enum_name + '", ' + value_params + ')\n';
  return code;
};








Blockly.Python['tp_parameters'] = function(block) {
  // Create a dict with any number of elements of any type.
  var elements = new Array(block.itemCount_);

  for (var i = 0; i < block.itemCount_; i++) {
    var child = block.getInputTargetBlock('ADD' + i);

    if (child === null || child.type != 'dictitem') {
      elements[i] = Blockly.Python.blank + ": " + Blockly.Python.blank;
      continue;
    }

    var key = Blockly.Python.valueToCode(child, 'key', Blockly.Python.ORDER_NONE) || Blockly.Python.blank;
    var value = Blockly.Python.valueToCode(child, 'value', Blockly.Python.ORDER_NONE) || Blockly.Python.blank;
    elements[i] = key + ": " + value;
  }

  var code = '{' + elements.join(', ') + '}';
  return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python['tp_dictitem'] = function(block) {
  var value_key = Blockly.Python.valueToCode(block, 'key', Blockly.Python.ORDER_ATOMIC);
  var value_value = Blockly.Python.valueToCode(block, 'value', Blockly.Python.ORDER_ATOMIC);
  return ["", Blockly.Python.ORDER_NONE];
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
