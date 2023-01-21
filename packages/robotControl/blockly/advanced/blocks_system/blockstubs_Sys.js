
Blockly.Python['sys_abort'] = function(block) {
  var value_message = Blockly.Python.valueToCode(block, 'message', Blockly.Python.ORDER_ATOMIC);
  var code = 'raise ScenarioAbort(' + value_message + ')\n';
  return code;
};


Blockly.Python['sys_message'] = function(block) {
  var value_message = Blockly.Python.valueToCode(block, 'message', Blockly.Python.ORDER_ATOMIC);
  var code = 'self.add_message(robotNr, "info", ' + value_message + ')\n';
  return code;
};

Blockly.Python['sys_sleep'] = function(block) {
  var number_duration = block.getFieldValue('duration');
  var code = 'time.sleep(' + number_duration + ')\n';
  return code;
};

Blockly.Python['sys_system'] = function(block) {
  var txt_cmd = block.getFieldValue('cmd');
  var code = 'os.system("' + txt_cmd + '")\n';
  return code;
};
