
Blockly.Blocks['sys_abort'] = {
  init: function() {
    this.appendValueInput("message")
        .setCheck("String")
        .appendField("abort: message=");
    this.setPreviousStatement(true, null);
    this.setColour(65);
    this.setTooltip("Aborts the scenario execution with the specified reason message");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['sys_message'] = {
  init: function() {
    this.appendValueInput("message")
        .setCheck("String")
        .appendField("message");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(65);
 this.setTooltip("Show a message alert");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['sys_sleep'] = {
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

Blockly.Blocks['sys_system'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("system(\"")
        .appendField(new Blockly.FieldTextInput('...'), 'cmd')
        .appendField("\")");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(65);
 this.setTooltip("system( cmd )");
 this.setHelpUrl("");
  }
};

