
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

