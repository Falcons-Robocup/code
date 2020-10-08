
Blockly.Python['sys_system'] = function(block) {
  var txt_cmd = block.getFieldValue('cmd');
  var code = 'os.system("' + txt_cmd + '")\n';
  return code;
};

