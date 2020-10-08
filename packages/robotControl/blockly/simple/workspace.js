$( "#toolboxContainer" ).load( "simple/toolbox.xml", function() {

    ////////////////////////
    // Initialize Blockly //
    ////////////////////////
    
    var toolbox = document.getElementById("toolbox");

    var options = { 
        toolbox : toolbox, 
        collapse : true, 
        comments : true, 
        disable : true, 
        maxBlocks : Infinity, 
        trashcan : true, 
        horizontalLayout : false, 
        toolboxPosition : 'start', 
        css : true, 
        rtl : false, 
        scrollbars : true, 
        sounds : true, 
        oneBasedIndex : true
    };

    /* Inject your workspace */ 
    var workspace = Blockly.inject("editor", options);

    /* Load Workspace Blocks from XML to workspace. Remove all code below if no blocks to load */

    /* TODO: Change workspace blocks XML ID if necessary. Can export workspace blocks XML from Workspace Factory. */
    var workspaceBlocks = document.getElementById("workspaceBlocks"); 

    /* Load blocks to workspace. */
    Blockly.Xml.domToWorkspace(workspaceBlocks, workspace);

    /* update code box */
    function myUpdateFunction(event) {
          var code = Blockly.Python.workspaceToCode(workspace);
            document.getElementById('pythoncode').value = code;
    }
    workspace.addChangeListener(myUpdateFunction);

    /////////////////////////////
    // Import & Export buttons //
    /////////////////////////////

    function exportBlockly()
    {
        var xml = Blockly.Xml.workspaceToDom(workspace);
        var xml_text = Blockly.Xml.domToText(xml);

        var fileType = "text/xml";
        var fileName = prompt("Enter the file name under which to save the current workspace.", "scenario.xml");

        if (fileName != null)
        {
            var blob = new Blob([xml_text], { type: fileType });

            var a = document.createElement('a');
            a.download = fileName;
            a.href = URL.createObjectURL(blob);
            a.dataset.downloadurl = [fileType, a.download, a.href].join(':');
            a.style.display = "none";
            document.body.appendChild(a);
            a.click();
            document.body.removeChild(a);
            setTimeout(function() { URL.revokeObjectURL(a.href); }, 1500);
        }
    }

    function importBlockly()
    {
        var input = document.createElement('input');
        input.type = 'file';

        input.onchange = e => { 

           // getting a hold of the file reference
           var file = e.target.files[0]; 

           // setting up the reader
           var reader = new FileReader();
           reader.readAsText(file,'UTF-8');

           // here we tell the reader what to do when it's done reading...
           reader.onload = readerEvent => {
              var content = readerEvent.target.result; // this is the content!

              // Write to Blockly Workspace
              var xml = Blockly.Xml.textToDom(content);
              Blockly.Xml.domToWorkspace(xml, workspace);
           }

        }

        input.click();
    }

    // Bind the import and export functions to the buttons
    $("#exportBlockly").on( "click", exportBlockly );
    $("#importBlockly").on( "click", importBlockly );
});
