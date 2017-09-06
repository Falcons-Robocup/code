proof of concept of 3D balltracking

features:
* read/write measurements (can load from tracing)
* direct fit of ball vector in FCS, including speed
* coordinate transformations towards distance weighting
* scene/measurement simulator, including most known error sources

to be implemented:
* camera Z
* distance weighting
* residuals handling
* bounce detection
* monte-carlo simulator
* GUI with sliders for realtime scene/solver control

to run:
* start octave interpreter
  * to install octave: 
    sudo apt-get install octave
* type: fbt_test
* this will run basic test cases
* the final test case is to plot a few scenes, each of which shows some ball measurements and the fit result (black)

code naming conventions:
fct = falcons coordinate transformations
fbt = falcons ball tracking



