Falcons BallTracking toolbox
(and obstacle tracking, it hardly differs)

This toolbox is meant for analysis/visualization and tuning+model development.
We could have put the visualization in 3D C++ visualizer, but that would be expensive to make and 
not fit with the other use cases anyway.

When analyzing, one loads the data from tracing into fbt_gui.
Data includes:
* raw ball measurements
* ball tracking results
* raw obstacle measurements
* obstacle tracking results
The GUI then can be used to browse through the data, select, run the offline model and compare/tune results.

Example:
    tgzdata = fbt_load_tgz('wmTracing_r4_20170211_morning_VDL.tgz');
    close all ; fbt_gui(tgzdata)

