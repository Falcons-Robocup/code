proof of concept of 2D fit
including reduced weight of distance

to run:
* start octave interpreter
  * to install octave: 
    sudo apt-get install octave
* type: balltracking2d
* a window will popup which shows 2 ball measurements (blue and red) and the fit result (black)
* if the distance weight factor is chosen to be 1, then the fit result will simply be the geometric average
* if the distance weight factor is chosen LOWER, then the fit result will move closer to the line intersection



auxiliary files for coordinate transformations: pad1.m, translate.m and rotate.m


next steps:
* 3d extension
* fit also speed from timed series of measurements
* simulator

