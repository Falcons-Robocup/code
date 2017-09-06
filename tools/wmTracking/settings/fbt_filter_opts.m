function options = fbt_filter_opts()

XLIM          = 9;
YLIM          = 12;
options.robot = 0; % id 0 = all
options.tmin  = 0;
options.tmax  = 1e16;
options.xmin  = -XLIM;
options.xmax  =  XLIM;
options.ymin  = -YLIM;
options.ymax  =  YLIM;
options.zmin  = -1;
options.zmax  =  5;
options.confmin  = 0.5;
options.maxballs = 1;
