# Copyright 2020 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/python

import numpy as np
from matplotlib.patches import Ellipse, Rectangle, Circle



def drawField(ax): # TODO move to EnvironmentField and use json field dimensions
    # NOTE: x and y are inverted because we like to see the field in 'landscape' mode, not 'portrait'
    p = []
    p.append(ax.add_patch(Rectangle((-9.0,-6.0), 18.0, 12.0, fill=False)))
    p.append(ax.add_patch(Rectangle((-9.0,-3.25), 2.25, 6.5, fill=False)))
    p.append(ax.add_patch(Rectangle((9.0-2.25,-3.25), 2.25, 6.5, fill=False)))
    p.append(ax.add_patch(Circle((0.0,0.0), 2.0, fill=False)))
    ax.set_xticks(np.arange(-20,20,2))
    ax.set_yticks(np.arange(-20,20,2))
    ax.set_yticklabels([str(v) for v in range(20, -20, -2)]) # invert axis labels ...
    # TODO: a better way to rotate the plot to align with our FCS might be to rotate entire axis, see
    # https://stackoverflow.com/questions/21652631/how-to-rotate-a-simple-matplotlib-axes
    ax.set_ylim(-7, 7)
    ax.set_xlim(-10, 17) # extra space to the right for legend
    ax.grid(True)
    return p

