""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
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

