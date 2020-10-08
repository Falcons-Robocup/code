""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/python

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from custom_checkbox import Checkbox
import warnings
warnings.filterwarnings("ignore", module="matplotlib") # suppress warning originating from removing bottom toolbar


NUMROBOTS=7



class PlotControl():
    def __init__(self):
        self.selectRobotBallMeasurements = [True] * (1+NUMROBOTS) # default show all
        self.selectRobotBallResults = [False] * (1+NUMROBOTS)
        self.selectRobotBallResults[0] = True # default show coach view (no 'ownBallsFirst' from robot perspective)
        self.connectRobotWithBallMeasurements = False # toggle connector lines (which can make the plot quite crowded)
        self.connectConsecutiveBallResults = False # toggle connector lines
        self.showRobotPositions = True
        self.showLowBallMeasurements = True
        self.showHighBallMeasurements = True # will use alpha transparency
        self.showLowBallResults = True
        self.showHighBallResults = True # will use alpha transparency
        self.showBallResultVelocity = False # TODO: implement
        self.highElevation = 0.0 # when to consider a ball measurement low or high (elevation)
        self.highBallZ = 0.7 # when to consider a ball result low or high (z)
        self.dataTimeout = 1.0
        self.ageMin = 0.0
        self.ageMax = 1e9
        self.ageSelect = None
        self.ageRange = None # handy for full-RDL mode
        # interactive matplotlib entities
        self.sliders = {}
        self.checkboxes = {}

    def selectedRobotsForBallMeasurements(self):
        return [r for r in range(1+NUMROBOTS) if self.selectRobotBallMeasurements[r]]

    def selectedRobotsForBallResults(self):
        return [r for r in range(1+NUMROBOTS) if self.selectRobotBallResults[r]]

    def update(self, *args):
        # copy widget state to self
        for r in range(1+NUMROBOTS):
            self.selectRobotBallMeasurements[r] = self.checkboxes['selectRobotBallMeasurements'][r].ticked
            self.selectRobotBallResults[r] = self.checkboxes['selectRobotBallResults'][r].ticked
        for k in self.checkboxes.keys():
            if not isinstance(self.checkboxes[k], list):
                setattr(self, k, self.checkboxes[k].ticked)
        for s in self.sliders.keys():
            setattr(self, s, self.sliders[s].val)

    def constructFigure(self):
        # setup the figure and connect data manipulators
        plt.rcParams['toolbar'] = 'None' # remove bottom toolbar, assume other figures are already constructed
        fig, ax = plt.subplots(figsize=[3.6, 5.8], frameon=False)
        fig.canvas.set_window_title('ball plotting options')
        axcolor = 'lightgoldenrodyellow'
        plt.axis('off') # disable main plot axis
        # first some sliders
        def addSlider(name, ypos, vmin, vmax):
            a = plt.axes([0.35, ypos, 0.50, 0.03], axisbg=axcolor)
            s = Slider(a, name, vmin, vmax, valinit=getattr(self, name))
            self.sliders[name] = s
            s.on_changed(self.update)
        y = 0.90
        h = 0.04
        addSlider('dataTimeout', y, 0.0, 5.0)
        y -= h
        addSlider('highElevation', y, -1.0, 1.0)
        y -= h
        addSlider('highBallZ', y, -1.0, 3.0)
        if self.ageRange != None: # age sliders only apply to 'rdl' mode
            self.ageMin = self.ageRange[0]
            self.ageMax = self.ageRange[1]
            self.ageSelect = self.ageMax
            y -= h
            addSlider('ageSelect', y, self.ageMin, self.ageMax)
            y -= h
        else:
            y -= 2*h
        # helper for checkboxes
        def addCheckbox(key, *args, **kwargs):
            c = Checkbox(*args, **kwargs)
            c.on_changed(self.update)
            if self.checkboxes.has_key(key):
                self.checkboxes[key].append(c)
            else:
                self.checkboxes[key] = c
        # robot-related checkboxes
        # NOTE: the standard matplotlib checkbuttons widget is a bit limited, so we made our own
        # (it doesn't nicely scale, it is multi-element including an annoying bounding box, has no simple state inspection)
        # TODO: column text X locations are slightly off... ?! w.r.t. a different axis?
        columns =  [('visball', 'selectRobotBallMeasurements', 0.35),
                    ('wmball', 'selectRobotBallResults', 0.65)]
        for col in columns:
            key = col[1]
            self.checkboxes[key] = []
            ax.text(col[2], y, col[0], horizontalalignment='left', verticalalignment='center')
        y -= 3*h
        for robotId in range(NUMROBOTS+1):
            for col in columns:
                key = col[1]
                name = key + 'R' + str(robotId)
                defaultValue = getattr(self, key)[robotId]
                label = ''
                if key == 'selectRobotBallMeasurements':
                    label = 'select r'+str(robotId)
                enabled = not (robotId == 0 and key == 'selectRobotBallMeasurements') # coach has no vision
                addCheckbox(key, name, [col[2], y, 0.10, h], default=defaultValue, label=label, enabled=enabled)
            y -= h
        rows = ['Low', 'High']
        columns =  [('BallMeasurements', 0.35),
                    ('BallResults', 0.65)]
        for row in rows:
            for col in columns:
                key = 'show' + row + col[0]
                defaultValue = getattr(self, key)
                label = ''
                if col[0] == 'BallMeasurements':
                    label = 'show ' + row.lower()
                addCheckbox(key, key, [col[1], y, 0.10, h], default=defaultValue, label=label)
            y -= h
        y -= h # extra empty row
        key = 'showRobotPositions'
        addCheckbox(key, key, [0.35, y, 0.10, h], default=getattr(self, key), label='show rpos')
        y -= h
        key = 'connectRobotWithBallMeasurements'
        addCheckbox(key, key, [0.35, y, 0.10, h], default=getattr(self, key), label='connect rpos')
        y -= h
        key = 'connectConsecutiveBallResults'
        addCheckbox(key, key, [0.35, y, 0.10, h], default=getattr(self, key), label='connect wmb')
        y -= h
        key = 'showBallResultVelocity'
        addCheckbox(key, key, [0.35, y, 0.10, h], default=getattr(self, key), label='ball velocity', enabled=False)
        # workaround to force draw before starting animation - figure would sometimes not come up during plot_balls_rdl.py
        fig.show()
        # done, return figure, so other things can be done with it
        return fig



if __name__ == "__main__":

    p = PlotControl()
    p.ageRange = [0, 3]
    p.constructFigure()
    plt.show()

