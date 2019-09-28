""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 import numpy as np
import numpy.linalg as LA
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from matplotlib.patches import Rectangle
from matplotlib.patches import Circle
import cv2
import math
import sys
import Queue
import threading
import time

class Gaussian_2d:
    def __init__(self, mean, covariance):
        self.mean = mean
        self.covariance = covariance

    @classmethod
    def create_from_coordinates(cls, mean, base1, variance1, base2, variance2):
        V = np.concatenate((base1, base2), axis=1)
        L = np.matrix([[variance1, 0],[0, variance2]])
        E = V * L * V.getI()
        return cls(mean, E)  

    def __mul__(self, other):
        new_covariance = (self.covariance.getI() + other.covariance.getI()).getI()
        new_mean = new_covariance*(self.covariance.getI()*self.mean + other.covariance.getI()*other.mean)        
        return Gaussian_2d(new_mean, new_covariance)

    
    def getIntersection(self, other):
        Zc = (1.0/math.sqrt(2*math.pi*LA.det(self.covariance + other.covariance))) * \
             math.exp(-0.5 * (self.mean - other.mean).T * (self.covariance + other.covariance).getI() * (self.mean - other.mean))
        return Zc


def create_coord(mean, base1, variance1, base2, variance2):
    return Gaussian_2d.create_from_coordinates(np.matrix(mean).T, np.matrix(base1).T, variance1, np.matrix(base2).T, variance2)

def create_coord_with_covariance(mean, covariance):
    return Gaussian_2d(np.matrix(mean).T, np.matrix(covariance))

def createEllipse(coord):
    chi_95 = 5.991146        
    eigenValues, eigenVectors = LA.eig(coord.covariance)
    # sort eingen vectors to get the largest
    idx = eigenValues.argsort()[::-1]
    eigenValues = eigenValues[idx]
    eigenVectors = eigenVectors[:,idx]

    ellipse = Ellipse(xy=coord.mean, 
                      width=2*math.sqrt(eigenValues[0]*chi_95), 
                      height=2*math.sqrt(eigenValues[1]*chi_95),  
                      angle=math.degrees(math.atan2(eigenVectors[1,0],eigenVectors[0,0])))
    return ellipse


def plot_field(plot, video_img):

    if not(video_img is None):
        plot.imshow(video_img, extent=[-6.35, 6.35, -9.4, 9.4])

    plot.add_patch(Rectangle((-6.0,-9.0), 12.0, 18.0, fill=False))
    plot.add_patch(Rectangle((-3.25,-9.0), 6.5, 2.25, fill=False))
    plot.add_patch(Rectangle((-3.25,9.0-2.25), 6.5, 2.25, fill=False))
    plot.add_patch(Circle((0.0,0.0), 2.0, fill=False))

def gaussian_plotter():
    
    coords = [
([1.612771,-4.999359],[[0.637779,-0.438923],[-0.438923,0.362282]]),
([1.464921,-2.255993],[[0.572718,0.023602],[0.023602,0.041046]]),
([-1.961218,-6.171248],[[0.040083,0.007451],[0.007451,0.708885]]),
([-4.322652,-0.783544],[[0.301332,-0.176411],[-0.176411,0.159086]]),
([1.612771,-4.999359],[[0.637779,-0.438923],[-0.438923,0.362282]]),
([1.466376,-2.246887],[[0.573220,0.025049],[0.025049,0.041177]]),
([-1.950447,-6.171213],[[0.040046,0.005537],[0.005537,0.708870]]),
([-4.322652,-0.783544],[[0.301332,-0.176411],[-0.176411,0.159086]]),

                ]

    ells = []
    for coord in coords:
        coordGauss = create_coord_with_covariance(coord[0], coord[1])
        ells.append(createEllipse(coordGauss)) 

    a = plt.subplot(111, aspect='equal')
    plot_field(a)

    for e in ells:
        e.set_clip_box(a.bbox)
        e.set_alpha(0.1)
        a.add_artist(e)

    plt.xlim(-10, 10)
    plt.ylim(-10, 10)
    plt.grid()
    plt.show() 


def coord_from_input(line):
    remove = '[]()'
    for char in remove:
        line = line.replace(char,'')

    v = line.split(',')
    coordGauss = create_coord_with_covariance([float(v[0]),float(v[1])], [[float(v[2]),float(v[3])],[float(v[4]),float(v[5])]])
    return coordGauss

def load_obstacles(num_obstacles):
    coords = []
    i = 0
    while i<num_obstacles:
        line = blocking_read_line()
        print line.strip('\n')
        if(line.startswith('M:')):
            line = line[2:]
            coords.append(coord_from_input(line))
            i = i + 1
    return coords

def read_int(line):
    return int((line.split(' '))[1])

def read_time(line):
    return float((line.split(' '))[0].split('=')[1])

def draw_obstacles(figure, obstacles, background):
    figure.clf()
    plot = figure.add_subplot(111, aspect='equal')

    plot_field(plot, background)

    i = 0
    for obs in obstacles:
        e = createEllipse(obs)
        e.set_clip_box(plot.bbox)
        e.set_alpha(0.15)
        plot.add_artist(e)
        plot.text(obs.mean[0], obs.mean[1], i, color='white')
        i = i + 1

    plot.set_xlim(-7, 7)
    plot.set_ylim(-10, 10)
    plot.grid()   
    figure.canvas.draw()

input_queue = Queue.Queue()
def add_input():
    while True:
        input_queue.put(sys.stdin.readline())

def blocking_read_line():
    while input_queue.empty():
        time.sleep(0.05)
    return input_queue.get()

def non_blocking_read_line():
    if not input_queue.empty():
        return input_queue.get()
    else:
        return ''

def input_loop(video_file_name, video_offset):
    obstaclesPre = []
    measurements = []
    obstaclesPost = []

    plt.ion()
    obstaclesPreFigure = plt.figure()
    measurementFigure = plt.figure()
    obstaclesPostFigure = plt.figure()
    
    input_thread = threading.Thread(target=add_input)
    input_thread.daemon = True
    input_thread.start()

    video_reader = VideoReader(video_file_name, video_offset)
    latest_timestamp = 0
    data_count = 0

    while True:
        line = non_blocking_read_line()
        if line != '':    
            #print line.strip('\n')
            if(line.startswith('Measurements:')):
                pass;                
                #measurements = load_obstacles(read_int(line))                           
            elif(line.startswith('Obstacles:')):
                obstaclesPre = obstaclesPost
                obstaclesPost = load_obstacles(read_int(line))
                data_count = data_count + 1
            elif(line.startswith('t=')):
                latest_timestamp = read_time(line)
            elif(line.startswith('2019')):
                print line.strip('\n')
        #else:  
        if data_count % 100 == 0:
            background = video_reader.get_frame(latest_timestamp) 
            draw_obstacles(obstaclesPreFigure, obstaclesPre, background)
            draw_obstacles(measurementFigure, measurements, background)            
            draw_obstacles(obstaclesPostFigure, obstaclesPost, background)
            #time.sleep(0.01)
            data_count = data_count + 1

            
        sys.stdout.flush()

def matplotlib_img_to_opencv_img(matplotlib_canvas):
    np_img = np.fromstring(matplotlib_canvas.tostring_rgb(), dtype='uint8')
    np_img = np_img.reshape(matplotlib_canvas.get_width_height()[::-1] + (3,))                
    np_img = cv2.cvtColor(np_img, cv2.COLOR_BGR2RGB)
    return np_img

class VideoReader:

    def __init__(self, video_file_name, video_offset):
        if video_file_name is None:
            self.video = None
        else:
            self.video = cv2.VideoCapture(video_file_name)
            self.offset = video_offset

    def get_frame(self, time):
        if self.video is None:
            return None

        time_millis = (time + video_offset) * 1000.0

        self.video.set(cv2.CAP_PROP_POS_MSEC, time_millis)
        ret, frame = self.video.read()
        
        # video needs to be rotated to match field orientation
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        return frame


video_file_name = None
video_offset = 0

if len(sys.argv) > 1:
    video_file_name = sys.argv[1]

if len(sys.argv) > 2:
    video_offset = float(sys.argv[2])

input_loop(video_file_name, video_offset)







