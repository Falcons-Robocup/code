# Copyright 2019-2020 lucas (Falcons)
# SPDX-License-Identifier: Apache-2.0
import numpy as np
import numpy.linalg as LA
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

def draw_ellipse(coord, image, coord_converter):
    chi_95 = 5.991146        
    eigenValues, eigenVectors = LA.eig(coord.covariance)
    # sort eingen vectors to get the largest
    idx = eigenValues.argsort()[::-1]   
    eigenValues = eigenValues[idx]
    eigenVectors = eigenVectors[:,idx]

    pos = coord_converter.field_pos_to_img_pos(coord.mean)
    width = coord_converter.field_distance_to_img_distance(math.sqrt(eigenValues[0]*chi_95))
    height = coord_converter.field_distance_to_img_distance(math.sqrt(eigenValues[1]*chi_95))
    axes = (width, height)
    # 180 - angle because opencv and the field have y coordinate in different directions
    angle = 180.0-math.degrees(math.atan2(eigenVectors[1,0],eigenVectors[0,0]))
    color = (255, 0, 0)

    tmp_image = image.copy()
    cv2.ellipse(tmp_image, pos, axes, angle, 0.0, 360.0, color, -1, cv2.LINE_AA)

    alpha = 0.15
    cv2.addWeighted(tmp_image, alpha, image, 1-alpha, 0, image)
    cv2.ellipse(image, pos, axes, angle, 0.0, 360.0, color, 1, cv2.LINE_AA)

def draw_speed(coord, vel, img, coord_converter):  

    fpos = coord_converter.field_pos_to_img_pos(coord.mean)  
    fvelx = coord_converter.field_distance_to_img_distance(vel[0])  
    fvely = coord_converter.field_distance_to_img_distance(vel[1])  
    p1 = fpos
    p2 = (fpos[0] + fvelx, fpos[1] + fvely)
    print(p1)
    print(p2)
    color = (255,0,0)
    thickness = 2
    cv2.line(img, p1, p2, color, thickness)    
    
def draw_field(img, coord_converter):

    black = (0, 0, 0)

    cv2.rectangle(img, coord_converter.field_pos_to_img_pos((-6.0, -9.0)),
                       coord_converter.field_pos_to_img_pos(( 6.0,  9.0)), black)

    cv2.rectangle(img, coord_converter.field_pos_to_img_pos((-3.25, -9.0)),
                       coord_converter.field_pos_to_img_pos(( 3.25, -6.75)), black)

    cv2.rectangle(img, coord_converter.field_pos_to_img_pos((-3.25, 6.75)),
                       coord_converter.field_pos_to_img_pos(( 3.25, 9.0)), black)

    radius = coord_converter.field_distance_to_img_distance(2.0)
    cv2.circle(img, coord_converter.field_pos_to_img_pos((0.0, 0.0)), radius, black, 1, cv2.LINE_AA)


def coord_from_input(line):
    remove = '[]()'
    for char in remove:
        line = line.replace(char,'')

    v = line.split(',')
    vel = [0,0]
    if len(v) >= 8:
        vel = [float(v[6]),float(v[7])]
    coordGauss = create_coord_with_covariance([float(v[0]),float(v[1])], [[float(v[2]),float(v[3])],[float(v[4]),float(v[5])]])
    return (coordGauss, vel)

def load_obstacles(num_obstacles):
    coords = []
    i = 0
    while i<num_obstacles:
        line = blocking_read_line()
        #print(line.strip('\n'))
        if(line.startswith('M:')):
            line = line[2:]
            coords.append(coord_from_input(line))
            i = i + 1
    return coords

def read_int(line):
    return int((line.split(' '))[1])

def read_time(line):
    return float((line.split(' '))[0].split('=')[1])

def draw_obstacles(image, obstacles, coord_converter):
 
    i = 0
    for (coord, vel) in obstacles:
        draw_ellipse(coord, image, coord_converter)
        draw_speed(coord, vel, image, coord_converter)

        font = cv2.FONT_HERSHEY_SIMPLEX
        text_pos = coord_converter.field_pos_to_img_pos(coord.mean)
        cv2.putText(image, str(i), text_pos, font, 1, (255,255,255), 1, cv2.LINE_AA)
        i = i + 1

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

    while True:
        line = non_blocking_read_line()
        if line != '':    
            print(line.strip('\n'))
            if(line.startswith('Measurements:')):
                measurements = load_obstacles(read_int(line))                           
            elif(line.startswith('Obstacles:')):
                obstaclesPre = obstaclesPost
                obstaclesPost = load_obstacles(read_int(line))
            elif(line.startswith('t=')):
                latest_timestamp = read_time(line)
        else:
            background = video_reader.get_frame(latest_timestamp) 
            draw_obstacles(obstaclesPreFigure, obstaclesPre, background)
            draw_obstacles(measurementFigure, measurements, background)            
            draw_obstacles(obstaclesPostFigure, obstaclesPost, background)
            
        sys.stdout.flush()

def matplotlib_img_to_opencv_img(matplotlib_canvas):
    np_img = np.fromstring(matplotlib_canvas.tostring_rgb(), dtype='uint8')
    np_img = np_img.reshape(matplotlib_canvas.get_width_height()[::-1] + (3,))                
    np_img = cv2.cvtColor(np_img, cv2.COLOR_BGR2RGB)
    return np_img


def write_output_video(input_video_name, video_offset, output_video_name):
    measurements = []
    obstaclesPost = []

    input_thread = threading.Thread(target=add_input)
    input_thread.daemon = True
    input_thread.start()

    top_left = (20, 18)
    bottom_right = (572-16, 840-20)
    coord_converter = FieldToImageConverter(top_left, bottom_right, field_dim=(12.0 - 0.25, 18.0 - 0.25))
    video_reader = VideoReader(video_file_name, video_offset)
    latest_timestamp = 0

    print(coord_converter.field_pos_to_img_pos((-6.0, 9.0)))

    output_dim = (2*572, 840)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out_video = cv2.VideoWriter(output_video_name, fourcc, 60.0, output_dim)

    while True:
    #for i in range(300):
        line = blocking_read_line()
        if line != '':    
            if(line.startswith('Measurements:')):
                measurements = load_obstacles(read_int(line))                           
            elif(line.startswith('Obstacles:')):
                obstaclesPost = load_obstacles(read_int(line))
            elif(line.startswith('t=')):
                print(line.strip('\n'))
                latest_timestamp = read_time(line)
                background = video_reader.get_frame(latest_timestamp)

                if background is None:
                   background = np.zeros((840,572,3), np.uint8)
                   background[:,:] = (255, 255, 255)

                measurement_frame = background.copy()
                obstacles_frame = background.copy()
    
                draw_field(measurement_frame, coord_converter)
                draw_field(obstacles_frame, coord_converter)

                draw_obstacles(measurement_frame, measurements, coord_converter)            
                draw_obstacles(obstacles_frame, obstaclesPost, coord_converter)
                
                out_frame = np.concatenate((measurement_frame, obstacles_frame), axis=1)

                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(out_frame, str(latest_timestamp+video_offset), (0, 15), font, 0.6, (50,50,255), 1, cv2.LINE_AA)
                
                out_video.write(out_frame)
        else:
            break

            
        sys.stdout.flush()

    out_video.release()

class VideoReader:

    def __init__(self, video_file_name, video_offset):
        self.video = cv2.VideoCapture(video_file_name)
        self.offset = video_offset

    def get_frame(self, time):
        time_millis = (time + video_offset) * 1000.0

        self.video.set(cv2.CAP_PROP_POS_MSEC, time_millis)
        ret, frame = self.video.read()
        
        # video needs to be rotated to match field orientation
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

        return frame

class FieldToImageConverter:
    
    def __init__(self, top_left_pos, bottom_right_pos, field_dim):
        self.x_factor = (bottom_right_pos[0] - top_left_pos[0]) / field_dim[0]
        self.y_factor = -(bottom_right_pos[1] - top_left_pos[1]) / field_dim[1]
        print(self.x_factor)
        print(self.y_factor)

        field_center_x = (bottom_right_pos[0] + top_left_pos[0]) / 2.0
        field_center_y = (bottom_right_pos[1] + top_left_pos[1]) / 2.0
        self.img_field_center = (field_center_x, field_center_y)

    def field_pos_to_img_pos(self, field_pos):
        img_x = int(field_pos[0] * self.x_factor + self.img_field_center[0])
        img_y = int(field_pos[1] * self.y_factor + self.img_field_center[1])
        img_pos = (img_x, img_y)
        return img_pos

    def field_distance_to_img_distance(self, field_distance):
        img_dist = int(field_distance * self.x_factor)
        return img_dist


video_file_name = None
video_offset = 0.0
output_video = None

if len(sys.argv) > 1:
    video_file_name = sys.argv[1]

if len(sys.argv) > 2:
    video_offset = float(sys.argv[2])

if len(sys.argv) > 3:
    output_video = sys.argv[3]

if (output_video is None):
    input_loop(video_file_name, video_offset)
else:
    write_output_video(video_file_name, video_offset, output_video)







