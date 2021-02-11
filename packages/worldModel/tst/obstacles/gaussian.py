# Copyright 2018-2020 lucas (Falcons)
# SPDX-License-Identifier: Apache-2.0
import numpy as np
import numpy.linalg as LA
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from matplotlib.patches import Rectangle
import math

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

    def __add__(self, other):
        new_covariance = self.covariance + other.covariance
        new_mean = self.mean + other.mean
        return Gaussian_2d(new_mean, new_covariance)

    def __sub__(self, other):
        new_covariance = self.covariance + other.covariance
        new_mean = self.mean - other.mean
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


def gaussian_demo():    
    coords = []

    coord1 = create_coord(mean=[-3, -3], base1=[1, 1], variance1=0.5, base2=[-1, 1] , variance2=0.1)
    coord2 = create_coord(mean=[3, 3], base1=[1, 1], variance1=1.5, base2=[-1, 1], variance2=1.1)

    coord3 = coord1 - coord2

    coords.append(coord1)
    coords.append(coord2)
    coords.append(coord3)

    print(coord1.mean)
    print(coord1.covariance)

    print(coord2.mean)
    print(coord2.covariance)

    print(coord3.mean)
    print(coord3.covariance)

    print(coord1.getIntersection(coord2))

    ells = []
    for coord in coords:
        ells.append(createEllipse(coord))

    a = plt.subplot(111, aspect='equal')
    for e in ells:
        e.set_clip_box(a.bbox)
        e.set_alpha(0.1)
        a.add_artist(e)

    plt.xlim(-10, 10)
    plt.ylim(-10, 10)

    plt.show()

def plot_field(plot):
    plot.add_patch(Rectangle((-6.0,-9.0),12.0,18.0, fill=False))

def gaussian_plotter():
    
    coords = [
                #([3.40,-7.32],[[0.1,0.0],[0.0,0.1]]),
                #([0.0938,-9.7839],[[0.0707,0.0],[0.0,0.0748]])
                # ([0.153821,0.154296],[[0.587269,0.461757],[0.461757,0.363086]]),
                # ([0.190369,0.171185],[[0.597275,0.466794],[0.466794,0.364835]]),
                # ([-0.006389,0.312845],[[0.013642,-0.147203],[-0.147203,1.589602]]),
                # ([0.009416,0.316041],[[0.012772,-0.142491],[-0.142491,1.590997]]),
                # ([-0.166881,0.433461],[[0.018625,0.196656],[0.196656,2.077523]]),
                # ([0.190418,0.169852],[[0.597435,0.466755],[0.466755,0.364675]]),
                # ([0.019873,0.211546],[[0.012466,-0.139386],[-0.139386,1.559802]]),
                # ([-0.154068,0.432208],[[0.019363,0.200480],[0.200480,2.076785]])

                ([0.18,-2.95],[[0.6617, 0.2322],[0.2322,0.1514]]),
                ([0.27,-3.6599],[[0.6841, 0.0511],[0.0511,0.0677]]),
                ([-2.83,-6.42],[[0.1118, 0.1829],[0.1829,0.7575]]),
                ([-2.79,-6.43],[[0.1072, 0.1743],[0.1743,0.7613]])



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

gaussian_plotter()
#gaussian_demo()







