""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
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
    coord2 = create_coord(mean=[3, 3], base1=[1, 1], variance1=5.5, base2=[-1, 1], variance2=1.1)

    coord3 = coord1 * coord2

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
([3.40,-7.32],[[0.1,0.0],[0.0,0.1]]),
([0.0938,-9.7839],[[0.0707,0.0],[0.0,0.0748]])
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







