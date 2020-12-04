
import numpy as np


class allMatrix:

    def __init__(self):
        print("creating matrices")
        self.l = 0.47/2 
        self.w = .3/2
        self.r = 0.0475
        wheel = self.w+self.l
        
        self.H = np.array([[-1/(wheel), 1/(wheel), 1/(wheel), -1/(wheel)],[1,1,1,1],[-1,1,-1,1]])

        self.F6 = np.zeros((6,4))
        self.F6[2:5] = self.r/4 * self.H

        #transformation from ....
        self.B_list_arm = np.array([ [0,0,0,0,0],
                                [0,-1,-1,-1,0],
                                [1,0,0,0,1],
                                [0,-0.5076,-0.3526,-0.2176,0],
                                [0.033,0,0,0,0],
                                [0,0,0,0,0]])
        #transformation from base of robot to robotic arm
        self.Tb0 = np.array([[1,0,0,0.1662],
                                    [0,1,0,0],
                                    [0,0,1,0.0026],
                                    [0,0,0,1]])

        #transformation from base of the robotic arm to eef 
        self.M0e = np.array([ [1,0,0,0.033],
                                    [0,1,0,0],
                                    [0,0,1,0.6546],
                                    [0,0,0,1]])