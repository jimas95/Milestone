
import numpy as np


class allMatrix:

    def __init__(self):
        print("creating matrices")
        self.l = 0.47/2 
        self.w = .3/2
        self.r = 0.0475
        wheel = self.w+self.l
        
        self.H = np.array([ [-1/(wheel), 1/(wheel), 1/(wheel), -1/(wheel)],
                            [1,1,1,1],
                            [-1,1,-1,1]])

        self.F6 = np.zeros((6,4))
        self.F6[2:5] = self.r/4 * self.H

#       When the arm is at its home configuration, the screw axes
        self.Blist = np.array([ [0,0,0,0,0],
                                [0,-1,-1,-1,0],
                                [1,0,0,0,1],
                                [0,-0.5076,-0.3526,-0.2176,0],
                                [0.033,0,0,0,0],
                                [0,0,0,0,0]])
#       The fixed offset from the chassis frame {b} to the base frame of the arm {0} 
        self.Tb0 = np.array([[1,0,0,0.1662],
                            [0,1,0,0],
                            [0,0,1,0.0026],
                            [0,0,0,1]])

#       When the arm is at its home configuration (all joint angles zero) 
#       the end-effector frame {e} relative to the arm base frame {0}
        self.M0e =np.array([[1,0,0,0.033],
                            [0,1,0,0],
                            [0,0,1,0.6546],
                            [0,0,0,1]])