
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


        #for trajecotry generation testing
        self.Tse_init = np.array([   [1, 0, 0, 0],
                                [0, 1, 0, 0],
                                [0, 0, 1, 0.4],
                                [0, 0, 0, 1,]])

        self.Tsc_init = np.array([   [1, 0, 0, 1],
                                [0, 1, 0, 0],
                                [0, 0, 1, 0.025],
                                [0, 0, 0, 1,]])

        self.Tsc_final = np.array(   [[0, 1, 0, 0],
                                [-1, 0, 0, -1],
                                [0, 0, 1, 0.025],
                                [0, 0, 0, 1,]])

        self.Tce_grasp = np.array([  [-1/np.sqrt(2), 0, 1/np.sqrt(2), 0],
                                [0, 1, 0, 0],
                                [-1/np.sqrt(2), 0, -1/np.sqrt(2), 0],
                                [0, 0, 0, 1]])

        self.Tce_standoff = np.array([[-1/np.sqrt(2), 0, 1/np.sqrt(2), -.15],
                                [0, 1, 0, 0],
                                [-1/np.sqrt(2), 0, -1/np.sqrt(2), .15],
                                [0, 0, 0, 1]])








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


        #initial matrices for test_Feedforward_Control 
        self.test_Xd = np.array([[0,0,1,0.5],
                                [0,1,0,0],
                                [-1,0,0,0.5],
                                [0,0,0,1]])
        self.test_Xd_next = np.array([[0,0,1,0.6],
                                    [0,1,0,0],
                                    [-1,0,0,0.3],
                                    [0,0,0,1]])

        self.test_X = np.array([[0.17, 0, 0.985, 0.387],
                                [0, 1, 0 , 0],
                                [-0.985, 0, .17, 0.57],
                                [0, 0, 0, 1]])


    def list_to_array(self,myList):
        res = np.array([[myList[0],myList[1],myList[2],myList[9]],
                        [myList[3],myList[4],myList[5],myList[10]],
                        [myList[6],myList[7],myList[8],myList[11]],
                        [0,0,0,1]])
        return res

    def get_Tsb(self,q):
        Tsb = np.array([[np.cos(q[0]),-np.sin(q[0]),0,q[1]],
                                [np.sin(q[0]),np.cos(q[0]),0,q[2]],
                                [0,0,1,0.0963],
                                [0,0,0,1]])
        return Tsb

