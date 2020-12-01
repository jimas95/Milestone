import numpy as np
import csv
import time
import functions
import modern_robotics as mr

class robotControler:

    def __init__(self):
        print("init robot Controler")

        path = "input.csv"
        self.init_conditions = []
        self.get_input(path)
        functions.display_init_conditions(self.init_conditions)

        #velocity limits
        self.wheel_limit = 10
        self.joint_limit = 10

        #time step
        self.dt = 0.01

        #wheel length 
        self.l = 0.47/2 
        self.w = .3/2

        #trajectory
        self.order_method = 5
        self.N = 200


    def next_state(self,input_state,velocity):
        """
        Input :
            input_state --> 13-vector representing the current configuration of the robot 
                            3 variables for the chassis configuration,
                            5 variables for the arm configuration, and 4 variables for the wheel angles
                            1 variable for gripper state
            velocity    --> A 9-vector of controls indicating the arm joint speeds theta
                            (5 variables) and the wheel speeds u (4 variables).
        Output:
            output_state--> A 12-vector representing the configuration of the robot time Δt later
        """
        
        
        #check velocity limits 
        velocity = functions.check_limits(velocity,self.joint_limit,self.wheel_limit)
        #update state
        output_state = np.zeros(13)
        #update chassis configuration which is obtained from odometry

        wheel = self.w+self.l
        H_mat = np.array([[-1/(wheel), 1/(wheel), 1/(wheel), -1/(wheel)],[1,1,1,1],[-1,1,-1,1]])
        output_state[0:3] = 0.0475/4. * np.matmul(H_mat, velocity[5:]) * self.dt + input_state[0:3]
        #update joints and wheels configuration
        output_state[3:12] = input_state[3:12] + np.array(velocity[:])*self.dt
        #copy gripper state
        output_state[-1] = input_state[-1]
        return output_state

    def TrajectoryGenerator(self,Tse_init,Tsc_init,Tsc_final,Tce_grasp,Tce_standoff,N):
        """
        The initial configuration of the end-effector in the reference trajectory: Tse,initial.
        The cube's initial configuration: Tsc,initial.
        The cube's desired final configuration: Tsc,final.
        The end-effector's configuration relative to the cube when it is grasping the cube: Tce,grasp.
        The end-effector's standoff configuration above the cube,
        """
        print("Generating trajectory....")
        tf = 10
        trajectory = [] 

        state_init      = Tse_init
        state_pregrasp  = np.matmul(Tsc_init,Tce_standoff)
        state_grasp     = np.matmul(Tsc_init,Tce_grasp)
        state_preplace  = np.matmul(Tsc_final,Tce_standoff)
        state_place     = np.matmul(Tsc_final,Tce_grasp)
        

        self.call_ScrewTrajectory(state_init        ,state_pregrasp ,tf,N,self.order_method,0,trajectory)
        self.call_ScrewTrajectory(state_pregrasp    ,state_grasp    ,tf,N,self.order_method,0,trajectory)
        self.call_ScrewTrajectory(state_grasp       ,state_grasp    ,tf,N,self.order_method,1,trajectory)
        self.call_ScrewTrajectory(state_grasp       ,state_pregrasp ,tf,N,self.order_method,1,trajectory)
        self.call_ScrewTrajectory(state_pregrasp    ,state_preplace ,tf,N,self.order_method,1,trajectory)
        self.call_ScrewTrajectory(state_preplace    ,state_place    ,tf,N,self.order_method,1,trajectory)
        self.call_ScrewTrajectory(state_place       ,state_place    ,tf,N,self.order_method,0,trajectory)
        self.call_ScrewTrajectory(state_place       ,state_preplace ,tf,N,self.order_method,0,trajectory)
        self.call_ScrewTrajectory(state_place       ,state_init     ,tf,N,self.order_method,0,trajectory)


        functions.write_csv(trajectory,"trajectory")

    def call_ScrewTrajectory(self,start, end, tf, N,order_method,gripper_state,trajectory_list):
        """
        call the ScrewTrajectory from moder robotics
            :param start: The initial end-effector configuration
            :param end: The final end-effector configuration
            :param Tf: Total time of the motion in seconds from rest to rest
            :param N: The number of points N > 1 (Start and stop) in the discrete
                    representation of the trajectory
            :param method: The time-scaling method, where 3 indicates cubic (third-
                        order polynomial) time scaling and 5 indicates quintic
                        (fifth-order polynomial) time scaling
            :return: The discretized trajectory as a list of N matrices in SE(3)
                    separated in time by Tf/(N-1). The first in the list is Xstart
                    and the Nth is Xend
        """
        trajectory = mr.ScrewTrajectory(start, end, tf, N, order_method) 
        functions.convertToCsvForm(trajectory_list,trajectory,gripper_state)

    def feedback_control(self):
        """
        """
        pass

    def run(self,id):
        """
        runs the whole thing....
        """
        print("Executing Run.....")

        if(id==1):
            print("Executing test nextState")
            self.test_nextState()
        elif(id==2):
            print("Executing test trajectory Generator")
            self.test_trajectory()



    def test_nextState(self):
        #initial values 
        start = np.array([0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0])
        velocity_1    = np.array([0, 0,0,0,0,10,10,10,10]) 
        velocity_2    = np.array([0, 0,0,0,0,-10,10,-10,10]) 
        velocity_3    = np.array([0, 0,0,0,0,-10,10,10,-10]) 

        #test of forward
        data = []
        print("Executing nextState forward")
        out = self.next_state(start,velocity_1)
        for i in range(100):
            out = self.next_state(out,velocity_1)
            data.append(out)
        functions.write_csv(data,"nextState_forward")
        print("last configuration -->")
        print(data[-1])
        data = []

        #test of sideways
        print("Executing nextState sideways")
        out = self.next_state(start,velocity_2)
        for i in range(100):
            out = self.next_state(out,velocity_2)
            data.append(out)
        functions.write_csv(data,"nextState_sideways")
        print("last configuration -->")
        print(data[-1])

        #test of turning
        data = []
        print("Executing nextState turn")
        out = self.next_state(start,velocity_3)
        for i in range(100):
            out = self.next_state(out,velocity_3)
            data.append(out)
        functions.write_csv(data,"nextState_turn")
        print("last configuration -->")
        print(data[-1])


    def test_trajectory(self):
        Tse_init = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0.4],
                            [0, 0, 0, 1,]])

        Tsc_init = np.array([[1, 0, 0, 1],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0.025],
                            [0, 0, 0, 1,]])

        Tsc_final = np.array([[0, 1, 0, 0],
                            [-1, 0, 0, -1],
                            [0, 0, 1, 0.025],
                            [0, 0, 0, 1,]])

        Tce_grasp = np.array([[-1/np.sqrt(2), 0, 1/np.sqrt(2), 0],
                            [0, 1, 0, 0],
                            [-1/np.sqrt(2), 0, -1/np.sqrt(2), 0],
                            [0, 0, 0, 1]])

        Tce_standoff = np.array([[-1/np.sqrt(2), 0, 1/np.sqrt(2), -.025],
                            [0, 1, 0, 0],
                            [-1/np.sqrt(2), 0, -1/np.sqrt(2), .025],
                            [0, 0, 0, 1]])

        self.TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff,self.N)

    def get_input(self,path):
        """
        read initial conditions for csv file
        """
        print("Reading initial conditions.....")
        with open(path, newline='') as csvfile:
            spamreader = csv.reader(csvfile, delimiter=',', quotechar=',')

            for row in spamreader:
                temp_list = [float(item) for item in row]
                self.init_conditions.append(temp_list)
        time.sleep(0.2)
        print("Reading initial conditions Done")
        time.sleep(0.1)

