import numpy as np
import csv
import time
import functions

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
        l = 0.47/2 
        w = .3/2
        
        H_mat = np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],[1,1,1,1],[-1,1,-1,1]])
        output_state[0:3] = 0.0475/4. * np.matmul(H_mat, velocity[5:]) * self.dt + input_state[0:3]
        #update joints and wheels configuration
        output_state[3:12] = input_state[3:12] + np.array(velocity[:])*self.dt
        #copy gripper state
        output_state[-1] = input_state[-1]
        return output_state

    def TrajectoryGenerator(self):
        """
        """
        pass


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

