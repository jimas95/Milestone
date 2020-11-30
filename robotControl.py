
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
        self.wheel_limit = 4
        self.joint_limit = 2

        #time step
        self.dt = 0.01




    def next_state(self,input_state,velocity):
        """
        Input :
            input_state --> 12-vector representing the current configuration of the robot 
                            3 variables for the chassis configuration,
                            5 variables for the arm configuration, and 4 variables for the wheel angles
            velocity    --> A 9-vector of controls indicating the arm joint speeds theta
                            (5 variables) and the wheel speeds u (4 variables).
        Output:
            output_state--> A 12-vector representing the configuration of the robot time Δt later
        """
        
        
        #check velocity limits 
        velocity = functions.check_limits(velocity,self.joint_limit,self.wheel_limit)
        #update state
        output_state = [0,0,0,0,0,0,0,0,0,0,0,0]

        #update chassis configuration which is obtained from odometry

        #update joints and wheels configuration
        for i in range(len(velocity)):
            value_new = input_state[i+3] + velocity[i]*self.dt
            output_state[i+3] = value_new
        return output_state

    def TrajectoryGenerator(self):
        """
        """
        pass


    def feedback_control(self):
        """
        """
        pass

    def run(self):
        """
        runs the whole thing....
        """
        print("Executing Run.....")
        input_state = [0,0,0,0,0,0,0,0,0,0,0,0]
        velocity    = [10,1,10,50,-10,20,-50,-100,10]
        out = self.next_state(input_state,velocity)
        print(out)
        out = self.next_state(out,velocity)
        print(out)
        out = self.next_state(out,velocity)
        print(out)
        out = self.next_state(out,velocity)
        print(out)






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
