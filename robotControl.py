import numpy as np
import csv
import time
import functions
import modern_robotics as mr
import myMatrix
import matplotlib.pyplot as plt


import logging
logger = logging.getLogger("my_log")
handler = logging.FileHandler('best.log')
formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)
logger.setLevel(logging.INFO)



class robotControler:

    def __init__(self):
        print("init robot Controler")

        path = "input.csv"
        self.init_conditions = []
        self.get_input(path)
        functions.display_init_conditions(self.init_conditions)


        #create object of all of our matrices we need
        self.allMatrix = myMatrix.allMatrix()
        
        
        #velocity limits
        self.wheel_limit = 5
        self.joint_limit = 5

        #time step
        self.dt = 0.01

        #wheel length 
        self.l = 0.47/2 
        self.w = .3/2
        self.r = 0.0475

        #Kp Ki control gains
        self.Kp = np.zeros((4,4))
        self.Ki = np.zeros((4,4))
        self.Kp = np.identity(4) * 3
        self.Ki = np.identity(4) * 0.1
        #intergal error for Ki control
        self.error_intergal = np.zeros((4,4))

        #trajectory
        self.order_method = 5
        self.N = 200


    def get_joint_vel(self,X,Xd,Xd_next,q):
        V_ee, error =  self.Feedforward_Control(X,Xd,Xd_next)
        Je_pinv, _ = self.calc_jacobian(q)
        vels = np.matmul(Je_pinv, V_ee)
        return vels,error

    def Feedforward_Control(self,X,Xd,Xd_next):
        """
        calculate the joint velocitys 
        based on the current eef configuration and the next(t+dt)  eef configuration
        Input: 
            The current actual end-effector configuration X
            The current end-effector reference configuration Xd
            The end-effector reference configuration at the next timestep in the reference trajectory,Xd_next at a time Δt later.
        Return:
            The eef twist and error
        """
        
        #calculate X error 
        X_err      = mr.MatrixLog6(np.matmul(mr.TransInv(X),Xd))
        X_next_err = mr.MatrixLog6(np.matmul(mr.TransInv(Xd), Xd_next))

        #calculate Vd
        Vd = mr.se3ToVec(1/self.dt * X_next_err )

        #calculate intergal error (kepp the sum in variable error_intergal)
        self.error_intergal = self.error_intergal + X_err*self.dt

        #calculate eef velocity twist 
        AdJoint_mat = mr.Adjoint(np.matmul(mr.TransInv(X),Xd))
        Kp_var = mr.se3ToVec(np.matmul(self.Kp,X_err))
        Ki_var = mr.se3ToVec(np.matmul(self.Ki,self.error_intergal))
        V = np.matmul(AdJoint_mat,Vd) + Kp_var + Ki_var
        return V, mr.se3ToVec(X_err)


    def calc_jacobian(self,q):
        """
        calculating the jacobian matrix 
        """
        T0e = mr.FKinBody(self.allMatrix.M0e,
                          self.allMatrix.Blist,
                          q[3:])

        Teb = np.matmul(mr.TransInv(T0e),mr.TransInv(self.allMatrix.Tb0))
        Ad = mr.Adjoint(Teb)
        Jbase = np.matmul(Ad,self.allMatrix.F6)
        J_arm = mr.JacobianBody(self.allMatrix.Blist, q[3:])
        J_e = np.concatenate((Jbase, J_arm),1)
        Je_pinv = np.linalg.pinv(J_e,1e-4)
        return Je_pinv, mr.TransInv(Teb)

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
        
        
        # #check velocity limits 
        velocity = functions.check_limits(velocity,self.joint_limit,self.wheel_limit)
        #update state
        output_state = np.zeros(13)
        #update chassis configuration which is obtained from odometry

        H_mat = self.allMatrix.H
        output_state[0:3] = 0.0475/4. * np.matmul(H_mat, velocity[:4]) * self.dt + input_state[0:3]
        #update joints and wheels configuration
        output_state[3:8] = input_state[3:8] + velocity[4:] * self.dt #arm velocities second part of thetadot
        output_state[8:12] = input_state[8:12] + velocity[:4] * self.dt #expects wheel velocity as first part       
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
        self.call_ScrewTrajectory(state_pregrasp    ,state_grasp    ,tf,2*N,self.order_method,0,trajectory)
        self.call_ScrewTrajectory(state_grasp       ,state_grasp    ,tf,N,self.order_method,1,trajectory)
        self.call_ScrewTrajectory(state_grasp       ,state_pregrasp ,tf,N,self.order_method,1,trajectory)
        self.call_ScrewTrajectory(state_pregrasp    ,state_preplace ,tf,3*N,self.order_method,1,trajectory)
        self.call_ScrewTrajectory(state_preplace    ,state_place    ,tf,N,self.order_method,1,trajectory)
        self.call_ScrewTrajectory(state_place       ,state_place    ,tf,N,self.order_method,0,trajectory)
        self.call_ScrewTrajectory(state_place       ,state_preplace ,tf,N,self.order_method,0,trajectory)
        # self.call_ScrewTrajectory(state_preplace    ,state_init     ,tf,N,self.order_method,0,trajectory)

        return trajectory
        

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
        elif(id==3):
            print("Executing test Feedforward Control")
            self.test_Feedforward_Control()
        elif(id==4):
            print("Executing pick and place!")
            self.test_matrix_convert()
        elif(id==5):
            print("Executing pick and place!")
            self.pick_and_place()
        elif(id==6):
            print("Executing pick and place!")
            # self.sakis()


    def pick_and_place(self):
        

        #get initial current configuration    
        q0 = self.init_conditions[2]

        #get refference current configuration & find eef position
        q_ref = self.init_conditions[3]
        _,Tbe = self.calc_jacobian(q=q_ref[:8])
        Tse_init = np.matmul(self.allMatrix.get_Tsb(q_ref),Tbe)

        #get initial cube position and placing postion
        Tsc_init = self.allMatrix.get_matrix(self.init_conditions[0])
        Tsc_final = self.allMatrix.get_matrix(self.init_conditions[1])

        #pregrasping and stanoff positions
        Tce_grasp =self.allMatrix.Tce_grasp
        Tce_standoff = self.allMatrix.Tce_standoff


            
        #generate trajectory
        trajectory = self.TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff,self.N)
        functions.write_csv(trajectory,"trajectory_")

        #initialize
        N = len(trajectory) -1        
        error_list = []

        # self.Kp = np.zeros((4,4))
        # self.Ki = np.zeros((4,4))

        data=[q0]
        for i in range(N):
            #set current configuration
            q = data[-1]
            
            #get matrices
            _,Tbe = self.calc_jacobian(q=q[:8])
            X = np.matmul(self.allMatrix.get_Tsb(q),Tbe)
            Xd = self.allMatrix.list_to_array(trajectory[i])
            Xd_next = self.allMatrix.list_to_array(trajectory[i+1])

            #call feedForward Control
            Vel,error  = self.get_joint_vel(X,Xd,Xd_next,q=q[:8])
            error_list.append(error) # keep a list with all errors
            
            #copy gripper state
            q[-1]=trajectory[i][-1]

            #call next state 
            out = self.next_state(q,Vel)
            data.append(out)

        #save output at csv format
        functions.write_csv(data,"pick_place")
        self.plotter(error_list)



    def plotter(self,error):
        """
        plot and save the total error
        """
        total_time = round(len(error)*self.dt,5)
        tvec = np.arange(0,total_time,self.dt)
        np_error = np.array(error)
        for i in range(6):
            plt.plot(tvec,np_error[:,i])
        plt.legend(['w_x','w_y','w_z','v_x','v_y','v_z'])
        plt.title("error over time")
        plt.ylabel("error")
        plt.xlabel("time [s]") 
        plt.savefig('error.png')
        plt.show()



    def test_Feedforward_Control(self):
        """
        test the Feedforward_Control
        """
        q0 = np.array([0,0,0,0,0,0.2,-1.6,0])

        self.Kp = np.zeros((4,4))
        self.Ki = np.zeros((4,4))
        self.dt = 0.01

        vels,error = self.get_joint_vel(self.allMatrix.test_X,
                                        self.allMatrix.test_Xd,
                                        self.allMatrix.test_Xd_next,
                                        q0)
        vels = np.around(vels,4)
        print("Got joint velocity again -->")
        print(vels)
        print("error:")
        print(error)

    def test_nextState(self):
        """
        test the next state function on scene 6
        create 3 csv files forward,sideways,sideways
        """
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
        """
        Test the generate trajectory , USE SCENE 8
        Tse_init     --> initial position of eef
        Tsc_init     --> initial position of cube
        Tsc_final    --> final   position of cube
        Tce_grasp    --> eef grasping position
        Tce_standoff --> use it go "above position" offset 
        """
        Tse_init = self.allMatrix.Tse_init
        Tsc_init = self.allMatrix.Tsc_init
        Tsc_final =self.allMatrix.Tsc_final
        Tce_grasp =self.allMatrix.Tce_grasp
        Tce_standoff = self.allMatrix.Tce_standoff

        trajectory = self.TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff,self.N)
        functions.write_csv(trajectory,"trajectory")

    def test_matrix_convert(self):
        q = [0,1,2,3,4,5,6,7,8,9,10,11,12]
        nothing = [] 
        matri = self.allMatrix.list_to_array(q)
        print(q)
        print(matri)
        functions.convertToCsvForm(nothing,[matri,matri],12)
        print(nothing)
        matri = self.allMatrix.list_to_array(nothing[-1])
        print(matri)
        functions.convertToCsvForm(nothing,[matri,matri],12)
        print(nothing)       

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

