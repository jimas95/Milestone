def display_init_conditions(init_conditions):
    """
    display the initial conditions
    """

    for i in range(len(init_conditions)):
        print(" ")
        if i==0:
            print("the initial resting configuration of the cube object ")
        elif i==1:
            print("the desired final resting configuration of the cube object")
        elif i==2:
            print("the actual initial configuration of the youBot")
        elif i==3:
            print("the reference initial configuration of the youBot")
        print(init_conditions[i])


def write_csv(data,name):
    """
    save data as csv file
    for robot state:
        csv form chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper state
    for trajectory :
        csv form r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz,gripper_state
    """

    # Open a file for output
    # Overwrite
    file_name = "data/" + name + ".csv"
    print("writting file : "+file_name)
    f = open(file_name, "w") 
    
    for row in data:
        output = ""
        for i in range(len(row)):
            output += str(row[i]) 
            if(i!=len(row)-1):
                output +=", "
        
        output += " \n"
        f.write(output)
        
    # close file
    f.close()

def convertToCsvForm(myList,data,gripper_state):
    """
    basicaly converts a 4x4 matrix to a list thats for csv format.
    data contains a trajectory, N transformations of 4x4 matrix
    appends myList with the values that we are intrested from the 4x4 matrix, in order to right them as csv
    """
    for row in data:
        r11 = row[0][0]
        r12 = row[0][1]
        r13 = row[0][2]
        r21 = row[1][0]
        r22 = row[1][1]
        r23 = row[1][2]
        r31 = row[2][0]
        r32 = row[2][1]
        r33 = row[2][2]
        px  = row[0][3]
        py  = row[1][3]
        pz  = row[2][3]
        myList.append([r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz,gripper_state])

def check_limits(input_state,joint_limit,wheel_limit):
    """
    check if any velocity exdeeds limits, if yes set the maximu velocity limit
    """
    for i in range(5):
        if(input_state[i]>joint_limit):
            input_state[i] = joint_limit
        if(input_state[i]<-joint_limit):
            input_state[i] = -joint_limit

    for i in range(4,9):
        if(input_state[i]>wheel_limit):
            input_state[i] = wheel_limit
        if(input_state[i]<-wheel_limit):
            input_state[i] = -wheel_limit
    return input_state
            