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
    csv form chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper state
    """

    # Open a file for output
    # Overwrite
    f = open("data/" + name + ".csv", "w") 
    
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
            