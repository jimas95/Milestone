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